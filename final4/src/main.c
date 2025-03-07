#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define TAMANHO_PILHA 1024 // Define o tamanho da pilha para as threads
#define TAMANHO_MENSAGEM 8 // Tamanho da mensagem em bits
#define GPIO "GPIOB" // Define GPIO para GPIOB
#define GPIO_TX 0x3 // PTB3 (pino GPIO de transmissão)
#define GPIO_RX 0x2 // PTB2 (pino GPIO de recepção)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
const struct device *dispositivo_gpio; 

K_MSGQ_DEFINE(teclado_fila, TAMANHO_MENSAGEM, 12, 4); // Define a message queue utilizada para guardar a mensagem retirada da serial

K_FIFO_DEFINE(cabecalho_fifo); // Define a fifo utilizada para guardar a mensagem junto com o cabeçalho e sincronizar com a thread de empacotamento
K_FIFO_DEFINE(mensagem_fifo); // Define a fifo utilizada para guardar a mensagem recebida e sincronizar com a thread de impressão

K_CONDVAR_DEFINE(denovo_cond); // Define a variável condicional para sincronizar um novo envio (serial e empacota_thread)
K_MUTEX_DEFINE(denovo_mux); 

K_CONDVAR_DEFINE(comp_cond); // Define a variável condicional que sincroniza a comparação de bytes recebidos (recepcao e comparador_thread)
K_MUTEX_DEFINE(comp_mux);

K_CONDVAR_DEFINE(imprimir_cond); // Define a variável condicional que sincroniza a impressão da mensagem, permitindo que a thread só imprima depois de comparar toda mensagem (comparador_thread e imprimir_thread)
K_MUTEX_DEFINE(imprimir_mux);

K_MUTEX_DEFINE(tx_mux); // Define o mutex que sincroniza a thread de transmissão com a função que seta o pino gpio 
K_MUTEX_DEFINE(nome_mux); // Define o mutex que sincroniza 
 

K_SEM_DEFINE(my_sem, 0, 1); // Define um semáforo para sinalizar a thread empacota que a mensagem já foi lida na serial (entre serial e empacota) 

volatile int count = 0; 
volatile uint8_t n = 0;
int pacote[97], j = 0, k = 0, x = 0, shiftou = 0, id_rx = 0;
int receb_buf[4];
int comp_buf[8];
bool flag = 0;

// Protótipo das funções 
void escrever_fifo(char dado, struct k_fifo *my_fifo);
char ler_fifo(struct k_fifo *my_fifo);
void start_gpio(void);
void start_uart(void);
void serial_cb(const struct device *dev, void *user_data);
int empacota(char byte, int cont);
void transmissao(struct k_timer *timer);
void recepcao(struct k_timer *timer);
int destrincha_id(char cabecalho);
void shift_direita(int *buf, int tam);
void shift_esquerda(int *buf, int tam);
int vetor_para_int(int *vetor, int tamanho);

K_TIMER_DEFINE(tx_timer, transmissao, NULL); // Define os timers
K_TIMER_DEFINE(rx_timer, recepcao, NULL);



/* ----------------------------------------------------------------- */

// Estrutura da fifo 
struct data_item_t 
{ 
    void *fifo_reserved;   // Necessário pela FIFO 
    char value;            // Valor armazenado 
};

// Escreve na fifo
void escrever_fifo(char dado, struct k_fifo *my_fifo)
{
    struct data_item_t *item = k_malloc(sizeof(struct data_item_t)); // Aloca memória para o item da fifo
    if (!item) { 
        printk("Erro: Falha ao alocar memória!\n");
        return;
    }
    //printk(" escrevi na fifo ");
    //printk(" dado = %c \n", dado);
    item->value = dado; // Coloca o valor desejado no struct
    k_fifo_put(my_fifo, item); // Coloca na fifo
}

// Lê a fifo
char ler_fifo(struct k_fifo *my_fifo)
{
    struct data_item_t *item = k_fifo_get(my_fifo, K_FOREVER); // Pega o valor da fifo
    char value = item->value; // Guarda numa variável
    k_free(item); // Libera espaço na memória
    return value; // Retorna o valor
}

/* ----------------------------------------------------------------- */

// Configura GPIO
void start_gpio(void)
{
    dispositivo_gpio = DEVICE_DT_GET(DT_NODELABEL(gpiob));
    gpio_pin_configure(dispositivo_gpio, GPIO_TX, GPIO_OUTPUT_LOW);
    gpio_pin_configure(dispositivo_gpio, GPIO_RX, GPIO_INPUT | GPIO_ACTIVE_HIGH);
}

// Configura UART
void start_uart(void)
{
    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);
}

/*  ----------------- Transmissão  ----------------- */

// Função de callback da interrupção na UART 
void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c; // Variável que guarda aquilo que vem da UART
    static int tx_buf[TAMANHO_MENSAGEM]; // Buffer para guardar a mensagem
    static int tx_buf_pos; // Buffer que armazena o último índice do tx_buf
    uint8_t id = 0b00100000;

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }


    // Lê até que a FIFO esteja vazia 
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        

        if ((c == '\n' || c == '\r') && tx_buf_pos > 0) {
            if (flag == 1) {
                printk(" Aguarde a transmissão ser finalizada para enviar uma nova mensagem! \n"); // Se existe uma transmissão simultânea, não permite que uma nova mensagem seja enviada
                memset(tx_buf, 0, sizeof(tx_buf));
                n = 0;
                tx_buf_pos = 0;
            }
            else if(tx_buf_pos > 7) { // Se a mensagem possuir mais que 7 caracteres, impede e imprime "ERRO"
                printk("ERRO\n");
                memset(tx_buf, 0, sizeof(tx_buf));
                tx_buf_pos = 0;
                n = 0;
                printk("Envie outra mensagem: ");
            } 
            else{ // Caso tudo esteja certo
                tx_buf[tx_buf_pos] = '\0';

                uint8_t aux = id | n;
                char cabecalho = (char)aux;
                //printk(" cabecalho = %c \n", cabecalho);
                escrever_fifo(cabecalho, &cabecalho_fifo); // Guarda cabeçalho na fifo 
                
                // Envio de pacote de dados na fifo teclado_fila
                for(int i = 0; i < tx_buf_pos; i++) {
                    k_msgq_put(&teclado_fila, &tx_buf[i], K_NO_WAIT); // Envia a mensagem digitada no teclado
                }

                k_timer_start(&tx_timer, K_MSEC(10), K_MSEC(10)); // Inicializa o timer de transmissão
                k_condvar_signal(&denovo_cond); // Envia o sinal para uma nova mensagem
                k_sem_give(&my_sem); // Sinaliza através do semáforo que a mensagem da serial já está na message queue
                
                flag = 1; // Sinaliza que existe uma transmissão ocorrendo
                tx_buf_pos = 0; // Zera o contador de posição do buffer 
            }
            
        } else if (tx_buf_pos < (sizeof(tx_buf) - 1)) {
            n++;
            tx_buf[tx_buf_pos++] = c; // Pega o que veio da uart e guarda em tx_buf
        }
    }
}

// Thread para colocar a mensagem digitada no teclado na fifo "cabecalho_fifo"
void coloca_fifo_thread(void)
{
    
    char tx1_buf[TAMANHO_MENSAGEM]; 
    int u = 0;
    
    while (1) {
        k_mutex_lock(&nome_mux, K_FOREVER); // Se não tem mensagem sendo empacotada 
        while (k_msgq_get(&teclado_fila, &tx1_buf[u], K_FOREVER) == 0) { // Se mensagem está na message queue
            //printk(" tx1_buf = %c \n", tx1_buf[u]);
            escrever_fifo(tx1_buf[u], &cabecalho_fifo); // Coloca a mensagem na fifo para ser enviada
            u++;
            if (u == n) { // Se toda mensagem foi retirada da message queue, libera o mutex
                u = 0;
                n = 0;
                k_mutex_unlock(&nome_mux); 
            }
        } 
    }
}

// Thread para empacotar (colocar cada byte no buffer "pacote")
void empacota_thread(void)
{
    // Pacote de dados
    char U = 85;
    char sync = 65;
    char STX = 66;
    char end = 67;

    char julia;

    while (1) {
            k_mutex_lock(&tx_mux, K_FOREVER); // Se não tem mensagem sendo transmitida
            k_sem_take(&my_sem, K_FOREVER); // Se existe uma mensagem enviada na serial
            k_mutex_lock(&nome_mux, K_FOREVER); // Se mensagem e cabeçalho já estão na fifo
            int resp = empacota(U, 0); // Coloca U, sync e STX no buffer de envio
            resp = empacota(sync, resp);
            resp = empacota(STX, resp);
            
            while (k_fifo_is_empty(&cabecalho_fifo) == 0) { // Enquanto a fifo da mensagem e cabeçalho não estiver vazia
                julia = ler_fifo(&cabecalho_fifo); // Coloca cabeçalho e mensagem no buffer para serem enviados
                resp = empacota(julia, resp);
            }  

            resp = empacota(end, resp); // Coloca end no buffer de envio
            pacote[resp] = 3; // Sinaliza na última posição do buffer que o pacote acabou
            k_mutex_unlock(&tx_mux); // Libera mutex para transmissão
            k_mutex_unlock(&nome_mux); // Libera mutex para nova mensagem
            k_condvar_wait(&denovo_cond, &denovo_mux, K_FOREVER); // Aguarda novo sinal para empacotar outra mensagem
            memset(pacote, 0, sizeof(pacote)); // Zera o buffer que guarda o pacote já transmitido
        }
        
}

// Guarda bit a bit os bytes do pacote no buffer 
int empacota(char byte, int cont) 
{
    uint8_t dado = (uint8_t)byte; 
    //printk(" dado = %d \n", dado);
    for (int i = 7; i >= 0; i--) { 
        //printk(" dado = %d \n", dado);
        pacote[cont] = ((dado >> i) & 1); // Seta o pino GPIO de acordo com o bit da mensagem
        cont++;
    } 
    return cont;
}

void transmissao(struct k_timer *timer)
{
    if (k_mutex_lock(&tx_mux, K_NO_WAIT) == 0) { // Se não tem pacote sendo formado
        //printk(" transmissão pegou \n");
        if (pacote[j] != 3) { // Enquanto o pacote não chegar ao fim
            gpio_pin_set(dispositivo_gpio, GPIO_TX, pacote[j]); // Seta o pino de acordo com o bit
            //printk(" pacote = %d \n", pacote[j]);
            j++;
            
        }
        else { // Se o pacote chegou ao fim
            j = 0; // Zera o contador para uma nova transmissão
            //printk(" fim da transmissão ");
            k_timer_stop(&tx_timer); // Para o timer
            k_mutex_unlock(&tx_mux); // Libera o mutex para um novo pacote
            flag = 0;
        }
        k_mutex_unlock(&tx_mux); // Libera o mutex para sincronizar
    }
    
}

/*  ----------------- Recepção  ----------------- */
void recepcao(struct k_timer *timer)
{
    //printk(" receb_buf = %d ", receb_buf[0]);
    if (x < 4) {
        receb_buf[x] = gpio_pin_get(dispositivo_gpio, GPIO_RX); // Pega os primeiros 4 bits recebidos e guarda no buffer para validação
        x++;
    }
    if (x == 4) { // Depois de guardar os 4 primeiros bits, valida-os
        if (receb_buf[1] == receb_buf[2]) { // Se for um bit válido
            shiftou = 0; // Sinaliza que o bit é válido
            //printk(" receb_buf = %d \n", receb_buf[1]);
            comp_buf[7] = receb_buf[1]; // Guarda no buffer de validação dos bytes do pacote recebido
            count++; // Incrementa o contador para identificar os 8 primeiros bits validados
            k_condvar_signal(&comp_cond); // Sinaliza para a thread de validação dos bytes que existe um byte válido
            x = 0; // Reinicializa o contador do buffer de validação de bits
        } 
        else { // Se o bit não for válido
            shift_direita(receb_buf, 4); // Shifta o buffer, eliminando o bit mais a direita
            if (shiftou == 1) { 
                receb_buf[0] = gpio_pin_get(dispositivo_gpio, GPIO_RX); // Guarda um novo bit no buffer para ser validado com os outros três anteriores
            }
            shiftou = 1;
        }
    }

}

// Faz a verificação do pacote byte a byte
void comparador_thread(void)
{
    char sync = 65;
    char STX = 66;
    char end = 67;

    int l = 0, q = 0, n = 0;
    while (1) {
        k_condvar_wait(&comp_cond, &comp_mux, K_FOREVER); // Aguarda a sinalização de um byte válido
        int var = vetor_para_int(comp_buf, 8); // Converte o buffer em int
        //printk(" var = %d \n", var);
        
        if ((l == 0) && (count > 7)) { // Compara o primeiro byte válido com sync
            if (var == sync) {
                //printk(" sync \n");
                count = 0;
                l = 1;
            }
        }

        else if ((l == 1) && (count > 7)) { // Compara o segundo byte válido com STX
            if (var == STX) {
                //printk(" stx \n");
                count = 0;
                l = 2;
            }
            else { // Se não for validado, reinicia toda a validação
                l = 0; 
            }
        }
        else if ((l == 2) && (count > 7)) { // Se STX for validado, guarda o byte de cabeçalho 
            //printk(" cabecalho\n");
            char cabecalho = (char)var; // Converte a variável int em char
            //printk(" cabecalho = %c \n", cabecalho);
            n = destrincha_id(cabecalho); // Separa os cinco primeiros bits para definir o ID do transmissor e os 3 últimos para definir o tamanho da mensagem (retorna o tamanho da mensagem)
           // printk(" n = %d ", n);
            l = 3;
            count = 0;
        }
        else if ((l == 3) && (q < n) && (count > 7)) { // Depois de guardar cabeçalho, guarda os n bytes da mensagem na fifo 
            //printk(" mensagem \n"); // mensagem
            char dado = (char)var; // Converte a variável int em char
            escrever_fifo(dado , &mensagem_fifo);
            q++;
            count = 0;
            if (q == n) { 
                l = 4;
            }
        }
        else if ((l == 4) && (count > 7)) { // Depois de guardar toda a mensagem, valida o byte de end
            if (var == end) {
                q = 0;
                n = 0;
                count = 0;
                l = 0; 
                //printk(" end \n");
                k_condvar_signal(&imprimir_cond);
                //printk(" count = %d \n", count);
            }
            else { // Se não for validado, reinicializa a validação
                l = 0;
            }
        }

        shift_esquerda(comp_buf, 8); // Shifta a esquerda o buffer de validação, caso o byte não seja validado

    }
}

// Imprime a mensagem
void imprimir_thread(void) 
{ 
    
    while (1) {
        k_condvar_wait(&imprimir_cond, &imprimir_mux, K_FOREVER);
        printk(" Seu amigo (Id = %d) enviou esta mensagem: ", id_rx);
        while (k_fifo_is_empty(&mensagem_fifo) == 0) { // Se não tiver vazio
            printk("%c", ler_fifo(&mensagem_fifo));
        }  
        printk("\n");
        printk("Digite uma nova mensagem: ");
    }
   
}

// Separa o cabeçalho em id e n (retorna n)
int destrincha_id(char cabecalho)
{
    uint8_t valor = (uint8_t) cabecalho;  
    id_rx = valor >> 3;             
    int n = valor & 0x07;        
    return n;
}

// Shifta o buffer uma vez para a esquerda
void shift_esquerda(int *buf, int tam) 
{
    for (int i = 0; i < tam - 1; i++) {
        buf[i] = buf[i + 1];
    }
    buf[tam - 1] = 0; 
}

// Shifta o buffer uma vez para a direita
void shift_direita(int *buf, int tam) 
{ 
    for (int i = tam - 1; i > 0; i--) {
        buf[i] = buf[i - 1];
    }
    buf[0] = 0; 
}

// Converte bits de um vetor para inteiro
int vetor_para_int(int *vetor, int tamanho) 
{ 
    int valor = 0; 
    for (int i = 0; i < tamanho; i++) {
        valor |= (vetor[i] << (tamanho - 1 - i)); 
    }
    return valor;
} 

// Definição das threads 
K_THREAD_DEFINE(empacota_id, TAMANHO_PILHA, empacota_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(coloca_id, TAMANHO_PILHA, coloca_fifo_thread, NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(comparador_id, TAMANHO_PILHA, comparador_thread, NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(imprimir_id, TAMANHO_PILHA, imprimir_thread, NULL, NULL, NULL, 2, 0, 0);

int main(void)
{
    start_gpio(); // Configura gpio
    start_uart(); // Configura uart
    
    k_timer_start(&rx_timer, K_USEC(2500), K_USEC(2500)); // Inicializa o timer de recepção 

    printk("Digite sua mensagem: ");

    while (1) {
        k_sleep(K_MSEC(1000)); 
    }

    return 0;
}