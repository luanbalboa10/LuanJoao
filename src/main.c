#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <string.h>

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define TAMANHO_PILHA 1024 // Define o tamanho da pilha para as threads
#define TAMANHO_MENSAGEM 8 // Tamanho da mensagem em bits
#define GPIO "GPIOB"
#define GPIO_TX 0x3 // PTB3 (pino GPIO de transmissão)
#define GPIO_RX 0x2 // PTB2 (pino GPIO de recepção)
#define T 20 // Tempo

K_MSGQ_DEFINE(teclado_fila, TAMANHO_MENSAGEM, 10, 4); // Define uma message queue 

K_CONDVAR_DEFINE(sync_cond); // Define as variáveis condicionais
K_CONDVAR_DEFINE(comp_cond);

K_MUTEX_DEFINE(sync_mux); // Define um mutex para utilizar as variáveis condicionais
K_MUTEX_DEFINE(comp_mux);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
const struct device *dispositivo_gpio; 
volatile int variavel = 0; 

// Protótipo das funções
void start_gpio(void);
void start_uart(void);
void serial_cb(const struct device *dev, void *user_data);
void enviar_mensagem_thread(void);
void recepcao_thread(void);
void comparador_thread(void);
int vetor_para_int(int *vetor, int tamanho);
void imprimir_mensagem(int mensagem_buf[56], int tm, int id_buf[5]);


// ------------- Configurações ------------- //

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

// ------------- Função serial ------------- //

// Função de callback da interrupção na UART 
void serial_cb(const struct device *dev, void *user_data)
{
    // Pacote de dados
    int U = 85;
    int sync = 65;
    int STX = 66;
    int cabecalho = 0xD0;
    int end = 67;
    
    int n; // Variável para guardar o tamanho em bytes da mensagem
    uint8_t c; // Variável que guarda aquilo que vem da UART
    static int tx_buf[TAMANHO_MENSAGEM]; // Buffer para guardar a mensagem
    static int tx_buf_pos; // Buffer que armazena o último índice do tx_buf

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }


    // Lê até que a FIFO esteja vazia 
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if ((c == '\n' || c == '\r') && tx_buf_pos > 0) {
            n = tx_buf_pos;
            tx_buf[tx_buf_pos] = '\0';
            cabecalho = cabecalho | n; // (11010 << 3) | n -> Constrói o cabecalho
            
            // Envio de pacote de dados na fifo teclado_fila
            k_msgq_put(&teclado_fila, &U, K_FOREVER);  // Envia U
            k_msgq_put(&teclado_fila, &sync, K_FOREVER); // Envia sync
            k_msgq_put(&teclado_fila, &STX, K_FOREVER); // Envia STX
            k_msgq_put(&teclado_fila, &cabecalho, K_FOREVER); // Envia cabecalho
            for(int i = 0; i < tx_buf_pos; i++) {
                k_msgq_put(&teclado_fila, &tx_buf[i], K_FOREVER); // Envia a mensagem digitada no teclado
            }
            k_msgq_put(&teclado_fila, &end, K_FOREVER); // Envia end

            // Envia o sinal para a thread "enviar_mensagem_thread" iniciar a transmissão
            k_condvar_signal(&sync_cond); 

            tx_buf_pos = 0; // Zera o contador de posição do buffer

        } else if (tx_buf_pos < (sizeof(tx_buf) - 1)) {
            tx_buf[tx_buf_pos++] = c; // Pega o que veio da uart e guarda em tx_buf
        }
        
    }
}

// ------------- Threads ------------- //

// Envia a mensagem byte a byte - bit a bit
void enviar_mensagem_thread(void) 
{
    int dado;
    while (1) {
        if (k_msgq_get(&teclado_fila, &dado, K_FOREVER) == 0) { // Confirma se o dado foi retirado da fifo com sucesso (retorna 0 quando retira o dado)
            uint8_t byte = (uint8_t)dado; // Converte a variável dado (int) na variável byte (uint8_t)
            for (int i = 7; i >= 0; i--) {  // Percorre os bits do mais significativo ao menos significativo
                gpio_pin_set(dispositivo_gpio, GPIO_TX, (byte >> i) & 1); // Seta o pino GPIO de acordo com o bit da mensagem
                //printk(" %d ", (byte >> i) & 1);
                k_sleep(K_MSEC(4*T)); // Tempo 4 vezes maior que o de recepção
            }
        }
    }
}

void recepcao_thread(void) {

    int rx_buf[4];
    int j = 1;
    printk("Digite algo no teclado para começar: ");
    k_condvar_wait(&sync_cond, &sync_mux, K_FOREVER);

    while (1) {
        for (int i = 0; i < 4; i++) {
            rx_buf[i] = gpio_pin_get(dispositivo_gpio, GPIO_RX); // Recebe 4 vezes o mesmo bit e guarda em um buffer
            //printk(" %d ", rx_buf[i]);
            k_sleep(K_MSEC(T)); 
        } 
    
        if (rx_buf[j] == rx_buf[j+1]){
            variavel = rx_buf[j]; // valida o bit
            //printk(" var = %d ", variavel);
            k_condvar_signal(&comp_cond); 
        }
        memset(rx_buf, 0, 32); // zera o buffer
    }
}

void comparador_thread(void) {
    
    int mensagem_buf[56];
    static int id_buf[5];
    static int n_buf[3];
    int resp = 0;

    // Pacote de dados
    int U = 85;
    int sync = 65;
    int STX = 66;
    int end = 67;

    int count = 0; 
    int count2 = 0;
    int count3 = 0;
    int count4 = 0;
    int count5 = 0;
    int count6 = 0;
    int tm = 0;

    while (1) {
        k_condvar_wait(&comp_cond, &comp_mux, K_FOREVER);
        
        if (count < 8) {    // compara o 1º byte (SYNC)
            //printk(" sync ");
            if (variavel == (sync >> (7 - count) & 0x01)) { 
                //printk(" %d \n", (sync >> (7 - count) & 0x01)); 
                count++;
            } 
            else {
                //printk(" erro ");
                //printk(" %d \n", (sync >> (7 - count) & 0x01));
                count = 0; // zera caso seja incompatível
            }
        } else if (count < 16 && count > 7) { 
            //printk(" stx ");  // compara o 2º byte (STX)
            if (variavel == (STX >> (7 - count2) & 0x01)) { 
                //printk(" %d ", (STX >> (7 - count2) & 0x01)); 
                count2++;
                count++;
                //printk(" count2: %d , count: %d \n", count2, count);
            } 
            else {
                //printk(" erro ");
                //printk(" %d ", (STX >> (7 - count2) & 0x01));
                //printk(" count2: %d , count: %d \n", count2, count);
                count = 0;
                count2 = 0;
                
            }
        } 

        else if (count > 15 && count < 21) { 
            //printk(" ID \n"); // guarda o ID do usuário transmissor 
            id_buf[count3] = variavel;
            count3++;
            count++;
        }

        else if (count > 20 && count < 24) {  
            //printk(" n \n");  // guarda o tamanho da mensagem
            n_buf[count4] = variavel; 
            count4++;
            count++;
            if (count == 24) { 
                //printk(" tm \n");// calcula o tamanho da mensagem em bits
                resp = vetor_para_int(n_buf, 3);
                tm = resp*8;
            }
        } 

        // coloca a mensagem no buffer
        else if (count > 23 && (count < tm + 24)) {
            //printk(" mensagem \n");
            mensagem_buf[count5] = variavel;
            count5++;
            count++;
        }

        else if ((count > tm + 23)  && (count < tm + 31)) {   
            //printk(" end "); // compara o último byte (end)
            if (variavel == (end >> (7 - count6) & 0x01)) { 
                //printk(" %d \n", (end >> (7 - count6) & 0x01)); 
                count6++;
                count++;
            } 
            else {
                //printk(" erro ");
                //printk(" %d \n", (end >> (7 - count6) & 0x01));
                count = 0;
                count4 = 0;
                count5 = 0;
                count6 = 0;
            }
        } 

        else if (count > tm + 30) { // caso tudo ok, imprime no monitor
            imprimir_mensagem(mensagem_buf, tm, id_buf);
            count = 0;
            count2 = 0;
            count3 = 0;
            count4 = 0;
            count5 = 0;
            count6 = 0;
        }
    }
}

// ------------- Ferramentas ------------- //

// Converte bits de um vetor para inteiro
int vetor_para_int(int *vetor, int tamanho) { 
    int valor = 0; 
    for (int i = 0; i < tamanho; i++) {
        valor |= (vetor[i] << tamanho - 1 - i); 
    }
    return valor;
} 

// Imprime a mensagem no monitor
void imprimir_mensagem (int mensagem_buf[56], int tm, int id_buf[5]) {

    int id = vetor_para_int(id_buf, 5);
    printk("\nSeu amigo (ID = %d) enviou esta mensagem: ", id); 
    for (int i = 0; i < tm; i += 8) {
        uint8_t byte = 0;
        for (int j = 0; j < 8; j++) {
            byte |= (mensagem_buf[i + j] << (8 - 1 - j));
        }
        char c = (char)byte;
        printk("%c", c);
    }
    printk("\n");
}

// Definição das threads 
K_THREAD_DEFINE(enviar_tid, TAMANHO_PILHA, enviar_mensagem_thread, NULL, NULL, NULL, 0, 0, 0);
K_THREAD_DEFINE(recepcao_tid, TAMANHO_PILHA, recepcao_thread, NULL, NULL, NULL, 0, 0, 0);
K_THREAD_DEFINE(comparador_tid, TAMANHO_PILHA, comparador_thread, NULL, NULL, NULL, 0, 0, 0);

// Main
void main(void)
{
    start_gpio(); 
    start_uart();

    while (1) {
        k_sleep(K_MSEC(T));
    }
}