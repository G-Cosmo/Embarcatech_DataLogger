#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "hardware/adc.h"
#include "hardware/rtc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"


#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"
#include "lib/sd_functions.h"

#define ADC_PIN 26 // GPIO 26

// Definição dos pinos I2C para o MPU6050
#define I2C_PORT i2c0                 // I2C0 usa pinos 0 e 1
#define I2C_SDA 0
#define I2C_SCL 1

// Definição dos pinos I2C para o display OLED
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define ENDERECO_DISP 0x3C            // Endereço I2C do display

#define ButtonB 6
#define ButtonA 5

uint32_t last_time = 0;

ssd1306_t ssd;
bool cor = true;
char op[50];
char bytewrite[50];
uint samples = 128;
char str_samples[15];

static bool logger_enabled;
static const uint32_t period = 1000;
static absolute_time_t next_log_time;

static char filename[20] = "mpu_data1.csv";

// Endereço padrão do MPU6050
static int addr = 0x68;

int16_t acceleration[3], gyro[3], temp;

int led[3] = {13,11,12};

bool interrupt_flag = false; //se false pode interromper, se true operação em andamento
bool mounted = false;
bool button_a_pressed = false;
bool button_b_pressed = false;

volatile int opt = -1;


void init_leds()
{
    gpio_init(led[0]);
    gpio_init(led[1]);
    gpio_init(led[2]);

    gpio_set_dir(led[0], GPIO_OUT);
    gpio_set_dir(led[1], GPIO_OUT);
    gpio_set_dir(led[2], GPIO_OUT);
}

static void mpu6050_reset()
{
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    sleep_ms(10);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    val = 0x43;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    val = 0x41;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

void capture_mpu_data_and_save()
{
    printf("\nCapturando dados do Sensor MPU. Aguarde finalização...\n");
    FIL file;
    FRESULT res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        printf("\n[ERRO] Não foi possível abrir o arquivo para escrita. Monte o Cartao.\n");
        return;
    }
    
    // Escrever cabeçalho CSV
    char header[] = "Sample,Acc.X,Acc.Y,Acc.Z,Gyro.X,Gyro.Y,Gyro.Z,Temp\n";
    UINT bw;
    res = f_write(&file, header, strlen(header), &bw);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível escrever cabeçalho no arquivo.\n");
        f_close(&file);
        return;
    }
    
    // Escrever dados
    for (int i = 0; i < samples; i++)
    {
        mpu6050_read_raw(acceleration, gyro, &temp);

        float temp_celsius = (temp / 340.0) + 36.53;

        char buffer[150];
        sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%.2f\n", i + 1,
             acceleration[0], acceleration[1], acceleration[2], 
             gyro[0], gyro[1], gyro[2], 
             temp_celsius); // Vírgula como separador (formato csv) e coloca no buffer os dados de cada eixo do sensor
        
        res = f_write(&file, buffer, strlen(buffer), &bw);
        if (res != FR_OK)
        {
            printf("[ERRO] Não foi possível escrever no arquivo. Monte o Cartao.\n");
            f_close(&file);
            return;
        }

        if (bw != strlen(buffer))
        {
            printf("[AVISO] Nem todos os bytes foram escritos na amostra %d.\n", i + 1);
        }

        sprintf(bytewrite, "%d", bw);

        sleep_ms(100);
    }
    f_close(&file);
    printf("\nDados do MPU salvos no arquivo CSV %s.\n\n", filename);
}

void print_error()
{
    ssd1306_fill(&ssd, !cor);
    ssd1306_draw_string(&ssd, op, 4, 8);
    ssd1306_draw_string(&ssd, "Erro Inesperado", 0, 50);
    ssd1306_send_data(&ssd);
    err_flag = false;
}

void print_op(bool done)
{
    if(done)
    {
        ssd1306_fill(&ssd,!cor);
        ssd1306_draw_string(&ssd, op, 4, 8);
        ssd1306_draw_string(&ssd, "Concluido!", 4, 50);
        ssd1306_send_data(&ssd);
    }else{
            ssd1306_fill(&ssd, !cor);
            ssd1306_draw_string(&ssd, op, 4, 8);
            ssd1306_send_data(&ssd);
    }
}

void clear_led()
{
    gpio_put(led[0], false);
    gpio_put(led[1], false);
    gpio_put(led[2], false);
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    uint32_t new_time = to_ms_since_boot(get_absolute_time());

    if(new_time - last_time > 500 && !interrupt_flag)
    {
        if(gpio == ButtonA)
        {
            button_a_pressed = true;
        }
        else if(gpio == ButtonB)
        {
            button_b_pressed = true;
        }
        
        last_time = new_time;
    }
}


int main()
{
    gpio_init(ButtonB);
    gpio_set_dir(ButtonB, GPIO_IN);
    gpio_pull_up(ButtonB);

    gpio_init(ButtonA);
    gpio_set_dir(ButtonA, GPIO_IN);
    gpio_pull_up(ButtonA);

    gpio_set_irq_enabled_with_callback(ButtonB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(ButtonA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();
    init_leds();

    gpio_put(led[0], true);
    gpio_put(led[1], true);

    sleep_ms(3000);
    time_init();


    // Inicializa a I2C do Display OLED em 400kHz
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);


    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO_DISP, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Limpa o display
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Inicialização da I2C do MPU6050
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    printf("\033[2J\033[H"); // Limpa tela
    printf("\n> ");
    stdio_flush();
    //    printf("A tela foi limpa...\n");
    //    printf("Depois do Flush\n");

    run_help();
    ssd1306_draw_string(&ssd, "Aguardando", 4, 4);
    ssd1306_draw_string(&ssd, "Comando...", 4, 13);
    ssd1306_send_data(&ssd);

    gpio_put(led[0], false);

    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
    mpu6050_reset();
    
    while (true)
    {

        if(button_a_pressed && !interrupt_flag)
        {
            opt = 'f';
            button_a_pressed = false; // Limpa o flag
        }
        
        if(button_b_pressed && !interrupt_flag)
        {
            if(mounted)
            {
                opt = 'b';
            }
            else
            {
                opt = 'a';
            }
            button_b_pressed = false; // Limpa o flag
        }

        // Processa entrada serial
        int serial_input = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != serial_input)
        {
            opt = serial_input;
            process_stdio(opt);
        }

        if (opt == 'a') // Monta o SD card se pressionar 'a'
        {
            printf("\nMontando o SD...\n");
            strcpy(op, "Montando SD...");
            print_op(false);

            clear_led();
            gpio_put(led[0], true);
            gpio_put(led[1], true);

            interrupt_flag = true;

            run_mount();

            if(err_flag)
            {
                print_error();
                clear_led();
                gpio_put(led[0], true);

            }else
            {
                print_op(true);
                clear_led();
                gpio_put(led[1], true);
            }

            interrupt_flag = false;
            mounted = true;

            opt = -1;
            printf("\nEscolha o comando (h = help):  ");
        }
        if (opt == 'b') // Desmonta o SD card se pressionar 'b'
        {
            printf("\nDesmontando o SD. Aguarde...\n");
            strcpy(op, "Desmontando SD...");
            print_op(false);

            clear_led();
            gpio_put(led[0], true);
            gpio_put(led[1], true);

            interrupt_flag = true;

            run_unmount();

            if(err_flag)
            {
                print_error();
                clear_led();
                gpio_put(led[0], true);

            }else
            {
                print_op(true);
                clear_led();
                gpio_put(led[1], true);
            }

            interrupt_flag = false;
            mounted = false;
        
            opt = -1;
            printf("\nEscolha o comando (h = help):  ");
        }
        if (opt == 'c') // Lista diretórios e os arquivos se pressionar 'c'
        {
            printf("\nListagem de arquivos no cartão SD.\n");
            strcpy(op, "Listando arquivos...");
            print_op(false);

            clear_led();
            gpio_put(led[0], true);
            gpio_put(led[1], true);

            interrupt_flag = true;

            run_ls();

            if(err_flag)
            {
                print_error();
                clear_led();
                gpio_put(led[0], true);
            }else
            {
                print_op(true);
                clear_led();
                gpio_put(led[1], true);
            }

            interrupt_flag = false;
            opt = -1;

            printf("\nListagem concluída.\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (opt == 'd') // Exibe o conteúdo do arquivo se pressionar 'd'
        {
            strcpy(op, "Lendo arquivo");

            clear_led();
            gpio_put(led[0], true);
            gpio_put(led[1], true);

            interrupt_flag = false;

            read_file(filename);

            if(err_flag)
            {
                print_error();
                clear_led();
                gpio_put(led[0], true);
            }else
            {
                print_op(true);
                clear_led();
                gpio_put(led[1], true);
            }

            interrupt_flag = false;
            opt = -1;

            printf("Escolha o comando (h = help):  ");
        }
        if (opt == 'e') // Obtém o espaço livre no SD card se pressionar 'e'
        {
            printf("\nObtendo espaço livre no SD.\n\n");
            strcpy(op, "Obtendo Espaco ");

            clear_led();
            gpio_put(led[0], true);
            gpio_put(led[1], true);

            print_op(false);

            interrupt_flag = true;

            run_getfree();

            if(err_flag)
            {
                print_error();
                clear_led();
                gpio_put(led[0], true);

            }else
            {

                ssd1306_fill(&ssd, !cor);
                ssd1306_draw_string(&ssd, "Espaco Obtido!", 0, 0);
                ssd1306_draw_string(&ssd, str_free_sct, 0, 16);
                ssd1306_draw_string(&ssd, "Kib", strlen(str_free_sct)*8+8, 16);
                ssd1306_draw_string(&ssd, "Livres", 0, 28);
                ssd1306_send_data(&ssd);

                clear_led();
                gpio_put(led[1], true);
            }

            interrupt_flag = false;
            opt = -1;

            printf("\nEspaço livre obtido.\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (opt == 'f') // Captura dados do MPU e salva no arquivo se pressionar 'f'
        {
            ssd1306_fill(&ssd, !cor);
            ssd1306_draw_string(&ssd, "Capturando Dado", 0, 0);
            ssd1306_draw_string(&ssd, "Nao Disconecte", 0, 24);
            ssd1306_draw_string(&ssd, "Aguarde...", 0, 34);
            ssd1306_send_data(&ssd);

            clear_led();
            gpio_put(led[0], true);
            gpio_put(led[1], true);

            interrupt_flag = true;

            capture_mpu_data_and_save();

            if(err_flag)
            {
                print_error();
                clear_led();
                gpio_put(led[0], true);

            }else
            {
                sprintf(str_samples, "%d", samples);
                ssd1306_fill(&ssd, !cor);
                ssd1306_draw_string(&ssd, "Dados Salvos!", 0, 0);
                ssd1306_draw_string(&ssd, "Bytes escritos:", 0, 20);
                ssd1306_draw_string(&ssd, bytewrite, 0, 29);
                ssd1306_draw_string(&ssd, str_samples, 0, 40);
                ssd1306_draw_string(&ssd, "amostras", strlen(str_samples)*8+8, 40);
                ssd1306_send_data(&ssd);

                clear_led();
                gpio_put(led[1], true);
            }

            interrupt_flag = false;
            opt = -1;

            printf("\nEscolha o comando (h = help):  ");
        }
        if (opt == 'g') // Formata o SD card se pressionar 'g'
        {
            printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");
            strcpy(op, "Formatando SD");

            clear_led();
            gpio_put(led[0], true);
            gpio_put(led[1], true);

            print_op(false);

            interrupt_flag = false;

            run_format();

            if(err_flag)
            {
                print_error();
                clear_led();
                gpio_put(led[0], true);
            }else
            {
                print_op(true);
                clear_led();
                gpio_put(led[1], true);
            }

            interrupt_flag = false;
            opt = -1;

            printf("\nFormatação concluída.\n\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (opt == 'h') // Exibe os comandos disponíveis se pressionar 'h'
        {
            ssd1306_fill(&ssd, !cor);
            ssd1306_draw_string(&ssd, "Aguardando", 4, 4);
            ssd1306_draw_string(&ssd, "Comando...", 4, 13);
            ssd1306_send_data(&ssd);

            run_help();

            interrupt_flag = false;
            opt = -1;

            clear_led();
            gpio_put(led[1], true);
        }
        sleep_ms(500);
    }
    return 0;
}