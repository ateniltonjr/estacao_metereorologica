#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "font.h"
#include "matrixws.h"
#include "leds_buttons.h"
#include "display.h"
#include "interface.h"
#include "pagina.h"

#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                   // 1 ou 3

#define WIFI_SSID "nome da rede wifi"
#define WIFI_PASS "senha da rede wifi"

#include "pico/bootrom.h"
#define debounce_delay 300
volatile uint tempo_interrupcao = 0;

void gpio_irq_handler(uint gpio, uint32_t events)
{
    uint tempo_atual = to_ms_since_boot(get_absolute_time());
    if (tempo_atual - tempo_interrupcao > debounce_delay)
    {
        if (gpio == BOTAO_A)
        {
            pagina_atual = (pagina_atual + 1) % 5;
            printf("Botão A pressionado! Página atual: %d\n", pagina_atual);
        }
        else if (gpio == BOTAO_B)
        {
            reset_usb_boot(0, 0); // Reinicia o dispositivo
        }
        tempo_interrupcao = tempo_atual;
    }
}

int iniciar_wifi()
{
    printf("Iniciando Wi-Fi...\n");
    escrever(&ssd, "Iniciando...", 10, 2, cor);
    ssd1306_fill(&ssd, false);

    if (cyw43_arch_init())
    {
        printf("Erro ao inicializar o Wi-Fi\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        printf("Erro ao conectar ao Wi-Fi\n");
        escrever(&ssd, "Erro ao conectar", 10, 2, cor);
        ssd1306_fill(&ssd, false);
        return 1;
    }

    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_str[24];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    printf("Conectado ao Wi-Fi...\n");
    printf("IP: %s\n", ip_str);
    escrever(&ssd, "IP:", 10, 5, cor);
    escrever(&ssd, ip_str, 5, 25, cor);
    ssd1306_fill(&ssd, false);
    sleep_ms(1000); // Aguarda 1 segundo para exibir o IP
}

void iniciar_I2Cs()
{
    // Inicializa o I2C0 (BMP280 nos pinos 0/1)
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    // Inicializa o I2C1 (AHT20 nos pinos 2/3)
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C); // SDA sensor
    gpio_set_function(1, GPIO_FUNC_I2C); // SCL sensor
    gpio_pull_up(0);
    gpio_pull_up(1);
}

char str_x[5]; // Buffer para armazenar a string
char str_y[5]; // Buffer para armazenar a string

char str_tmp1[5];  // Buffer para armazenar a string
char str_alt[5];  // Buffer para armazenar a string  
char str_tmp2[5];  // Buffer para armazenar a string
char str_umi[5];  // Buffer para armazenar a string

// Estrutura para armazenar os dados do sensor
    AHT20_Data data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;
    struct bmp280_calib_param params;

void dados_sensores()
{
    // Inicializa o BMP280 no i2c0 (pinos 0 e 1)
    bmp280_init(i2c0);
    bmp280_get_calib_params(i2c0, &params);

    // Inicializa o AHT20 no i2c1 (pinos 2 e 3)
    aht20_reset(i2c0);
    aht20_init(i2c0);
}

int main()
{   
    inicializar_leds();
    iniciar_buzzer();
    controle(PINO_MATRIZ); // Inicializa a matriz de LEDs
    iniciar_botoes(); // Inicializa os botões
    init_display(); // Inicializa o display OLED
    iniciar_I2Cs(); // Inicializa os barramentos I2C

    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();
    sleep_ms(2000);

    iniciar_wifi();

    start_http_server();
    dados_sensores(); 

    while (true)
    {
        cyw43_arch_poll();

        // Leitura do BMP280 no i2c0 (pinos 0 e 1)
        bmp280_read_raw(i2c0, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);
        double altitude = calculate_altitude(pressure);

        // Leitura do AHT20 no i2c1 (pinos 2 e 3)
        int aht_ok = aht20_read(i2c0, &data);

        // Armazenar valores no histórico, aplicando offset, mas só se leitura válida
        float valores[5];
        valores[0] = (pressure / 1000.0f) + offsets[0];
        valores[1] = (temperature / 100.0f) + offsets[1];
        valores[2] = altitude + offsets[2];
        valores[3] = aht_ok ? (data.temperature + offsets[3]) : historico[3][(historico_idx[3] + HIST_SIZE - 1) % HIST_SIZE];
        valores[4] = aht_ok ? (data.humidity + offsets[4]) : historico[4][(historico_idx[4] + HIST_SIZE - 1) % HIST_SIZE];

        for (int i = 0; i < 5; i++) {
            historico[i][historico_idx[i]] = valores[i];
            historico_idx[i] = (historico_idx[i] + 1) % HIST_SIZE;
        }

        // Exibe no display o nome, valor e unidade da página atual
        ssd1306_fill(&ssd, false); // Limpa display
        const char* unidade = "";
        switch (pagina_atual) {
            case 0: unidade = "kPa"; break; // Pressão
            case 1: unidade = "°C"; break; // Temperatura BMP280
            case 2: unidade = "m"; break; // Altitude
            case 3: unidade = "°C"; break; // Temperatura AHT20
            case 4: unidade = "%"; break; // Umidade
        }
        char valor_str[32];
        snprintf(valor_str, sizeof(valor_str), "%.2f %s", valores[pagina_atual], unidade);
        escrever(&ssd, nomes_leituras[pagina_atual], 5, 10, cor); // Nome na linha de cima
        escrever(&ssd, valor_str, 5, 30, cor); // Valor + unidade na linha de baixo
        ssd1306_send_data(&ssd);

        // LED azul aceso se TODOS os parâmetros estão dentro dos limites
        // LED vermelho aceso se PELO MENOS UM estiver fora
        int todos_dentro = 1;
        for (int i = 0; i < 5; i++) {
            if (valores[i] < limites_min[i] || valores[i] > limites_max[i]) {
                todos_dentro = 0;
                break;
            }
        }
        if (todos_dentro) {
            gpio_put(led_red, 0);
            gpio_put(led_blue, 1);
            desliga();
        } else {
            gpio_put(led_red, 1);
            gpio_put(led_blue, 0);
            matriz_vermelha();
            tocar_nota(500, 300); // Toca um beep curto
            printf("Pelo menos um parâmetro está fora dos limites! LED vermelho ACESO.\n");
        }

        sleep_ms(300);
    }

    cyw43_arch_deinit();
    return 0;
}
