#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/tcp.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ssd1306.h"
#include "font.h"
#include "aht20.h"
#include "bmp280.h"
#include <math.h>

#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                   // 1 ou 3
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

#define LED_PIN 12 // Removido uso do LED
#define BOTAO_A 5
#define BOTAO_JOY 22
#define JOYSTICK_X 26
#define JOYSTICK_Y 27

#define WIFI_SSID "KLAZ"
#define WIFI_PASS "10213250"

const char HTML_BODY[] =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Estação Meteorológica</title>"
    "<script>"
    "function atualizar() {"
    "  fetch('/estado').then(res => res.json()).then(data => {"
    "    document.getElementById('pressao').innerText = data.pressao !== null ? (data.pressao.toFixed(2) + ' kPa') : '--';"
    "    document.getElementById('temp_bmp').innerText = data.temp_bmp !== null ? (data.temp_bmp.toFixed(2) + ' °C') : '--';"
    "    document.getElementById('altitude').innerText = data.altitude !== null ? (data.altitude.toFixed(2) + ' m') : '--';"
    "    document.getElementById('temp_aht').innerText = data.temp_aht !== null ? (data.temp_aht.toFixed(2) + ' °C') : '--';"
    "    document.getElementById('umidade').innerText = data.umidade !== null ? (data.umidade.toFixed(2) + ' %') : '--';"
    "  });"
    "}"
    "setInterval(atualizar, 1000);"
    "</script></head><body>"

    "<h1>Estação Meteorológica</h1>"

    "<p>Pressão: <span id='pressao'>--</span></p>"
    "<p>Temperatura BMP280: <span id='temp_bmp'>--</span></p>"
    "<p>Altitude estimada: <span id='altitude'>--</span></p>"
    "<p>Temperatura AHT20: <span id='temp_aht'>--</span></p>"
    "<p>Umidade: <span id='umidade'>--</span></p>"

    "<hr style='margin-top: 20px;'>"
    "<p style='font-size: 15px; color: #336699; font-style: italic; max-width: 90%; margin: 10px auto;'>"
    "</p>"

    "</body></html>";

struct http_state
{
    char response[4096];
    size_t len;
    size_t sent;
};

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len)
    {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p)
    {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs)
    {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;

    if (strstr(req, "GET /estado"))
    {
        // Leitura do BMP280
        int32_t raw_temp_bmp, raw_pressure;
        struct bmp280_calib_param params;
        bmp280_get_calib_params(i2c0, &params);
        bmp280_read_raw(i2c0, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);
        double altitude = calculate_altitude(pressure);

        // Leitura do AHT20
        AHT20_Data data;
        int aht_ok = aht20_read(i2c1, &data);

        // JSON com as 5 leituras
        char json_payload[192];
        if (aht_ok) {
            int json_len = snprintf(json_payload, sizeof(json_payload),
                "{\"pressao\":%.2f,\"temp_bmp\":%.2f,\"altitude\":%.2f,\"temp_aht\":%.2f,\"umidade\":%.2f}\r\n",
                pressure / 1000.0,
                temperature / 100.0,
                altitude,
                data.temperature,
                data.humidity);
            hs->len = snprintf(hs->response, sizeof(hs->response),
                               "HTTP/1.1 200 OK\r\n"
                               "Content-Type: application/json\r\n"
                               "Content-Length: %d\r\n"
                               "Connection: close\r\n"
                               "\r\n"
                               "%s",
                               json_len, json_payload);
        } else {
            int json_len = snprintf(json_payload, sizeof(json_payload),
                "{\"pressao\":%.2f,\"temp_bmp\":%.2f,\"altitude\":%.2f,\"temp_aht\":null,\"umidade\":null}\r\n",
                pressure / 1000.0,
                temperature / 100.0,
                altitude);
            hs->len = snprintf(hs->response, sizeof(hs->response),
                               "HTTP/1.1 200 OK\r\n"
                               "Content-Type: application/json\r\n"
                               "Content-Length: %d\r\n"
                               "Connection: close\r\n"
                               "\r\n"
                               "%s",
                               json_len, json_payload);
        }
        printf("[DEBUG] JSON: %s\n", json_payload);
    }
    else
    {
        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/html\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(HTML_BODY), HTML_BODY);
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);

    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    pbuf_free(p);
    return ERR_OK;
}

static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

static void start_http_server(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)
    {
        printf("Erro ao criar PCB TCP\n");
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}

#include "pico/bootrom.h"
#define BOTAO_B 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();
    sleep_ms(2000);

    adc_init();
    adc_gpio_init(JOYSTICK_X);
    adc_gpio_init(JOYSTICK_Y);

    printf("Iniciando Wi-Fi...\n");

    if (cyw43_arch_init())
    {
        printf("Erro ao inicializar o Wi-Fi\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        printf("Erro ao conectar ao Wi-Fi\n");
        return 1;
    }

    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_str[24];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    printf("Conectado ao Wi-Fi...\n");
    printf("IP: %s\n", ip_str);

    start_http_server();
    char str_x[5]; // Buffer para armazenar a string
    char str_y[5]; // Buffer para armazenar a string
    bool cor = true;

    /**/
    // Inicializa o I2C0 (BMP280 nos pinos 0/1)
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    // Inicializa o I2C1 (AHT20 nos pinos 2/3)
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(2, GPIO_FUNC_I2C); // SDA sensor
    gpio_set_function(3, GPIO_FUNC_I2C); // SCL sensor
    gpio_pull_up(2);
    gpio_pull_up(3);

    // Inicializa o BMP280 no i2c0 (pinos 0 e 1)
    bmp280_init(i2c0);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(i2c0, &params);

    // Inicializa o AHT20 no i2c1 (pinos 2 e 3)
    aht20_reset(i2c1);
    aht20_init(i2c1);

    // Estrutura para armazenar os dados do sensor
    AHT20_Data data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;

    char str_tmp1[5];  // Buffer para armazenar a string
    char str_alt[5];  // Buffer para armazenar a string  
    char str_tmp2[5];  // Buffer para armazenar a string
    char str_umi[5];  // Buffer para armazenar a string 

    while (true)
    {
        cyw43_arch_poll();

        // Leitura do BMP280 no i2c0 (pinos 0 e 1)
        bmp280_read_raw(i2c0, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Cálculo da altitude
        double altitude = calculate_altitude(pressure);

        //printf("Pressao = %.3f kPa\n", pressure / 1000.0);
        //printf("Temperatura BMP: = %.2f C\n", temperature / 100.0);
        //printf("Altitude estimada: %.2f m\n", altitude);

        // Leitura do AHT20 no i2c1 (pinos 2 e 3)
        if (aht20_read(i2c1, &data))
        {
            //printf("Temperatura AHT: %.2f C\n", data.temperature);
            //printf("Umidade: %.2f %%\n\n\n", data.humidity);
        }
        else
        {
            //printf("Erro na leitura do AHT10!\n\n\n");
        }

        sleep_ms(300);
    }

    cyw43_arch_deinit();
    return 0;
}
