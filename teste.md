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
#include "buzzer.h"

#define led_red 13
#define led_green 11
#define led_blue 12

void inicializar_leds() 
{
    gpio_init(led_red);
    gpio_set_dir(led_red, GPIO_OUT);
    gpio_init(led_green);
    gpio_set_dir(led_green, GPIO_OUT);
    gpio_init(led_blue);
    gpio_set_dir(led_blue, GPIO_OUT);
}

// Controle de página para exibir uma leitura por vez
volatile uint8_t pagina_atual = 0;
const char* nomes_leituras[5] = {"Pressão", "Temperatura BMP280", "Altitude", "Temperatura AHT20", "Umidade"};

// Histórico e configuração de cada leitura
#define HIST_SIZE 30
float historico[5][HIST_SIZE] = {{0}};
uint8_t historico_idx[5] = {0};
float limites_min[5] = {0, 0, 0, 0, 0};
float limites_max[5] = {1000, 100, 10000, 30, 100};
float offsets[5] = {0, 0, 0, 0, 0};

#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                   // 1 ou 3
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

#define WIFI_SSID "KLAZ"
#define WIFI_PASS "10213250"

const char HTML_BODY[] =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Estação Meteorológica</title>"
    "<style>canvas{background:#fff;border:1px solid #ccc;}</style>"
    "<script>"
    "let historico = [];"
    "let paginaAtual = null;"
    "let camposPreenchidos = false;"
    "function atualizar() {"
    "  fetch('/estado').then(res => res.json()).then(data => {"
    "    document.getElementById('nome').innerText = data.nome;"
    "    document.getElementById('valor').innerText = data.valor.toFixed(2);"
    "    historico = data.historico;"
    "    desenharGrafico();"
    "    if (paginaAtual !== data.pagina || !camposPreenchidos) {"
    "      document.getElementById('lim_min').value = data.lim_min;"
    "      document.getElementById('lim_max').value = data.lim_max;"
    "      document.getElementById('offset').value = data.offset;"
    "      paginaAtual = data.pagina;"
    "      camposPreenchidos = true;"
    "    }"
    "  });"
    "}"
    "function desenharGrafico() {"
    "  let c = document.getElementById('grafico');"
    "  let ctx = c.getContext('2d');"
    "  ctx.clearRect(0,0,c.width,c.height);"
    "  if(historico.length<2) return;"
    "  let min = Math.min(...historico), max = Math.max(...historico);"
    "  if(min==max) {min-=1;max+=1;}"
    "  let w = c.width, h = c.height, n = historico.length;"
    "  ctx.beginPath();"
    "  for(let i=0;i<n;i++){"
    "    let x = i*(w/(n-1));"
    "    let y = h-(h*(historico[i]-min)/(max-min));"
    "    if(i==0) ctx.moveTo(x,y); else ctx.lineTo(x,y);"
    "  }"
    "  ctx.strokeStyle='#336699';ctx.lineWidth=2;ctx.stroke();"
    "}"
    "function enviarConfig() {"
    "  let lim_min = parseFloat(document.getElementById('lim_min').value);"
    "  let lim_max = parseFloat(document.getElementById('lim_max').value);"
    "  let offset = parseFloat(document.getElementById('offset').value);"
    "  let pagina = paginaAtual;"
    "  fetch('/config', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({pagina,lim_min,lim_max,offset})})"
    "    .then(()=>{camposPreenchidos=false; setTimeout(atualizar,300);});"
    "  return false;"
    "}"
    "setInterval(atualizar, 1000);"
    "window.onload=atualizar;"
    "</script></head><body>"
    "<h1>Estação Meteorológica</h1>"
    "<div style='margin-top:30px; text-align:center;'>"
    "  <h2 id='nome'>--</h2>"
    "  <div style='font-size:2.5em; margin:20px 0;' id='valor'>--</div>"
    "  <canvas id='grafico' width='320' height='100'></canvas>"
    "  <form onsubmit='return enviarConfig();' style='margin:20px auto;max-width:320px;'>"
    "    <label>Limite Mín: <input id='lim_min' type='number' step='any'></label><br>"
    "    <label>Limite Máx: <input id='lim_max' type='number' step='any'></label><br>"
    "    <label>Offset: <input id='offset' type='number' step='any'></label><br>"
    "    <button type='submit'>Salvar</button>"
    "  </form>"
    "  <p style='font-size:1em; color:#888;'>Pressione o botão A para alternar a leitura exibida</p>"
    "</div>"
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

    // Copia o payload da requisição para um buffer local
    char req[1024] = {0};
    pbuf_copy_partial(p, req, sizeof(req) - 1, 0);

    struct http_state hs;

    if (strstr(req, "POST /config")) {
        // Espera JSON: {"pagina":..., "lim_min":..., "lim_max":..., "offset":...}
        int pagina = -1;
        float lim_min = 0, lim_max = 0, offset = 0;
        char *json_start = strchr(req, '{');
        if (json_start) {
            // Tenta extrair os campos usando sscanf (robusto para JSON simples)
            int ok = sscanf(json_start,
                "{\"pagina\":%d,\"lim_min\":%f,\"lim_max\":%f,\"offset\":%f",
                &pagina, &lim_min, &lim_max, &offset);
            if (ok == 4 && pagina >= 0 && pagina < 5) {
                limites_min[pagina] = lim_min;
                limites_max[pagina] = lim_max;
                offsets[pagina] = offset;
                printf("Configuração alterada via HTTP para página %d: lim_min=%.2f, lim_max=%.2f, offset=%.2f\n", pagina, lim_min, lim_max, offset);
            } else {
                // fallback: tenta extrair cada campo separadamente
                char *pag_ptr = strstr(json_start, "\"pagina\"");
                char *min_ptr = strstr(json_start, "\"lim_min\"");
                char *max_ptr = strstr(json_start, "\"lim_max\"");
                char *off_ptr = strstr(json_start, "\"offset\"");
                if (pag_ptr) pagina = atoi(pag_ptr + 8);
                if (min_ptr) lim_min = atof(min_ptr + 9);
                if (max_ptr) lim_max = atof(max_ptr + 9);
                if (off_ptr) offset = atof(off_ptr + 8);
                if (pagina >= 0 && pagina < 5) {
                    limites_min[pagina] = lim_min;
                    limites_max[pagina] = lim_max;
                    offsets[pagina] = offset;
                    printf("Configuração alterada via HTTP para página %d: lim_min=%.2f, lim_max=%.2f, offset=%.2f\n", pagina, lim_min, lim_max, offset);
                }
            }
        }
        int len = snprintf(hs.response, sizeof(hs.response),
            "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok");
        tcp_write(tpcb, hs.response, len, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        pbuf_free(p);
        return ERR_OK;
    } else if (strstr(req, "GET /estado")) {
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

        // JSON para a leitura da página atual, incluindo histórico, limites e offset
        char historico_str[HIST_SIZE * 10] = "";
        int idx = historico_idx[pagina_atual];
        int n = 0;
        for (int i = 0; i < HIST_SIZE; i++) {
            int pos = (idx + i) % HIST_SIZE;
            n += snprintf(historico_str + n, sizeof(historico_str) - n, (i == 0 ? "[%.2f" : ",%.2f"), historico[pagina_atual][pos]);
        }
        snprintf(historico_str + n, sizeof(historico_str) - n, "]");

        const char* nome = nomes_leituras[pagina_atual];
        float valor_atual = historico[pagina_atual][(historico_idx[pagina_atual] + HIST_SIZE - 1) % HIST_SIZE];
        float lim_min = limites_min[pagina_atual];
        float lim_max = limites_max[pagina_atual];
        float offset = offsets[pagina_atual];

        char json_payload[1024];
        int json_len = snprintf(json_payload, sizeof(json_payload),
            "{\"pagina\":%d,\"nome\":\"%s\",\"valor\":%.2f,\"historico\":%s,\"lim_min\":%.2f,\"lim_max\":%.2f,\"offset\":%.2f}\r\n",
            pagina_atual, nome, valor_atual, historico_str, lim_min, lim_max, offset);
        int len = snprintf(hs.response, sizeof(hs.response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           json_len, json_payload);
        tcp_write(tpcb, hs.response, len, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        pbuf_free(p);
        return ERR_OK;
    } else {
        int len = snprintf(hs.response, sizeof(hs.response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/html\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(HTML_BODY), HTML_BODY);
        tcp_write(tpcb, hs.response, len, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        pbuf_free(p);
        return ERR_OK;
    }
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
#define BOTAO_A 5
#define BOTAO_B 6
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

int main()
{   
    inicializar_leds();
    iniciar_buzzer();
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();
    sleep_ms(2000);

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
        double altitude = calculate_altitude(pressure);

        // Leitura do AHT20 no i2c1 (pinos 2 e 3)
        int aht_ok = aht20_read(i2c1, &data);

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
        } else {
            gpio_put(led_red, 1);
            gpio_put(led_blue, 0);
            beep_curto();
            printf("Pelo menos um parâmetro está fora dos limites! LED vermelho ACESO.\n");
        }

        sleep_ms(300);
    }

    cyw43_arch_deinit();
    return 0;
}
