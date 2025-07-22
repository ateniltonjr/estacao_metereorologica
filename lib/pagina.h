#ifndef PAGINA_H
#define PAGINA_H 

#include "aht20.h"
#include "bmp280.h"
#include <math.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/tcp.h"

// Controle de página para exibir uma leitura por vez
volatile uint8_t pagina_atual = 0;
const char* nomes_leituras[5] = {"Pressão", "Temperatura BMP280", "Altitude", "Temperatura AHT20", "Umidade"};
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa

// Histórico e configuração de cada leitura
#define HIST_SIZE 30
float historico[5][HIST_SIZE] = {{0}};
uint8_t historico_idx[5] = {0};
float limites_min[5] = {0, 0, 0, 0, 0};
float limites_max[5] = {1000, 100, 10000, 30, 100};
float offsets[5] = {0, 0, 0, 0, 0};

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

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

    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs) {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }

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
        int len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok");
        tcp_write(tpcb, hs->response, len, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        pbuf_free(p);
        free(hs);
        return ERR_OK;
    } else if (strstr(req, "POST /pagina")) {
        // Alterna a página exibida
        pagina_atual = (pagina_atual + 1) % 5;
        printf("Página alterada via HTTP: %d\n", pagina_atual);
        int len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok");
        tcp_write(tpcb, hs->response, len, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        pbuf_free(p);
        free(hs);
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
        int aht_ok = aht20_read(i2c0, &data);

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
        const char* unidade = "";
        switch (pagina_atual) {
            case 0: unidade = "kPa"; break;
            case 1: unidade = "°C"; break;
            case 2: unidade = "m"; break;
            case 3: unidade = "°C"; break;
            case 4: unidade = "%"; break;
        }

        char json_payload[1024];
        int json_len = snprintf(json_payload, sizeof(json_payload),
            "{\"pagina\":%d,\"nome\":\"%s\",\"valor\":%.2f,\"unidade\":\"%s\",\"historico\":%s,\"lim_min\":%.2f,\"lim_max\":%.2f,\"offset\":%.2f}\r\n",
            pagina_atual, nome, valor_atual, unidade, historico_str, lim_min, lim_max, offset);
        int len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           json_len, json_payload);
        tcp_write(tpcb, hs->response, len, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        pbuf_free(p);
        free(hs);
        return ERR_OK;
    } else {
        int len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/html\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(HTML_BODY), HTML_BODY);
        tcp_write(tpcb, hs->response, len, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        pbuf_free(p);
        free(hs);
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

#endif