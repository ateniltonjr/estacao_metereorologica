#ifndef DISPLAY_H
#define DISPLAY_H

#include "ssd1306.h"

#define I2C_PORT_display i2c1
#define I2C_SDA_display 14
#define I2C_SCL_display 15
#define endereco_display 0x3C

ssd1306_t ssd;
bool cor = true;

void init_display()
{
  // Inicializa I2C
  i2c_init(I2C_PORT_display, 400 * 1000);
  gpio_set_function(I2C_SDA_display, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_display, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_display);
  gpio_pull_up(I2C_SCL_display);

  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco_display, I2C_PORT_display);
  ssd1306_config(&ssd);
  ssd1306_send_data(&ssd);

  ssd1306_fill(&ssd, false); // Limpa o display
  ssd1306_send_data(&ssd);
}

void escrever(ssd1306_t *display, const char *texto, uint8_t x, uint8_t y, bool cor) 
{
    ssd1306_draw_string(display, texto, x, y);
    ssd1306_send_data(display);
}

void limpar_area(uint8_t x, uint8_t y, uint8_t largura, uint8_t altura) 
{
    for (uint8_t i = x; i < x + largura; i++) {
        for (uint8_t j = y; j < y + altura; j++) {
            ssd1306_pixel(&ssd, i, j, false);
        }
    }
}

#endif