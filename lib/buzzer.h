#ifndef BUZZER_H
#define BUZZER_H    

#include "pico/stdio.h"
#include <stdint.h>
#include "hardware/pwm.h"

#define buzzerA 21

void iniciar_buzzer()
{
    gpio_init(buzzerA);
    gpio_set_dir(buzzerA, GPIO_OUT);
}

void tocar_nota(int frequencia, int duracao) 
{
    if (frequencia == 0) {
        gpio_put(buzzerA, 0);
        return;
    }
    
    int periodo = 1000000 / frequencia;
    int metade_periodo = periodo / 2;
    
    for (int i = 0; i < duracao * 1000L / periodo; i++) {
        gpio_put(buzzerA, 1);
        sleep_us(metade_periodo);
        gpio_put(buzzerA, 0);
        sleep_us(metade_periodo);
    }
}

// Função para beep curto
void beep_curto() 
{
    tocar_nota(500, 300);
}

// Função para beep duplo
void beep_duplo() 
{
    tocar_nota(500, 80);
    sleep_ms(300);
    tocar_nota(500, 80);
}

#endif