#ifndef LEDS_BUTTONS_H
#define LEDS_BUTTONS_H

#define BOTAO_A 5
#define BOTAO_B 6

void iniciar_botoes()
{
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);
    
}

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

#endif