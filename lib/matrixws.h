#ifndef MATRIXWS_H
#define MATRIXWS_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2818b.pio.h"

// Definição de constantes
#define PINO_MATRIZ 7 // Pino de controle da matriz de LEDs
#define NUM_LEDS 25  // Número total de LEDs na matriz
#define BRILHO 20   // Brilho dos LEDs 20/255

// Função para converter as posições (x, y) da matriz para um índice do vetor de LEDs
static inline int getIndex(int x, int y) {
    if (x % 2 == 0) { // Se a linha for par
        return 24 - (x * 5 + y); // Calcula o índice do LED considerando a ordem da matriz
    } else { // Se a linha for ímpar
        return 24 - (x * 5 + (4 - y)); // Calcula o índice invertendo a posição dos LEDs
    }
}

// Definição da estrutura de cor para cada LED
typedef struct {
    uint8_t R, G, B;       // Componentes de cor: vermelho, verde e azul
} npLED_t;

npLED_t leds[NUM_LEDS];  // Vetor que armazena as cores de todos os LEDs
PIO np_pio;  // PIO utilizado para comunicação com os LEDs
uint sm;     // State machine associada ao PIO

// Função para atualizar os LEDs da matriz
static inline void bf() 
{
    for (uint i = 0; i < NUM_LEDS; ++i) { // Percorre todos os LEDs
        pio_sm_put_blocking(np_pio, sm, leds[i].R); // Envia valor do componente vermelho
        pio_sm_put_blocking(np_pio, sm, leds[i].G); // Envia valor do componente verde
        pio_sm_put_blocking(np_pio, sm, leds[i].B); // Envia valor do componente azul
    }
    sleep_us(100); // Aguarda 100 microsegundos para estabilizar
}

// Função de controle inicial da matriz de LEDs
static inline void controle(uint pino) 
{
    uint offset = pio_add_program(pio0, &ws2818b_program); // Carrega programa para controlar LEDs no PIO
    np_pio = pio0; // Seleciona o PIO 0
    sm = pio_claim_unused_sm(np_pio, true); // Reivindica uma state machine livre
    ws2818b_program_init(np_pio, sm, offset, pino, 800000.f); // Inicializa o controle da matriz no pino especificado

    for (uint i = 0; i < NUM_LEDS; ++i) { // Inicializa todos os LEDs com a cor preta (desligados)
        leds[i].R = leds[i].G = leds[i].B = 0;
    }
    bf(); // Atualiza o estado dos LEDs
}

// Função para configurar a cor de um LED específico
static inline void cores_matriz(const uint indice, const uint8_t g, const uint8_t r, const uint8_t b) 
{
    leds[indice].R = r; // Define o componente vermelho
    leds[indice].G = g; // Define o componente verde
    leds[indice].B = b; // Define o componente azul
}

// Função para desligar todos os LEDs
static inline void desliga() 
{
    for (uint i = 0; i < NUM_LEDS; ++i) { // Percorre todos os LEDs
        cores_matriz(i, 0, 0, 0); // Define a cor preta (desligado) para cada LED
    }
    bf(); // Atualiza o estado dos LEDs
}

// Função para percorrer a matriz em linha
static inline void desenhaMatriz(int mat[5][5][3]) 
{
    for (int linha = 0; linha < 5; linha++) {
        for (int cols = 0; cols < 5; cols++) {
            int posicao = getIndex(linha, cols);
            cores_matriz(posicao, mat[linha][cols][0], mat[linha][cols][1], mat[linha][cols][2]);
        }
    }
    bf();  // Atualiza os LEDs
}

// Função para desenhar uma matriz com cor vermelha
void matriz_vermelha() {
    int mat[5][5][3] = {
        {{BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}},
        {{BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}},
        {{BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}},
        {{BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}},
        {{BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}, {BRILHO, 0, 0}}
    };
    desenhaMatriz(mat);
}

#endif 