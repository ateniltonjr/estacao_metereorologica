# Estação Meteorológica

Este projeto implementa uma estação meteorológica embarcada utilizando Raspberry Pi Pico W, sensores BMP280 e AHT20, display OLED, LEDs, buzzer, matriz de LEDs e interface web para monitoramento e configuração.

## Funcionalidades
- Leitura de pressão, temperatura (BMP280 e AHT20), altitude e umidade.
- Exibição dos dados e unidade no display OLED e na interface web.
- Histórico gráfico das leituras na web.
- Configuração de limites e offsets dos sensores via web.
- Indicação visual (LED azul/vermelho) do estado dos parâmetros.
- Sinalização sonora e matriz de LEDs quando parâmetros estão fora dos limites.
- Alternância da leitura exibida via botão físico ou botão na web.
- Conexão Wi-Fi automática e servidor HTTP embutido.

## Hardware
- Raspberry Pi Pico W
- Sensor BMP280 (pressão/temperatura)
- Sensor AHT20 (umidade/temperatura)
- Display OLED
- LEDs (azul/vermelho)
- Buzzer
- Matriz de LEDs
- Botões físicos

## Como usar
1. Conecte todos os componentes conforme o esquema de hardware.
2. Configure o Wi-Fi (SSID e senha) no código (`WIFI_SSID`, `WIFI_PASS`).
3. Compile e grave o firmware usando o ambiente Pico SDK.
4. Acesse o IP exibido no display OLED para abrir a interface web.
5. Monitore os dados, ajuste limites/offsets e alterne páginas pelo site ou botão físico.

## Estrutura do Projeto
- `estacao_metereorologica.c`: Código principal do firmware.
- `lib/`: Drivers dos sensores e periféricos.
- `CMakeLists.txt`, `pico_sdk_import.cmake`: Build system.
- `build/`: Arquivos gerados na compilação.

## Interface Web
- Mostra o nome, valor e unidade da leitura atual.
- Permite configurar limites e offset de cada sensor.
- Exibe gráfico do histórico.
- Botão para alternar página de leitura.

## Autor
Atenilton Santos de Souza Júnior

---
