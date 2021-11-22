// Monitoração de RSSI no NodeMCU
#include <Arduino.h>

#include "RoA.h"

/* Define --------------------------------------------------------------------*/
#define LED_verde D1     // Define que o Led verde está no Pino D1 da PK2
#define LED_amarelo D2   // Define que o Led amarelo está no Pino D2 da PK2
#define LED_vermelho D3  // Define que o Led vermelho está no Pino d3 da PK2
#define Buzzer D5        // Define buzzer no Pino D5 da PK1

bool TX_ledblink;  // Variável booleana para determinar se o led vermelho deve
                   // piscar na recepção
bool RX_ledblink;  // Variável booleana para determinar se o led verde deve
                   // piscar na transmissão

struct st_digital_data_t {
  uint8_t green_led;
  uint8_t noknow1;
  uint8_t noknow2;
  uint8_t yellow_led;
  uint8_t noknow3;
  uint8_t noknow4;
  uint8_t red_led;
} DigitalData;

void setup() {
  pinMode(LED_verde, OUTPUT);     // Define o pino do LED Verde como saída
  pinMode(LED_amarelo, OUTPUT);   // Define o pino do LED Amarelo como saída
  pinMode(LED_vermelho, OUTPUT);  // Define o pino do LED Vermelho como saída
  pinMode(Buzzer, OUTPUT);        // Define o pino do Buzzer como saída
  Serial.begin(
      115200);       // Inicializa a porta serial com uma taxa de 115200 kbps
  RoA.initialize();  // Inicializa o RoA (Radio Over Arduino)
}

void loop() {
  uint8_t Data[7];

  while (1) {
    RoA.receive(Data);  // Recebe os dados

    DigitalData.green_led = Data[0];
    DigitalData.yellow_led = Data[3];
    DigitalData.red_led = Data[6];

    if (DigitalData.green_led) {
      TX_ledblink = false;  // Configura o módulo para não piscar o LED verde
                            // quando envia pacote na serial
    } else {
      TX_ledblink = true;  // Configura o módulo para piscar o LED verde quando
                           // envia pacote na serial
    }

    /* Liga ou desliga o LED verde de acordo com o status */
    digitalWrite(LED_verde, DigitalData.green_led);

    /* Liga ou desliga o LED Amarelo de acordo com o status */
    digitalWrite(LED_amarelo, DigitalData.yellow_led);

    if (DigitalData.red_led) {
      RX_ledblink = false;  // Configura o módulo para não piscar o LED vermelho
                            // quando recebe pacote na serial
    } else {
      RX_ledblink = true;  // Configura o módulo para piscar o LED vermelho
                           // quando recebe pacote na serial
    }
    /* Liga ou desliga o LED vermelho de acordo com o status */
    digitalWrite(LED_vermelho, DigitalData.red_led);

    if (Data) {
      RoA.send(Data);
    }
  }
}