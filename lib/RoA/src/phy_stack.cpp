/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : phy_stack.cpp
 * @author     : Lucas Lui Motta
 * @version	   : V0.0
 * @date       : 09/07/2021
 * @brief      : Source file of MAC Stack for RoA
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
// C standard library
// Application library
#include "phy_stack.h"

// Third-party library
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

/*******************************************************************************
                        HOW TO USE THIS MIDDLEWARE
********************************************************************************

1. 	First, you should include in your .c file the "ea_api_led.h" file.

2.  Call ea_api_led_init() to initialize the driver;

3.  Call ea_api_led_set() to set or ea_api_led_reset() to reset the LED.

*******************************************************************************/

/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

WiFiUDP Udp;

char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];  // Buffer de leitura do socket

st_led_blink_t *Pisca_LED;

uint8_t RSSI_ul;  // RSSI de UpLink (byte0 - Pacote)
uint8_t LQI_ul;   // LQI de UpLink (byte1 - Pacote)
uint8_t RSSI_dl;  // RSSI de DownLink (byte2 - Pacote)
uint8_t LQI_dl;   // LQI de DownLink (byte3 - Pacote)

float RSSI_dBm_dl;  // RSSI de Downlink em dBm
float RSSI_dBm_ul;  // RSSI de Uplink em dBm

/* Private function prototypes -----------------------------------------------*/

static void phy_initialize(st_led_blink_t *led_blink);
static void phy_receive(uint8_t *Pacote_RX);
static void phy_send(uint8_t *Pacote_TX);
static void phy_dBm_to_Radiuino();
static void phy_shadowing();
static void phy_box_muller(double sigma, double *r1, double *r2);

/* Public objects ------------------------------------------------------------*/

st_phy_stack_t PHY = {
    /* Peripheral disabled ****************************************************/
    /* variable init ***************************************************/
    .ssid = "GATITA",
    .password = "tiziano1993",
    .localPort = 8888,
    .newHostname = "SENSOR001",

    /* Function pointers loaded ***********************************************/
    .initialize = &phy_initialize,
    .receive = &phy_receive,
    .send = &phy_send,
};

/* Body of private functions -------------------------------------------------*/

/**
 * @Func       : phy_initialize
 * @brief      : Initialize physical layer
 * @pre-cond.  : Set variables such as ssid, password, localPorta and
 * newHostname
 * @post-cond. : Can invoke phy_receive() and phy_send()
 * @parameters : void
 * @retval     : void
 */
static uint8_t phy_initialize(st_led_blink_t *led_blink) {
  randomSeed(analogRead(0));  // semente do número aleatorio para RSSI
  WiFi.mode(WIFI_STA);
  WiFi.hostname(PHY.newHostname.c_str());
  WiFi.begin(PHY.ssid, PHY.password);
  // WiFi.waitForConnectResult();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP().toString());
  Serial.printf("UDP server on port %d\n", PHY.localPort);
  Udp.begin(PHY.localPort);
  Serial.printf("Default hostname: %s\n", WiFi.hostname().c_str());

  led_blink->rx = true;  // Configura o modulo para piscar o LED vermelho quando
                         // recebe pacote na serial
  led_blink->tx = true;  // Configura o modulo para piscar o LED verde quando
                         // envia pacote na serial

  return OK;
}

/**
 * @Func       : phy_receive()
 * @brief      : Receive physical layer packet
 * @pre-cond.  : phy_initialize() must be called first
 * @post-cond. : none
 * @parameters : void
 * @retval     : void
 */
static void phy_receive(uint8_t *Pacote_RX) {
  //===================== RECEPcaO DO PACOTE RX
  int packetSize = Udp.parsePacket();
  if (packetSize >= 52) {  // Testa se tem 52 bytes na serial
    //    Serial.printf("Received packet of size %d from %s:%d to %s:%d\n",
    //                  packetSize,
    //                  Udp.remoteIP().toString().c_str(), Udp.remotePort(),
    //                  Udp.destinationIP().toString().c_str(),
    //                  Udp.localPort());
    //
    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;

    if (Pisca_LED->rx) {
      digitalWrite(LED_vermelho, HIGH);
    }

    for (byte i = 0; i < 52;
         i++)  // Loop de 52 vezes para fazer a leitura do pacote byte a byte
    {
      Pacote_RX[i] =
          packetBuffer[i];  // Gravacao de cada byte no pacote de recepcao
    }

    int x = random(1, 101);  // Numero randomico
    int AB =
        ((Pacote_RX[47] * 10) + Pacote_RX[48]);  // dois penútimos digitos do RA
    int CD =
        ((Pacote_RX[49] * 10) + Pacote_RX[50]);  // dois ultimos digitos do RA

    if (Pacote_RX[51] == 1 &&
        x <= Pacote_RX[50]) {  // Entra neste for caso esteja selecionado para
                               // obter uma taxa de erro de pacotes e o teste de
                               // pacotes seja menor ou igual ao valor setado
      if (RX_ledblink) {
        digitalWrite(LED_vermelho, LOW);
      }
    } else if (Pacote_RX[51] == 2 &&
               x <= (((AB + CD) % 5) + 1) *
                        10) {  // Entra neste for caso esteja selecionado para
                               // obter uma taxa de erro de pacotes e o teste de
                               // pacotes seja menor ou igual ao valor setado
      if (RX_ledblink) {
        digitalWrite(LED_vermelho, LOW);
      }
    }

    else {
      if (RX_ledblink) {
        delay(50);
        digitalWrite(LED_vermelho, LOW);
      }
    }
  }
}

/**
 * @Func       : phy_initialize
 * @brief      : funcao de envio de pacote da Camada Física
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void phy_send(uint8_t *Pacote_TX) {
  double Shadowing_marsaglia;  // Variavel para uso de shadowing

  phy_box_muller(
      5, &Shadowing_marsaglia,
      NULL);  // Chama a função que gera números pseudo-aleatórios na
              // distribuição normal chamada box_muller(desvio-padrao,
              // Ponteiro_para_onde_vai_primeiro_PRGN,
              // Ponteiro_para_onde_vai_segundo_PRGN) agora, média zero e
              // desvio padrão 5, na variável Shadowing_marsaglia

  if (TX_ledblink) {
    digitalWrite(LED_verde, HIGH);
  }

  // Montagem pacote de transmissao - camada física

  if (Pacote_RX[51] == 3) {
    int BA =
        ((Pacote_RX[48] * 10) + Pacote_RX[47]);  // dois penútimos digitos do RA
    int DC =
        ((Pacote_RX[50] * 10) + Pacote_RX[49]);  // dois ultimos digitos do RA

    if (((BA + DC) % 5) == 0) {
      RSSI_dBm_dl = (-93 + Shadowing_marsaglia);
      // RSSI_dBm_dl =
      // ((random(-104,-81)+random(-104,-81)+random(-104,-81)+random(-104,-81)+random(-104,-81)+random(-104,-81)+random(-104,-81)+random(-104,-81)+random(-104,-81)+random(-104,-81))/10.0);
      // // Coloca um número aleatorio no byte0 para simular uma RSSI
      // RSSI_dBm_ul = random(-99,-86); // Coloca um número aleatorio no byte0
      // para simular uma RSSI
    } else if (((BA + DC) % 5) == 1) {
      RSSI_dBm_dl = (-86 + Shadowing_marsaglia);
      // RSSI_dBm_dl =
      // ((random(-96,-75)+random(-96,-75)+random(-96,-75)+random(-96,-75)+random(-96,-75)+random(-96,-75)+random(-96,-75)+random(-96,-75)+random(-96,-75)+random(-96,-75))/10.0);
      // // Coloca um número aleatorio no byte0 para simular uma RSSI
      // RSSI_dBm_ul = random(-91,-80); // Coloca um número aleatorio no byte0
      // para simular uma RSSI
    } else if (((BA + DC) % 5) == 2) {
      RSSI_dBm_dl = (-84 + Shadowing_marsaglia);
      // RSSI_dBm_dl =
      // ((random(-94,-73)+random(-94,-73)+random(-94,-73)+random(-94,-73)+random(-94,-73)+random(-94,-73)+random(-94,-73)+random(-94,-73)+random(-94,-73)+random(-94,-73))/10.0);
      // // Coloca um número aleatorio no byte0 para simular uma RSSI
      // RSSI_dBm_ul = random(-89,-78); // Coloca um número aleatorio no byte0
      // para simular uma RSSI
    } else if (((BA + DC) % 5) == 3) {
      RSSI_dBm_dl = (-79 + Shadowing_marsaglia);
      // RSSI_dBm_dl =
      // ((random(-89,-68)+random(-89,-68)+random(-89,-68)+random(-89,-68)+random(-89,-68)+random(-89,-68)+random(-89,-68)+random(-89,-68)+random(-89,-68)+random(-89,-68))/10.0);
      // // Coloca um número aleatorio no byte0 para simular uma RSSI
      // RSSI_dBm_ul = random(-84,-73); // Coloca um número aleatorio no byte0
      // para simular uma RSSI
    } else if (((BA + DC) % 5) == 4) {
      RSSI_dBm_dl = (-70 + Shadowing_marsaglia);
      // RSSI_dBm_dl =
      // ((random(-80,-59)+random(-80,-59)+random(-80,-59)+random(-80,-59)+random(-80,-59)+random(-80,-59)+random(-80,-59)+random(-80,-59)+random(-80,-59)+random(-80,-59))/10.0);
      // // Coloca um número aleatorio no byte0 para simular uma RSSI
      // RSSI_dBm_ul = random(-75,-64); // Coloca um número aleatorio no byte0
      // para simular uma RSSI
    }

  }

  else if (Pacote_RX[51] == 4) {
    phy_shadowing();
  }

  else {
    RSSI_dBm_dl =
        WiFi.RSSI();  // Coloca um número aleatorio no byte0 para simular uma
                      // RSSI com média -70 e desvio padrao 5
    // RSSI_dBm_ul = random(-74,-60); // Coloca um número aleatorio no byte0
    // para simular uma RSSI
  }

  phy_dBm_to_Radiuino();

  Pacote_TX[0] = RSSI_ul;  // Posicao 0 do pacote e preenchida com a RSSI de
                           // Uplink ( deveria ser preenchido na Base)
  Pacote_TX[1] = LQI_ul;   // Posicao 1 do pacote e preenchida com a LQI de
                           // Uplink
  Pacote_TX[2] =
      RSSI_dl;  // Posicao 2 do pacote e preenchida com a RSSI de Downlink
  Pacote_TX[3] =
      LQI_dl;  // Posicao 3 do pacote e preenchida com a LQI de Downlink

  // TRANSMISSaO DO PACOTE TX

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  for (byte i = 0; i < 52; i++) {
    Udp.write(Pacote_TX[i]);  // escrita do pacote Radiuino no UDP
  }
  Udp.endPacket();

  if (TX_ledblink) {
    delay(50);
    digitalWrite(LED_verde, LOW);
  }
}

/**
 * @Func       : phy_initialize
 * @brief      : Funcao que transforma RSSI em dBm da leitura do WiFi
                 para RSSI utilizada no radiuino (complemento de 2 com
                 passo 1/2 e 74 de offset)
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void phy_dBm_to_Radiuino() {
  /*tabela usada durante a criacao da funcao
   *   dBm     RSSI
   *  -10,5   127
   *  -74     0
   *  -138    128
   *  -74,5   255
   */

  if (RSSI_dBm_dl >
      -10.5)  // Caso a RSSI medida esteja acima do valor superior -10,5 dBm
  {
    RSSI_dl = 127;  // equivalente a -10,5 dBm
    LQI_dl = 1;     // alerta que alcancou saturacao no byte de LQI
  }

  if (RSSI_dBm_dl <= -10.5 &&
      RSSI_dBm_dl >=
          -74)  // Caso a RSSI medida esteja no intervalo [-10,5 dBm e -74 dBm]
  {
    RSSI_dl = ((RSSI_dBm_dl + 74) * 2);
    LQI_dl = 0;
  }

  if (RSSI_dBm_dl <
      -74)  // Caso a RSSI medida esteja no intervalo ]-74 dBm e -138 dBm]
  {
    RSSI_dl = (((RSSI_dBm_dl + 74) * 2) + 256);
    LQI_dl = 0;
  }

  //  if(RSSI_dBm_ul > -10.5)  // Caso a RSSI medida esteja acima do valor
  //  superior -10,5 dBm
  //  {
  //   RSSI_ul = 127; // equivalente a -10,5 dBm
  //   LQI_ul = 1;    // alerta que alcancou saturacao no byte de LQI
  //  }
  //
  //  if(RSSI_dBm_ul <= -10.5 && RSSI_dBm_ul >= -74) // Caso a RSSI medida
  //  esteja no intervalo [-10,5 dBm e -74 dBm]
  //  {
  //   RSSI_ul = ((RSSI_dBm_ul +74)*2) ;
  //   LQI_ul = 0;
  //  }
  //
  //  if(RSSI_dBm_ul < -74) // Caso a RSSI medida esteja no intervalo ]-74 dBm e
  //  -138 dBm]
  //  {
  //   RSSI_ul = (((RSSI_dBm_ul +74)*2)+256) ;
  //   LQI_ul = 0;
  //  }
}

/**
 * @Func       : phy_shadowing()
 * @brief      : Funcao que grava em RSSI_dBm_dl a RSSI de downlink calculada
                 de acordo com o modelo de shadowing ( Log Distance + V.A.) (
                 A VA deveria ser log normal mas o arduino gera numeros
                 aleatorios em distribuicao uniforme)
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void phy_shadowing() {
  float Frequencia;
  float Ptx, Despacolivre, Dtotal;
  float Gtx, Grx, Beta, Shadowing, Pd0;

  float Lel,
      Lambda;  // Variáveis usadas para ajudar nos calculos da potencia recebida

  double Shadowing_marsaglia;  // Variável para uso de shadowing

  if (Pacote_RX[50] == 1) {
    Frequencia = 433e6;  // 433 MHz
  } else if (Pacote_RX[50] == 2) {
    Frequencia = 915e6;  // 915MHz
  } else if (Pacote_RX[50] == 3) {
    Frequencia =
        2.437e9;  // Frequencia central (canal 6) do WiFi em 2,4 GHz (2,437 GHz)
  } else if (Pacote_RX[50] == 4) {
    Frequencia =
        5.5e9;  // Frequencia central da banda de 5Ghz do WiFi (5,5 GHz)
  }

  Ptx = Pacote_RX[49] / 10.0;  // Distancia de Espaco Livre

  Gtx = Pacote_RX[48] / 10.0;  // Ganho da antena de Transmissao

  Grx = Pacote_RX[47] / 10.0;  // Ganho da antena de Recepcao

  Despacolivre = Pacote_RX[46];  // Distancia D0 em metros

  Beta = Pacote_RX[45] / 10.0;  // Coeficiente de perda de percurso

  Dtotal =
      ((Pacote_RX[43] * 256) + Pacote_RX[44]);  // Distancia Total em metros

  Shadowing = Pacote_RX[42];  // limites da V.A. de Shadowing (lembrar que ela
                              // está multiplicada por 10)

  Lambda =
      3e8 / Frequencia;  // Cálculo do comprimento de onda V=lambda*Frequencia

  Lel =
      pow(((12.5664 * Despacolivre) / Lambda), 2);  // Atenuacao de espaco livre

  Pd0 = Ptx + Gtx + Grx - 10 * log10(Lel);  // Potencia recebida em D0

  phy_box_muller(
      Shadowing / 10, &Shadowing_marsaglia,
      NULL);  // Chama a função que gera números pseudo-aleatórios na
              // distribuição normal chamada box_muller(desvio-padrao,
              // Ponteiro_para_onde_vai_primeiro_PRGN,
              // Ponteiro_para_onde_vai_segundo_PRGN);

  RSSI_dBm_dl =
      Pd0 - 10 * Beta * (log10(Dtotal / Despacolivre)) + Shadowing_marsaglia;
}

/**
 * @Func       : phy_box_muller()
 * @brief      : generate a pair of normally distributed random numbers
                 using a Box-Muller transformation. The mean is 0 -- numbers
                 are equally likely to be + or -.  The required stdev is
                 given in 'sigma'.  Either result pointers can be NULL, if
                 you want just one number.
                https://forum.arduino.cc/t/pseudo-random-number-and-animal-behavior/112080/5
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void phy_box_muller(double sigma, double *r1, double *r2) {
  double u1, u2, v1, v2, s, z1, z2;
  for (;;) {
    // get two uniform random numbers from 0 to .999...
    u1 = (float)random(RAND_MAX) / ((float)RAND_MAX + 1);
    u2 = (float)random(RAND_MAX) / ((float)RAND_MAX + 1);

    v1 = 2.0L * u1 - 1.0L;
    v2 = 2.0L * u2 - 1.0L;
    s = v1 * v1 + v2 * v2;

    if (s <= 1.0L && s != 0.0L) break;
  }

  z1 = sqrt(-2.0L * log(s) / s) * v1;
  z2 = sqrt(-2.0L * log(s) / s) * v2;

  if (r1 != NULL) *r1 = (z1 * sigma);
  if (r2 != NULL) *r2 = (z2 * sigma);

  return;
}
