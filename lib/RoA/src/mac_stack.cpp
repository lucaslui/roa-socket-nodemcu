/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : mac_stack.cpp
 * @author     : Lucas Lui Motta
 * @version	   : V0.0
 * @date       : 09/07/2021
 * @brief      : Source file of MAC Stack for RoA
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
// C language standard library
// IoT M3F middleware library
#include "mac_stack.h"

#include "net_stack.h"
#include "phy_stack.h"

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

uint8_t byte4, byte5, byte6, byte7;

/* Private function prototypes -----------------------------------------------*/

static void mac_initialize();  // Initialize LED API
static void mac_receive(uint8_t*);     // Deinitialize LED API
static void mac_send(uint8_t*);        // Set LED

/* Public objects ------------------------------------------------------------*/

st_mac_stack_t MAC = {
    /* Peripheral disabled ****************************************************/
    /* variable init ***************************************************/
    .contador_mac = 0,

    /* Function pointers loaded ***********************************************/
    .initialize = &mac_initialize,
    .receive = &mac_receive,
    .send = &mac_send,
};

/* Body of private functions -------------------------------------------------*/

/**
 * @Func       : mac_initialize
 * @brief      : função de inicialização da camada de Acesso ao Meio
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void mac_initialize() {
  MAC.contador_mac = 0;  // Inicializa o contador da MAC
}

/**
 * @Func       : mac_receive
 * @brief      : função de recepção de pacote da Camada MAC
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void mac_receive(uint8_t *Pacote_RX) {
  byte4 = Pacote_RX[4];  // Retira a informação da RSSI de Downlink do pacote
                         // recebido
  byte5 = Pacote_RX[5];
  byte6 = Pacote_RX[6];
  byte7 = Pacote_RX[7];
}

/**
 * @Func       : mac_send
 * @brief      : função de envio de pacote da Camada MAC
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void mac_send(uint8_t *Pacote_TX) {
  Pacote_TX[4] =
      MAC.contador_mac /
      256;  // Byte 0 do cabeçalho de Transporte recebe o byte mais
            // significativo do contador de pacotes (divisão inteira por 256)
  Pacote_TX[5] =
      MAC.contador_mac %
      256;  // Byte 1 do cabeçalho de Transporte recebe o byte menos
            // significativo do contador de pacotes  (Resto da divisão por 256)
  Pacote_TX[6] = byte6;
  Pacote_TX[7] = byte7;
}
