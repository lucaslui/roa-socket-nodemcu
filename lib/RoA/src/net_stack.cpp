/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : net_stack.cpp
 * @author     : Lucas Lui Motta
 * @version    : V0.0
 * @date       : 09/07/2021
 * @brief      : Source file of NET Stack for RoA
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
// C language standard library
// IoT M3F middleware library
#include "net_stack.h"

#include "mac_stack.h"

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

uint8_t byte8, byte9, byte10, byte11;

/* Private function prototypes -----------------------------------------------*/
static void net_initialize();
static void net_receive(uint8_t *Pacote_RX);
static void net_send(uint8_t *Pacote_TX);

/* Public objects ------------------------------------------------------------*/
st_net_stack_t NET = {
    /* Peripheral disabled ****************************************************/
    /* variable init ***************************************************/
    .my_address = 0,

    /* Function pointers loaded ***********************************************/
    .initialize = &net_initialize,
    .receive = &net_receive,
    .send = &net_send,
};

/* Body of private functions -------------------------------------------------*/
/**
 * @Func       : net_initialize
 * @brief      : This function handles EXTI line[15:10] interrupts.
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void net_initialize()  // Fun????o de inicializa????o da camada de Rede
{
  NET.my_address = 1;  // Define o endere??o do sensor
}

/**
 * @Func       : net_receive
 * @brief      : fun????o de recep????o de pacote da Camada de Rede
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void net_receive(uint8_t *Pacote_RX) {
  // J?? fez a compara????o com o meu endere??o antes de come??ar a desencapsular o
  // pacote. o lugar natural seria aqui.

  byte8 = Pacote_RX[8];  // Endere??o de Destino
  byte9 = Pacote_RX[9];  // Endere??o de Origem
  byte10 = Pacote_RX[10];
  byte11 = Pacote_RX[11];

  if (byte8 == NET.my_address)  // Faz uma verifica????o do endere??o de destido
                                // antes de come??ar a desencapsular o pacote
                                // (existe crosslayer neste ponto)
  {
    MAC.contador_mac =
        MAC.contador_mac +
        10;  // Contador ?? incrementado em 10 a cada vez que um pacote passa
             // pela fun????o de recep????o da Camada MAC
  }
}

/**
 * @Func       : net_send
 * @brief      : fun????o de envio de pacote da Camada de Rede
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void net_send(uint8_t *Pacote_TX) {
  Pacote_TX[8] =
      byte10;  // Inverte o endere??o de origem e destino, colocando o endere??o
               // de origem do pacote recebido como destinat??rio
  Pacote_TX[9] = byte9;
  Pacote_TX[10] =
      NET.my_address;  // coloca o meu endere??o como origem do pacote de TX
  Pacote_TX[11] = byte11;
}
