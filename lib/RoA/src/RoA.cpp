/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : RoA.cpp
 * @author     : Lucas Lui Motta
 * @version	   : V0.0
 * @date       : 09/07/2021
 * @brief      : Source file of RoA (Radio Over Arduino)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
// C standard library
// Application library
#include "RoA.h"

#include "app_stack.h"
#include "mac_stack.h"
#include "net_stack.h"
#include "phy_stack.h"
#include "transp_stack.h"

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

uint8_t Pacote_RX[52];  // Pacote de recepção
uint8_t Pacote_TX[52];  // Pacote de transmissão


/* Private function prototypes -----------------------------------------------*/

static void roa_initialize();
static void roa_receive(uint8_t *Data);
static void roa_send(uint8_t *Data);

/* Public objects ------------------------------------------------------------*/

st_roa_t RoA = {
    /* Peripheral disabled ****************************************************/
    /* variable init ***************************************************/
    /* Function pointers loaded ***********************************************/
    .initialize = &roa_initialize,
    .receive = &roa_receive,
    .send = &roa_send,
};

/* Body of private functions -------------------------------------------------*/

/**
 * @Func       : phy_initialize
 * @brief      : This function handles EXTI line[15:10] interrupts.
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void roa_initialize()  // Funcao de inicializacao da camada Física
{
  PHY.initialize();  // Inicializa a camada Física
  MAC.initialize();     // Inicializa a camada de Controle de Acesso ao Meio
  NET.initialize();     // Inicializa a camada de Rede
  TRANSP.initialize();  // Inicializa a camada de Transporte
  APP.initialize();     // Inicializa a camada de Aplicação
}

/**
 * @Func       : roa_receive
 * @brief      : This function handles EXTI line[15:10] interrupts.
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void roa_receive(uint8_t *Data)
{
  PHY.receive(Pacote_RX);
  MAC.receive(Pacote_RX);
  NET.receive(Pacote_RX);
  TRANSP.receive(Pacote_RX);
  APP.receive(Data, Pacote_RX);
}

/**
 * @Func       : roa_send
 * @brief      : This function handles EXTI line[15:10] interrupts.
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void roa_send(uint8_t *Data)
{
  APP.send(Data, &Pacote_TX);
  TRANSP.send(&Pacote_TX);
  NET.send(&Pacote_TX);
  MAC.send(&Pacote_TX);
  PHY.send(&Pacote_TX);
}

/*****************************END OF FILE**************************************/