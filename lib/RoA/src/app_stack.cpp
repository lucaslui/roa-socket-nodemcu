/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : app_stack.cpp
 * @author     : Lucas Lui Motta
 * @version	   : V0.0
 * @date       : 09/07/2021
 * @brief      : Source file of MAC Stack for RoA
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
// C standard library
// Application library

#include "app_stack.h"

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

uint8_t byte16, byte17, byte18;  // ADC 0
uint8_t byte19, byte20, byte21;  // ADC 1
uint8_t byte22, byte23, byte24;  // ADC 2
uint8_t byte25, byte26, byte27;  // ADC 3
uint8_t byte28, byte29, byte30;  // ADC 4
uint8_t byte31, byte32, byte33;  // ADC 5
uint8_t byte34, byte35, byte36;  // DIGITAL 0
uint8_t byte37, byte38, byte39;  // DIGITAL 1
uint8_t byte40, byte41, byte42;  // DIGITAL 2
uint8_t byte43, byte44, byte45;  // DIGITAL 3
uint8_t byte46, byte47, byte48;  // DIGITAL 4
uint8_t byte49, byte50, byte51;  // DIGITAL 5

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void app_initialize();
static void app_receive(uint8_t *Data, uint8_t *Pacote_RX);
static void app_send(uint8_t *Data, uint8_t *Pacote_TX);

/* Public objects ------------------------------------------------------------*/

st_app_stack_t APP = {
    /* Peripheral disabled
     ****************************************************/
    /* variable init ***************************************************/
    /* Function pointers loaded
     ***********************************************/
    .initialize = &app_initialize,
    .receive = &app_receive,
    .send = &app_send,
};

/* Body of private functions -------------------------------------------------*/
/**
 * @Func       : app_initialize
 * @brief      : This function handles EXTI line[15:10] interrupts.
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void app_initialize() {}

/**
 * @Func       : app_receive
 * @brief      : This function handles EXTI line[15:10] interrupts.
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void app_receive(uint8_t *Data, uint8_t *Pacote_RX) {
  for (uint8_t i = 0; i < 7; i++) {
    Data[i] = Pacote_RX[i + 34];
  }
}

/**
 * @Func       : app_send
 * @brief      : função de envio da Camada de Aplicação
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void app_send(uint8_t *Data, uint8_t *Pacote_TX) {
  for (byte i = 0; i < 52; i++) {
    Pacote_TX[i] = 0;  // Zera cada um dos bytes do pacote de transmissao
  }
  for (uint8_t i = 0; i < 18; i++) {
    Pacote_TX[i + 34] = Data[i];
  }
}
