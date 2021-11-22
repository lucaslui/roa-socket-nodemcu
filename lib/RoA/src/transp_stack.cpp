/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : transp_stack.cpp
 * @author     : Lucas Lui Motta
 * @version	   : V0.0
 * @date       : 09/07/2021
 * @brief      : Source file of MAC Stack for RoA
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
// C language standard library
// IoT M3F middleware library
#include "transp_stack.h"

#include "net_stack.h"
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
/* Private variables ---------------------------------------------------------*/

struct st_transp_header_t {

} Header;

uint8_t MSB_pkt_counter_up;
uint8_t LSB_pkt_counter_up;

uint8_t MSB_pkt_counter_down;
uint8_t LSB_pkt_counter_down;

uint8_t byte12, byte13, byte14, byte15;  // Bytes da camada de transporte

/* Private function prototypes -----------------------------------------------*/
static void transp_initialize();  
static void transp_receive(uint8 *Pacote_RX);     
static void transp_send(uint8 *Pacote_TX);        

/* Public objects ------------------------------------------------------------*/
st_transp_stack_t TRANSP = {
    /* Peripheral disabled ****************************************************/
    /* variable init ***************************************************/
    .pkt_counter_up = 0,
    .pkt_counter_down = 0,

    /* Function pointers loaded ***********************************************/
    .initialize = &transp_initialize,
    .receive = &transp_receive,
    .send = &transp_send,
};

/* Body of private functions -------------------------------------------------*/
/**
 * @Func       : mac_initialize
 * @brief      : This function handles EXTI line[15:10] interrupts.
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void transp_initialize()  // Função de inicialização da camada de Transporte
{
  TRANSP.pkt_counter_up = 0; //Inicializa o contador de pacotes 
}

/**
 * @Func       : mac_initialize
 * @brief      : This function handles EXTI line[15:10] interrupts.
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void transp_receive(uint8 *Pacote_RX)   // Função de recepção de pacote da Camada de Transporte
{
  MSB_pkt_counter_down = Pacote_RX[12];
  LSB_pkt_counter_down = Pacote_RX[13];
  MSB_pkt_counter_up = Pacote_RX[14];
  LSB_pkt_counter_up = Pacote_RX[15];
}

/**
 * @Func       : transp_send
 * @brief      : Função de envio de pacote da Camada de Transporte
 * @pre-cond.  : ea_gpio_open() must be called first
 * @post-cond. : Attends interrupts and invoke callbacks if it's enabled
 * @parameters : void
 * @retval     : void
 */
static void transp_send(uint8 *Pacote_TX)
{ 
  TRANSP.pkt_counter_up = TRANSP.pkt_counter_up + 1;
  /* MSB do contador de pacotes (divisão inteira por 256) */
  MSB_pkt_counter_up = TRANSP.pkt_counter_up / 256;
  /* LSB do contador de pacotes  (resto da divisão por 256) */
  LSB_pkt_counter_up = TRANSP.pkt_counter_up % 256;
  Pacote_TX[12] = MSB_pkt_counter_down;  
  Pacote_TX[13] = LSB_pkt_counter_down;  
  Pacote_TX[14] = MSB_pkt_counter_up;
  Pacote_TX[15] = LSB_pkt_counter_up;
  }
