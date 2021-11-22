/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file	     : net_stack.h
 * @author     : Lucas Lui Motta
 * @version    : V0.0
 * @date       : 09/07/2021
 * @brief      : Header file of Network Stack for RoA
 *****************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _NET_H
#define _NET_H

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>

/* Define --------------------------------------------------------------------*/
/* Typedef -------------------------------------------------------------------*/
typedef struct {
  /* Flags ******************************************************************/
  /* Variables **************************************************************/
  uint8 my_address;

  /* Function Pointers ******************************************************/
  void (*initialize)(void);  // Função de inicialização da camada de Rede
  void (*receive)(uint8 *);  // Função de recepção de pacote da Camada de Rede
  void (*send)(uint8 *);     // Função de envio de pacote da Camada de Rede

} st_net_stack_t;

/* Public objects ------------------------------------------------------------*/
extern st_net_stack_t NET; // Objeto da Camada NET

#endif  // _NET_H
/*****************************END OF FILE**************************************/