/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : app_stack.h
 * @author     : Lucas Lui Motta
 * @version	   : V0.0
 * @date       : 09/07/2021
 * @brief      : Header file of APP Stack for RoA
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _APP_H
#define _APP_H

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>

/* Define --------------------------------------------------------------------*/
/* Typedef -------------------------------------------------------------------*/
typedef struct {
  /* Flags ******************************************************************/
  /* Variables **************************************************************/
  int pkt_counter_up;
  int pkt_counter_down;

  /* Function Pointers ******************************************************/
  void (*initialize)();  // Função de inicialização da camada de Aplicação
  void (*receive)(uint8_t*, uint8 *);  // Função de recepção de pacote da Camada de Transporte
  void (*send)(uint8_t*, uint8 *);  // Função de envio da Camada de Aplicação
} st_app_stack_t;

/* Public objects ------------------------------------------------------------*/
extern st_app_stack_t APP;  // Objeto da Camada MAC

#endif  // _APP_H
/*****************************END OF FILE**************************************/