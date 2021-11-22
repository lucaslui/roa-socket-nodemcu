/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : transp_stack.h
 * @author     : Lucas Lui Motta
 * @version	   : V0.0
 * @date       : 09/07/2021
 * @brief      : Header file of TRANSP Stack for RoA
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _TRANSP_H  
#define _TRANSP_H 

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>

/* Define --------------------------------------------------------------------*/
/* Typedef -------------------------------------------------------------------*/
typedef struct {
  /* Flags ******************************************************************/
  /* Variables **************************************************************/
  int pkt_counter_up;   // Contador de pacotes transmitido pelo sensor
  int pkt_counter_down; // Contador de pacotes recebidos pelo sensor

  /* Function Pointers ******************************************************/
  void (*initialize)();  // Função de inicialização da camada de Transporte
  void (*receive)(uint8 *);     // Função de recepção de pacote da Camada de Transporte
  void (*send)(uint8 *);        // Função de envio de pacote da Camada de Transporte
} st_transp_stack_t;

/* Public objects ------------------------------------------------------------*/
extern st_transp_stack_t TRANSP;  // Objeto da Camada MAC

#endif  // _TRANSP_H
/*****************************END OF FILE**************************************/
