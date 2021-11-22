/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : mac_stack.h
 * @author     : Lucas Lui Motta
 * @version	   : V0.0
 * @date       : 09/07/2021
 * @brief      : Header file of MAC Stack for RoA
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MAC_H
#define _MAC_H

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>

/* Define --------------------------------------------------------------------*/
/* Typedef -------------------------------------------------------------------*/
typedef struct {
  /* Flags ******************************************************************/
  /* Variables **************************************************************/
  uint8 contador_mac;  // mac counter

  /* Function Pointers ******************************************************/
  void (*initialize)();  // Função de inicialização da camada de Acesso ao Meio
  void (*receive)(uint8_t*);     // Função de recepção de pacote da Camada MAC
  void (*send)(uint8_t*);        // Função de envio de pacote da Camada MAC
} st_mac_stack_t;

/* Public objects ------------------------------------------------------------*/
extern st_mac_stack_t MAC;  // Objeto da Camada MAC

#endif /* _MAC_H */
/*****************************END OF FILE**************************************/
