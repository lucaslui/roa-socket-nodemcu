/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : phy_stack.h
 * @author     : Lucas Lui Motta
 * @version	   : V0.0
 * @date       : 09/07/2021
 * @brief      : Header file of PHY Stack for RoA
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _PHY_H
#define _PHY_H

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>

#include "RoA.h"

/* Define --------------------------------------------------------------------*/
/* Typedef -------------------------------------------------------------------*/
typedef struct {
  /* Flags ******************************************************************/
  /* Variables **************************************************************/
  String ssid;             // Nome da Rede sem fio que vai se conectar
  String password;         // Senha da Rede sem fio que vai se conectar
  unsigned int localPort;  // Porta na qual o servidor ficará escutando
  String newHostname;      // Nome do Host na rede

  /* Function Pointers ******************************************************/
  void (*initialize)(st_led_blink_t *);  // Funcao de inicializacao da camada Física
  void (*receive)(uint8_t*);        // Funcao de recepcao de pacote da Camada Física
  void (*send)(uint8_t*);           // Funcao de envio de pacote da Camada Física
} st_phy_stack_t;

/* Public objects ------------------------------------------------------------*/
extern st_phy_stack_t PHY;  // Objeto da Camada MAC

#endif  // _PHY_H
/*****************************END OF FILE**************************************/
