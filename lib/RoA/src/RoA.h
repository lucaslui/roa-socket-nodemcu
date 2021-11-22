/**
 ******************************************************************************
 * @Company    : IoT M3F
 * @file       : RoA.h
 * @author     : Lucas Lui Motta
 * @version	   : V0.0
 * @date       : 09/07/2021
 * @brief      : Header file of RoA (Radio Over Arduino)
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef _ROA_H
#define _ROA_H

/* Includes ------------------------------------------------------------------*/

#include <Arduino.h>

/* Define --------------------------------------------------------------------*/
/* Typedef -------------------------------------------------------------------*/

typedef struct {
  bool rx;  // Variável booleana para determinar se o led vermelho
            // deve piscar na recepção
  bool tx;  // Variável booleana para determinar se o led verde deve
            // piscar na transmissão

} st_led_blink_t;

typedef struct {
  /* Flags ******************************************************************/
  /* Variables **************************************************************/
  String ssid;             // Nome da Rede sem fio que vai se conectar
  String password;         // Senha da Rede sem fio que vai se conectar
  unsigned int localPort;  // Porta na qual o servidor ficará escutando
  String newHostname;      // Nome do Host na rede

  /* Function Pointers ******************************************************/
  void (*initialize)();       // Funcao de inicializacao 
  void (*receive)(uint8 *);  // Funcao de recepcao
  void (*send)(uint8 *);  // Funcao de transmissao 

} st_roa_t;

/* Public objects ------------------------------------------------------------*/

extern st_roa_t RoA;  // Objeto RoA

#endif  // _ROA_H
/*****************************END OF FILE**************************************/
