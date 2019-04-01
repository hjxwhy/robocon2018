#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include <stdio.h>


void USART_Config(void);
void USART2_Config(void); 
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);

#endif /* __USART_H */
