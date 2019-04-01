#include "commiunication.h"
#include "USART.h"


void oscilloscope(int8_t data)
{
	Usart_SendByte( USART1,0X03);
	Usart_SendByte( USART1,~0X03);
	Usart_SendByte(USART1,data);
	Usart_SendByte( USART1,~0X03);
	Usart_SendByte( USART1,0X03);
}

