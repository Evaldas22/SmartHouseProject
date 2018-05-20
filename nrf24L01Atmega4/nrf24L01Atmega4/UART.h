#ifndef UART
#define UART

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define BAUD 9600 // baud rate for our communication
#define BAUDRATE ((F_CPU/(BAUD * 16UL))-1) // UBRR value

void Uart_Init(void);
void Uart_Transmit (unsigned char data);
unsigned char Uart_Receive();

void Uart_Init()
{
	DDRD |= 1 << PD1; // TX
	DDRD &= ~(1 << PD0); // RX
	
	UBRRH = BAUDRATE >> 8;
	UBRRL = BAUDRATE;
	UCSRB |= (1 << RXEN) | (1 << TXEN); // enable RX and TX
	UCSRC |= (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0); // 8 bit, 1 stop, no parity
}

void Uart_Transmit(unsigned char data)
{
	while (!(UCSRA & (1 << UDRE))); // while uart data register is NOT empty
	// UDRE = 1 when it's ready
	UDR = data;
}

unsigned char Uart_Receive()
{
	while (!(UCSRA & (1 << RXC))); // RXC = 1, when there is data in it.
	return UDR;					// RXC = 0, when it's empty (or not full)
}

void Uart_Send_String(char *string)
{
	while(*string)
	{
		Uart_Transmit(*string++);
	}
}

#endif