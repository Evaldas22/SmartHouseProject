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
	DDRD |= 1 << 1; // TX
	DDRD &= ~(1 << 0); // RX
	
	UBRR0H  = BAUDRATE >> 8;
	UBRR0L  = BAUDRATE;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); // enable RX and TX
	UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01)); // 8 bit, 1 stop, no parity
}

void Uart_Transmit(unsigned char data)
{
	while (!(UCSR0A & (1 << UDRE0))); // while uart data register is NOT empty
	// UDRE = 1 when it's ready
	UDR0 = data;
}

unsigned char Uart_Receive()
{
	while (!(UCSR0A & (1 << RXC0))); // RXC = 1, when there is data in it.
	return UDR0;					// RXC = 0, when it's empty (or not full)
}

void Uart_Send_String(char *string)
{
	while(*string)
	{
		Uart_Transmit(*string++);
	}
}

#endif