#ifndef SPI
#define SPI

#include <avr/io.h>
#include <stdint.h>

#define CE 1 // PB1. For nRF24
#define SS 2 // PB2. CSN for nRF24. Acitve-low pin
#define MOSI 3 // PB3
#define MISO 4 // PB4
#define SCK 5 // PB5

void spi_master_init(void)
{
	DDRB |= (1<<SCK) | (1<<MOSI) | (1<<SS) | (1 << CE);
	DDRB &= ~(1<<MISO); // this will be input
	SPCR |= (1<<SPE) | (1<<MSTR); // Enable SPI and set this ATmega as master
	set_bit(PORTB, SS); // no slaves are selected right now
	clear_bit(PORTB, CE); // nothing to send/receive yet
}

// send + receive
uint8_t spi_Send_Receive (uint8_t data)
{
	SPDR = data;

	while(!(SPSR & (1<<SPIF) ));

	return(SPDR);
}

/* I don't need this function for this project
void spi_send(unsigned char data)
{
	PORTB &= ~(1<<SS); // SS -> 0, enable slave
	SPDR = data; // load the buffer with our data
	while(!(SPSR &(1<<SPIF))); // wait for complete transfer of data
	PORTB |= (1<<SS); // disable the slave after transfer
}

void spi_slave_init()
{
	DDRB |= (1<<MISO);
	SPCR |= (1<<SPE);
}

unsigned char spi_receive(void)
{
	while (!(SPSR & (1<<SPIF)));
	return SPDR;
}
*/
#endif