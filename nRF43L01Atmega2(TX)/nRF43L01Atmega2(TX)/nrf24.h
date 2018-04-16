#ifndef nrf24
#define nrf24

#define W 1
#define R 0

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "SPI.h"
#include "nRF24L01.h"
#include "Basic_operations.h"

uint8_t ReadRegister(uint8_t register);
uint8_t *ReadWriteNRF(uint8_t R_W, uint8_t reg, uint8_t *data, uint8_t size);
void Nrf24_init(uint8_t pipe, uint8_t *addrRX, uint8_t *addrTX,  char mode);
void transmit_data(uint8_t *data);
void receive_data(void);
void resetNrf(void);
void changeNrfToRX();
void changeNrfToTX();

// reg is memory address
uint8_t ReadRegister(uint8_t reg)
{
	_delay_us(10); // for safety precautions
	clear_bit(PORTB, SS); // select nRF as slave to communicate to
	_delay_us(10);
	// R_REGISTER is redudant here because it's 0x00
	spi_Send_Receive(R_REGISTER + reg); // Read register command + register
	_delay_us(10);
	reg = spi_Send_Receive(NOP); // send dummy byte to receive 1 byte
	_delay_us(10);
	set_bit(PORTB, SS); // deselect nRF -> finish procedure
	return reg;
}

// fucntion to read/write to/from nrf multiple bytes
uint8_t *ReadWriteNRF(uint8_t R_W, uint8_t reg, uint8_t *data, uint8_t size)
{
	cli();
	// if write operation is required. W_TX_PAYLOAD cannot be used with write instruction
	if(R_W == W && reg != W_TX_PAYLOAD && reg != FLUSH_TX)
	{
		reg = W_REGISTER + reg; // append write instruction to specified register
	}
	
	// Read instruction is 0x00 so there is no point in adding it to reg
	
	static uint8_t returnedData[32]; // 32bytes are maximum???
	
	_delay_us(10);
	clear_bit(PORTB, SS); // enable slave
	_delay_us(10);
	spi_Send_Receive(reg); // send instruction + register
	_delay_us(10);
	
	for (int i = 0; i < size; i++)
	{
		// if receive data
		if (R_W == R && reg != W_TX_PAYLOAD)
		{
			returnedData[i] = spi_Send_Receive(NOP); // send dummy byte to shift
			_delay_us(10);							 // data into array
		}
		// if send data
		else
		{
			spi_Send_Receive(data[i]);
			_delay_us(10);
		}
	}
	
	set_bit(PORTB, SS); // disable slave
	sei();
	return returnedData;
}

void Nrf24_init(uint8_t pipe, uint8_t *addrRX, uint8_t *addrTX,  char mode)
{
	// define 5 byte array
	uint8_t values[5];
	
	_delay_ms(100);
	
	// enable auto acknowledgement for certaint pipe
	values[0] = 0x01 + pipe;
	ReadWriteNRF(W, EN_AA, values, 1);
	
	values[0] = 0x2F; // 500uS delay and 15 retries to send data if failed
	ReadWriteNRF(W, SETUP_RETR, values, 1);
	
	//enable pipe
	values[0] = 0x01 + pipe;
	ReadWriteNRF(W, EN_RXADDR, values, 1);
	
	// Setup address width. 0x03 for 5 bytes long
	values[0] = 0x03;
	ReadWriteNRF(W, SETUP_AW, values, 1);
	
	// RF channel setup (2400 - 2525) 1MHz step. 125 channels
	values[0] = 0x01; // 1st channel
	ReadWriteNRF(W, RF_CH, values, 1);
	
	// RF power mode and data speed setup
	values[0] = 0x07; // 0b0000 0111 - 1Mbps(longer range), 0 dBm
	ReadWriteNRF(W, RF_SETUP, values, 1);
	
	// setup RX, TX addresses and Payload size (5bytes)
	values[0] = 0x05;
	switch (pipe)
	{
		case 0:
		ReadWriteNRF(W, RX_ADDR_P0, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrTX, 5);
		ReadWriteNRF(W, RX_PW_P0, values, 1);
		break;
		case 1:
		ReadWriteNRF(W, RX_ADDR_P1, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrTX, 5);
		ReadWriteNRF(W, RX_PW_P1, values, 1);
		break;
		case 2:
		ReadWriteNRF(W, RX_ADDR_P2, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrTX, 5);
		ReadWriteNRF(W, RX_PW_P2, values, 1);
		break;
		case 3:
		ReadWriteNRF(W, RX_ADDR_P3, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrTX, 5);
		ReadWriteNRF(W, RX_PW_P3, values, 1);
		break;
		case 4:
		ReadWriteNRF(W, RX_ADDR_P4, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrTX, 5);
		ReadWriteNRF(W, RX_PW_P4, values, 1);
		break;
		case 5:
		ReadWriteNRF(W, RX_ADDR_P5, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrTX, 5);
		ReadWriteNRF(W, RX_PW_P5, values, 1);
		break;
	}
	
	// set nRF mode - receiver or transmitter
	if(mode == 'T') // set module as TX
	{
		// CONFIG register setup (transmitter, pwr up)
		values[0] = 0x1E;//0b0001 1110
		ReadWriteNRF(W, CONFIG, values, 1);
	}
	else // set module as RX
	{
		// CONFIG register setup (receiver, pwr up)
		values[0] = 0x1F;//0b0001 1111
		ReadWriteNRF(W, CONFIG, values, 1);
	}
	
	_delay_ms(100);
}

void transmit_data(uint8_t *data)
{
	ReadWriteNRF(R, FLUSH_TX, data, 0); // clear the buffer
	ReadWriteNRF(R, W_TX_PAYLOAD, data, 5); // because payload is 5 bytes
	
	_delay_ms(10);
	set_bit(PORTB, CE); // enable nrf for TX
	_delay_us(50); // wait atleast 10uS
	clear_bit(PORTB, CE); // disable TX
	_delay_us(10);
	resetNrf();
}

void receive_data(void)
{
	sei(); // enable interrupts if used
	
	set_bit(PORTB, CE); // enable for listening
	_delay_ms(2000);
	clear_bit(PORTB, CE); // stop listening
	cli(); // disable all interrupts
	resetNrf();
}

// after every received/transmitted payload IRQs must be reseted
void resetNrf(void)
{
	_delay_us(10);
	clear_bit(PORTB, SS); // enable slave
	_delay_us(10);
	spi_Send_Receive(W_REGISTER + STATUS);
	_delay_us(10);
	spi_Send_Receive(0x70); // reset all irq in statys register
	_delay_us(10);
	set_bit(PORTB, SS); // disable slave
}

// change nrf into receiver mode
void changeNrfToRX()
{
	uint8_t values[1];
	// CONFIG register setup (receiver, pwr up)
	values[0] = 0x1F;//0b0001 1111
	ReadWriteNRF(W, CONFIG, values, 1);
	_delay_ms(100);
}

// change nrf into transmitter mode
void changeNrfToTX()
{
	uint8_t values[1];
	// CONFIG register setup (transmitter, pwr up)
	values[0] = 0x1E;//0b0001 1110
	ReadWriteNRF(W, CONFIG, values, 1);
	_delay_ms(100);
}
#endif