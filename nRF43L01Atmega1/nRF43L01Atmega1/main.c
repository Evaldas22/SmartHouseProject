/*
 * nRF43L01Atmega1.c
 *
 * Created: 2018-02-21 1:13:56 PM
 * Author : EVALDAS-PC
 */ 

// MCU clock
#define F_CPU 16000000UL

// includes
#include "Basic_operations.h"

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "SPI.h"
#include "nRF24L01.h"
#include "UART.h"
#include "ExternalInterrupts.h"

// defines
#define W 1
#define R 0

// Functions
void RequestFrom(uint8_t *addr);
uint8_t ReadRegister(uint8_t reg);
void WriteByteToNRF(uint8_t reg, uint8_t data);
uint8_t *ReadWriteNRF(uint8_t R_W, uint8_t reg, uint8_t *data, uint8_t size);
void Nrf24_init(uint8_t pipe, uint8_t *addrRX, uint8_t *addrTX,  char mode);
void transmit_data(uint8_t *data);
void receive_data(void);
void resetNrf(void);
void Add_pipe_with_address(uint8_t pipe, uint8_t *addrRX);
void changeNrfToRX();
void changeNrfToTX();

// global variable for storing data
volatile uint8_t *data;
volatile uint8_t TX = 0;

int main(void)
{
	// LED in PB0, HIGH (for indicating that MCU works)
	set_bit(DDRB, PB0);
	set_bit(PORTB, PB0);
	
    Uart_Init(); // for debugging purposes
	spi_master_init(); // enable SPI
	
	// set addresses of nrf. Must match to use auto ACK
	uint8_t rxAddrFirstTX[5] = {0x12, 0x12, 0x12, 0x12, 0x12};
	uint8_t txAddrFirstTX[5] = {0x12, 0x12, 0x12, 0x12, 0x12};
		
	// command for atmega16
	uint8_t atmega16Address[5] = {0x41, 0x42, 0x43, 0x44, 0x16};
	
	// command for atmega328p	
	uint8_t atmega328Address[5] = {0x11, 0x11, 0x11, 0x11, 0x32};
		
	// first command will be sent
	Nrf24_init(0, rxAddrFirstTX, txAddrFirstTX, 'T');
	
	// enable external interrupt
	EnableExternalInterrupt0();
	
	// flush any IRQs from nrf before begining 
	resetNrf();	
	
    while (1) 
    {	
		//RequestFrom(atmega328Address);
		//_delay_ms(1000);
		RequestFrom(atmega16Address);
		_delay_ms(1000);
	}
}
void RequestFrom(uint8_t *addr)
{
	TX = 1;
	transmit_data(addr);
	_delay_us(100);
	TX = 0;
	changeNrfToRX();
	receive_data();
	//_delay_ms(2000);
	changeNrfToTX();
	TX = 1;
}

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


// reg is memory address. This function only send one byte of data
void WriteByteToNRF(uint8_t reg, uint8_t data)
{
	_delay_us(10); // profilaktiskas laukimas
	clear_bit(PORTB, SS); // select slave (nRF)
	_delay_us(10);
	spi_Send_Receive(W_REGISTER + reg); // write to specified register 'reg'
	_delay_us(10);
	spi_Send_Receive(data); // shift data into specified register
	_delay_us(10);
	set_bit(PORTB, SS); // disable slave
}


// fucntion to read/write to nrf multiple bytes
uint8_t *ReadWriteNRF(uint8_t R_W, uint8_t reg, uint8_t *data, uint8_t size)
{
	cli();
	
	// if write operation is required. W_TX_PAYLOAD cannot be used with write instruction
	if (R_W == W && reg != W_TX_PAYLOAD && reg != FLUSH_TX)
	{
		reg = W_REGISTER + reg;
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
		if (R_W == R)
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
	// define 1 byte array
	uint8_t values[5];
	
	_delay_ms(100);
	
	// enable auto acknowledgement for certaint pipe
	values[0] = 0x01 + pipe;
	ReadWriteNRF(W, EN_AA, values, 1);
	
	values[0] = 0x1F; // 750uS delay and 15 retries to send data if failed
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
	ReadWriteNRF(W, RF_SETUP, values, 1);;
	
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
	
	// for reaching stand by mode
	_delay_ms(100);
}

void transmit_data(uint8_t *data)
{
	ReadWriteNRF(W, FLUSH_TX, data, 0); // clear the buffer
	ReadWriteNRF(W, W_TX_PAYLOAD, data, 5); // because payload is 5 bytes
	
	sei(); // if interrupts are used
	
	_delay_ms(10);
	set_bit(PORTB, CE); // enable nrf for TX
	_delay_us(50); // wait atleast 10uS
	clear_bit(PORTB, CE); // disable TX
	_delay_us(10);
	cli();
	resetNrf();
}

void receive_data(void)
{
	sei(); // enable interrupts if used
	
	set_bit(PORTB, CE); // enable for listening
	_delay_ms(3000); // listen for 3s
	clear_bit(PORTB, CE); // stop listening
	cli(); // disable all interrupts
	resetNrf();
}

// after every received/transmitted payload IRQ must be reseted
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

void Add_pipe_with_address(uint8_t pipe, uint8_t *addrRX)
{
	// define 1 byte array
	uint8_t values[1];
	
	_delay_ms(100);
	
	// enable auto acknowledgement for certaint pipe
	values[0] = 0x01 + pipe;
	ReadWriteNRF(W, EN_AA, values, 1);
	
	//enable pipe
	values[0] = 0x01 + pipe;
	ReadWriteNRF(W, EN_RXADDR, values, 1);
	
	// setup RX, TX addresses
	values[0] = 0x05;
	switch (pipe)
	{
		case 0:
		ReadWriteNRF(W, RX_ADDR_P0, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrRX, 5);
		ReadWriteNRF(W, RX_PW_P0, values, 1);
		break;
		case 1:
		ReadWriteNRF(W, RX_ADDR_P1, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrRX, 5);
		ReadWriteNRF(W, RX_PW_P1, values, 1);
		break;
		case 2:
		ReadWriteNRF(W, RX_ADDR_P2, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrRX, 5);
		ReadWriteNRF(W, RX_PW_P2, values, 1);
		break;
		case 3:
		ReadWriteNRF(W, RX_ADDR_P3, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrRX, 5);
		ReadWriteNRF(W, RX_PW_P3, values, 1);
		break;
		case 4:
		ReadWriteNRF(W, RX_ADDR_P4, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrRX, 5);
		ReadWriteNRF(W, RX_PW_P4, values, 1);
		break;
		case 5:
		ReadWriteNRF(W, RX_ADDR_P5, addrRX, 5);
		ReadWriteNRF(W, TX_ADDR, addrRX, 5);
		ReadWriteNRF(W, RX_PW_P5, values, 1);
		break;
	}
	
	// for reaching stand by mode
	_delay_ms(100);
}

void changeNrfToRX()
{
	uint8_t values[1];
	// CONFIG register setup (receiver, pwr up)
	values[0] = 0x1F;//0b0001 1111
	ReadWriteNRF(W, CONFIG, values, 1);
	_delay_ms(100);
}

void changeNrfToTX()
{
	uint8_t values[1];
	// CONFIG register setup (transmitter, pwr up)
	values[0] = 0x1E;//0b0001 1110
	ReadWriteNRF(W, CONFIG, values, 1);
	_delay_ms(100);
}

ISR(INT0_vect)
{
	cli(); // disable interrupt
	clear_bit(PORTB, CE); // disable chip to stop listening
	
	if (!TX)
	{		
		data = ReadWriteNRF(R, R_RX_PAYLOAD, data, 5);
		resetNrf();
		
		// ignore any data that come not from these 2 TX'es
		if(data[0] == 0x41)
		{
			Uart_Send_String("It's atmega16 sending: ");
			// send received data to pc to be displayed on terminal
			for (int i = 0; i < 5; i++)
			{
				Uart_Transmit(data[i]);
			}
			Uart_Transmit(0x0D);
		}
		else if(data[0] == 0x50)
		{
			Uart_Send_String("It's atmega328p sending: ");
			// send received data to pc to be displayed on terminal
			for (int i = 0; i < 5; i++)
			{
				Uart_Transmit(data[i]);
			}
			Uart_Transmit(0x0D);
		} 
	}
	else
	{	
		resetNrf();
	}
	sei(); // re-enable interrupts again
}