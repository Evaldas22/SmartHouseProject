#ifndef nrf24
#define nrf24

#include "nRF24L01.h"
#include <wiringPi.h>
#include <bcm2835.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>


// ---------------------Defines-----------------------------
#define _delay_us(x) bcm2835_delayMicroseconds(x)
#define _delay_ms(x) bcm2835_delay(x)
#define set_bit(pin) bcm2835_gpio_write(pin, HIGH)
#define clear_bit(pin) bcm2835_gpio_write(pin, LOW)
#define W 1
#define R 0

// Some pins
#define CE RPI_GPIO_P1_22 // GPIO25
#define SS RPI_GPIO_P1_18 // GPIO24. I'll use this to manually control NRF Slave select pin
//#define IRQ_PIN 29 // GPIO21

//-------------------------functions------------------------
uint8_t ReadRegister(uint8_t reg);
uint8_t * ReadWriteNRF(uint8_t R_W, uint8_t reg, uint8_t *data, uint8_t size);
void Nrf24_init(uint8_t pipe, uint8_t *addrRX, uint8_t *addrTX,  char mode);
void transmit_data(uint8_t *data);
void resetNrf(void);
void changeNrfToRX();
void changeNrfToTX();
// ---------------------------------------------------------

// reg is memory address
uint8_t ReadRegister(uint8_t reg)
{
	_delay_us(10); // for safety precautions
    clear_bit(SS);
    _delay_us(10);

	// R_REGISTER is redudant here because it's 0x00
	bcm2835_spi_transfer(R_REGISTER | reg); // Read register command + register
	_delay_us(10);

	reg = bcm2835_spi_transfer(NOP); // send dummy byte to receive 1 byte
	_delay_us(10);

	set_bit(SS);
	return reg;
}

uint8_t *ReadWriteNRF(uint8_t R_W, uint8_t reg, uint8_t *data, uint8_t size)
{
	// if write operation is required. W_TX_PAYLOAD cannot be used with write instruction
	if (R_W == W && reg != W_TX_PAYLOAD && reg != FLUSH_TX && reg != FLUSH_RX)
	{
		reg = W_REGISTER + reg;
	}

	// Read instruction is 0x00 so there is no point in adding it to reg

	static uint8_t returnedData[32]; // 32bytes are maximum

	_delay_us(10);
	clear_bit(SS); // enable slave
	_delay_us(10);
	bcm2835_spi_transfer(reg);
	_delay_us(10);

	for (int i = 0; i < size; i++)
	{
		// if receive data
		if (R_W == R)
		{
			returnedData[i] = bcm2835_spi_transfer(NOP); // send dummy byte to shift
			_delay_us(10);							     // data into array
		}
		// if send data
		else
		{
			bcm2835_spi_transfer(data[i]);
			_delay_us(10);
		}
	}

	set_bit(SS); // disable slave
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

	values[0] = 0x2F; // 750uS delay and 15 retries to send data if failed
	ReadWriteNRF(W, SETUP_RETR, values, 1);

	//enable pipe
	values[0] = 0x01 + pipe;
	ReadWriteNRF(W, EN_RXADDR, values, 1);

	// Setup address width. 0x03 for 5 bytes long
	values[0] = 0x03;
	ReadWriteNRF(W, SETUP_AW, values, 1);

	// RF channel setup (2400 - 2525) 1MHz step. 125 channels
	values[0] = 0x6C; // 108 channel
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
		values[0] = 0x5E;//0b0101 1110
		ReadWriteNRF(W, CONFIG, values, 1);
	}
	else // set module as RX
	{
		// CONFIG register setup (receiver, pwr up)
		values[0] = 0x5F;//0b0101 1110
		ReadWriteNRF(W, CONFIG, values, 1);
	}

	// for reaching stand by mode
	_delay_ms(100);
}

void transmit_data(uint8_t *data)
{
	ReadWriteNRF(W, FLUSH_TX, data, 0); // clear the buffer
	ReadWriteNRF(W, W_TX_PAYLOAD, data, 5); // because payload is 5 bytes

	_delay_ms(10);
	set_bit(CE); // enable nrf for TX
	_delay_us(1000); // wait atleast 10uS
	clear_bit(CE); // disable TX
	_delay_us(10);

	resetNrf();
}

// after every received/transmitted payload IRQ must be reseted
void resetNrf(void)
{
	_delay_us(10);
	clear_bit(SS); // enable slave
	_delay_us(10);
	bcm2835_spi_transfer(W_REGISTER + STATUS);
	_delay_us(10);
	bcm2835_spi_transfer(0x70); // reset all irq in status register
	_delay_us(10);
	set_bit(SS); // disable slave
}

void changeNrfToRX()
{
	uint8_t values[1];
	// CONFIG register setup (receiver, pwr up)
	values[0] = 0x5F;//0b0101 1110
	ReadWriteNRF(W, CONFIG, values, 1);
	_delay_ms(100);
}

void changeNrfToTX()
{
	uint8_t values[1];
	// CONFIG register setup (transmitter, pwr up)
	values[0] = 0x5E;//0b0101 1110
	ReadWriteNRF(W, CONFIG, values, 1);
	_delay_ms(100);
}

#endif
