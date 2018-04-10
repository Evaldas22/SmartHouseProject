/*
 * nRF43L01Atmega2(TX).c
 *
 * Created: 2018-02-24 10:51:12 AM
 * Author : EVALDAS-PC
 */ 

// MCU clock
#define F_CPU 16000000UL

// includes
#include "Basic_operations.h"

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "SPI.h"
#include "nRF24L01.h"
#include "UART.h"
#include "ExternalInterrupts.h"
#include "ButtonPressed.h"

// defines
#define W 1
#define R 0
#define DHT11_PIN 6
#define BUTTON_PIN 4

#define MAX_RETRIES 10

// Functions
void SendAnswer();
uint8_t ReadRegister(uint8_t register);
uint8_t *ReadWriteNRF(uint8_t R_W, uint8_t reg, uint8_t *data, uint8_t size);
void Nrf24_init(uint8_t pipe, uint8_t *addrRX, uint8_t *addrTX,  char mode);
void transmit_data(uint8_t *data);
void receive_data(void);
void resetNrf(void);
void changeNrfToRX();
void changeNrfToTX();
void SendDHT11Data();
void Request();
void Response();
uint8_t Receive_data_DHT11();

void CheckButtonState();

// global variable for storing any received data
volatile uint8_t *data;

volatile uint8_t successfullySend = 0;

// global var for DHT11 sensor
uint8_t I_RH,D_RH,I_Temp,D_Temp,CheckSum, temp = 0;
int8_t humidity, temperature;

int main(void)
{
	// init button
	clear_bit(DDRD, BUTTON_PIN); // input
	set_bit(PORTD, BUTTON_PIN); //pull-up
	
	// LED in PB0, LOW 
	set_bit(DDRB, PB0);
	set_bit(PORTB, PB0);
	
    Uart_Init(); // for debugging purposes
	spi_master_init(); // enable SPI
	
	// set nrf addresses
	uint8_t rxAddr[5] = {0x12, 0x12, 0x12, 0x12, 0x12};
	uint8_t txAddr[5] = {0x12, 0x12, 0x12, 0x12, 0x12};
		
	// first nrf will listen for command
	Nrf24_init(0, rxAddr, txAddr, 'R');
	
	//enable external interupt INT0
	EnableExternalInterrupt0();
	
	// flush any IRQs from nrf before begining
	resetNrf();
	
	Uart_Send_String("Working\n");
	
    while (1) 
    {
		receive_data();
		CheckButtonState();
    }
}

void CheckButtonState()
{
	if(!check_bit(PIND, BUTTON_PIN)){
		uint8_t retries = 0;
		Uart_Send_String("Button is pressed\n");
		
		while((successfullySend != 1) && (retries < MAX_RETRIES))
		{
			Uart_Send_String("Sending that to Rpi\n");
			uint8_t buttonPressedData[5] = {0x41, 0x42, 0x43, 0x44, 0xF1};
			SendAnswer(buttonPressedData);
			receive_data();
			retries++;
		}
		successfullySend = 0;
	}
}

void SendAnswer(uint8_t *data)
{
	resetNrf();
	changeNrfToTX(); // change to TX mode
	transmit_data(data); // transmit data
	_delay_us(100); // some delay for safe transmission
	changeNrfToRX(); // change back to RX mode
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

// fucntion to read/write to nrf multiple bytes
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
	
	values[0] = 0x1F; // 500uS delay and 15 retries to send data if failed
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

// this interrupt will be triggered when transmission/receive is succesfull
// this handles all the answers that need to be sent
ISR(INT0_vect)
{
	// default answer
	uint8_t dataToSend[5] = {0x41, 0x42, 0x43, 0x44, 0x45}; // ABCDE
	cli(); // disable interrupt 
	clear_bit(PORTB, CE); // disable chip to stop listening
	
	data = ReadWriteNRF(R, R_RX_PAYLOAD, data, 5);
		
	// if message is for this atmega
	if (data[4] == 0x16)
	{
		Uart_Send_String("Received command: "); Uart_Transmit(data[3]);
		Uart_Send_String("\n");
		
		if(data[3] == 0x70) SendDHT11Data();
		else if(data[3] == 0x66) successfullySend = 1;
		else SendAnswer(dataToSend);
	}
	
	sei(); // re-enable interrupts again
}

void SendDHT11Data()
{
	Request();		//send start pulse 
	Response();		// receive response 
	humidity = Receive_data_DHT11();	// store first eight bit in I_RH 
	D_RH =Receive_data_DHT11();			// store next eight bit in D_RH 
	temperature = Receive_data_DHT11();	// store next eight bit in I_Temp 
	D_Temp = Receive_data_DHT11();		// store next eight bit in D_Temp 
	CheckSum = Receive_data_DHT11();	// store next eight bit in CheckSum 
	
	// check sum
	if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
	{
		Uart_Send_String("Error with check sum\n");
	}
	// display via uart
	char temp[2], dregme[2];
	sprintf(temp, "%d", temperature);
	Uart_Send_String("Temperature: "); Uart_Send_String(temp); Uart_Send_String("\n");
	
	sprintf(dregme, "%d", humidity);
	Uart_Send_String("Dregme: "); Uart_Send_String(dregme); Uart_Send_String("\n");
	
	// send actual data back
	uint8_t answer[5] = {0x41, 0x42, 0x43, temperature, humidity};
	SendAnswer(answer);
	
	_delay_ms(1100); // sampling period is 1 second. If less, DHT will fail
}

void Request()
{
	set_bit(DDRD, DHT11_PIN);		//set as output
	clear_bit(PORTD, DHT11_PIN);	// set to low pin 
	_delay_ms(20);					// wait for 20ms 
	set_bit(PORTD, DHT11_PIN);		// set to high pin 
}

void Response()
{
	clear_bit(DDRD, DHT11_PIN);			// now set as input		
	while(check_bit(PIND, DHT11_PIN));		// wait for first response(20-40uS)
	while((check_bit(PIND, DHT11_PIN))==0);	// wait for LOW response(80uS)
	while(check_bit(PIND, DHT11_PIN));		// wait for preparation HIGH(80uS)
}

uint8_t Receive_data_DHT11()
{
	for (int q=0; q<8; q++)
	{
		while(!(check_bit(PIND, DHT11_PIN)));  // while first part is LOW
		
		_delay_us(30); // after 30us see what signal it is
		// if still high - means it's "1"
		if(check_bit(PIND, DHT11_PIN)) temp = (temp << 1) | 0x01;
		// if it's low - means other bit started and this one was 0
		else temp = temp << 1;
		
		while(check_bit(PIND, DHT11_PIN)); // wait for HIGH signal to end
	}
	return temp;
}

