/*
 * nrf24L01Atmega4.c
 *
 * Created: 2018-05-14 11:18:01 AM
 * Author : EVALDAS-PC
 */ 

// MCU clock
#define F_CPU 16000000UL

// ----------------------------includes----------------------------
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
#include "nrf24.h"
#include "DHT.h"
// ---------------------------------------------------------------

// ---------------------------defines-----------------------------
#define DEVICE_ADDR 0x17
#define DEVICE_ANSWER_START 0x31

#define DHT22_PIN PC0

#define MAX_RETRIES 10

#define DHT22_REQUEST 0x70
// ---------------------------------------------------------------

// --------------------------Functions----------------------------
void SendAnswerUntilReceived(uint8_t answerByte, uint8_t state);
void SendAnswer();
void SendDHTData();
uint8_t CheckInput(unsigned char PIN, unsigned char pinToCheck);
void convertToIntArray(double *doubleArray, int *values);
// ---------------------------------------------------------------
// ------------------------GLOBAL VARIABLES-----------------------------------
volatile uint8_t successfullySend = 0;

// global var for DHT11 sensor

int main(void)
{
	// LED in PB0, LOW. Remove to save power
	set_bit(DDRB, PB0);
	set_bit(PORTB, PB0);
	
    Uart_Init(); // for debugging purposes
	spi_master_init(); // enable SPI
	
	// set nrf addresses
	uint8_t rxAddr[5] = {0x12, 0x12, 0x12, 0x12, 0x12};
	uint8_t txAddr[5] = {0x12, 0x12, 0x12, 0x12, 0x12};
		
	// pipe - 0
	Nrf24_init(0, rxAddr, txAddr, 'R');
	
	EnableExternalInterrupt0();

	// flush any IRQs from nrf before begining
	resetNrf();
	
	Uart_Send_String("Working\n");
	
    while (1) 
    {
		receive_data();
    }
}

void SendAnswerUntilReceived(uint8_t answerByte, uint8_t state)
{
	uint8_t retries = 0;
	successfullySend = 0;
	
	while((successfullySend != 1) && (retries < MAX_RETRIES))
	{		
		uint8_t dataToSend[5] = {0x41, 0x42, 0x43, state, answerByte};
		SendAnswer(dataToSend);
		// receive response
		receive_data();
		retries++;
	}
	successfullySend = 0;
}

void SendAnswer(uint8_t *data)
{
	resetNrf();
	changeNrfToTX();
	transmit_data(data);
	_delay_us(100); // some delay for safe transmission
	changeNrfToRX();
}

// this interrupt will be triggered when received any data
// this handles all the answers that need to be sent
ISR(INT0_vect)
{
	uint8_t *data;
	
	uint8_t defaultAnswer[5] = {DEVICE_ANSWER_START, 0x42, 0x43, 0x44, 0x45};
		
	cli(); // MUST disable interrupts
	clear_bit(PORTB, CE); // STOP LISTENING
	
	data = ReadWriteNRF(R, R_RX_PAYLOAD, data, 5);
		
	// if message is meant for this atmega
	if (data[4] == DEVICE_ADDR)
	{		
		if(data[3] == DHT22_REQUEST) SendDHTData();
		
		else SendAnswer(defaultAnswer);
	}
	
	sei(); // re-enable interrupts again
}

void SendDHTData()
{
	double temp[1] = {0}, humid[1] = {0}; //Temperature and humidity
	DHT_Read(temp, humid); // sample rate is 0.5 Hz, so atleast wait 2 s
	
	int valuesTemp[2];
	convertToIntArray(temp, valuesTemp);
	
	int valuesHum[2];
	convertToIntArray(humid, valuesHum);
	
	uint8_t answer[5];
	answer[0] = DEVICE_ANSWER_START;
	answer[1] = valuesTemp[0];
	answer[2] = valuesTemp[1];
	answer[3] = valuesHum[0];
	answer[4] = valuesHum[1];
	SendAnswer(answer);
}

uint8_t CheckInput(unsigned char PIN, unsigned char pinToCheck)
{
	return check_bit(PIN, pinToCheck);
}

void convertToIntArray(double* doubleArray, int *values)
{
	int a,b;
	a = floor(doubleArray[0]);
	b = doubleArray[0] * pow(10,2) - a * pow(10,2);
	
	values[0] = 0;
	values[1] = 0;
	
	values[0] = a;
	values[1] = b;
}



