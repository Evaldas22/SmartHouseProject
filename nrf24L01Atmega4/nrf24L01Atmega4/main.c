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
#define DEVICE_ADDR				0x17
#define DEVICE_ANSWER_START		0x31

#define FIND_STATE				0x52
#define RELAY_TOGGLE			0x51
#define RELAY1_STATE			DEVICE_ADDR + 1
#define RELAY2_STATE			DEVICE_ADDR + 2
#define ERROR					0x05

#define DHT22_PIN				PC0
#define RELAY_IN1				PA0
#define RELAY_IN2				PA1

#define MAX_RETRIES				10

#define DHT22_REQUEST			0x70
// ---------------------------------------------------------------

// --------------------------Functions----------------------------
void SendAnswerUntilReceived(uint8_t answerByte, uint8_t state);
void SendAnswer();
void SendDHTData();
uint8_t CheckInput(unsigned char PIN, unsigned char pinToCheck);
void convertToIntArray(double *doubleArray, int *values);
void initRelays(uint8_t *relayPins);
void ToggleLights(uint8_t relay);
// ---------------------------------------------------------------
// ------------------------GLOBAL VARIABLES-----------------------------------
volatile uint8_t successfullySend = 0;
uint8_t lastRelay1State;

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
	
	uint8_t relays[] = {RELAY_IN1, RELAY_IN2};
	initRelays(relays);
	
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
		
		else if(data[3] == RELAY_TOGGLE)
		{
			if(data[2] == RELAY1_STATE)
			{
				ToggleLights(RELAY_IN1);
				lastRelay1State = !(CheckInput(PINA, RELAY_IN1));
			}
			else if(data[2] == RELAY2_STATE)
			{
				ToggleLights(RELAY_IN2);
				lastRelay1State = !(CheckInput(PINA, RELAY_IN2));
			}
		}
		
		else if(data[3] == FIND_STATE)
		{
			if(data[2] == RELAY1_STATE)
			{
				uint8_t stateRelay1 = !(CheckInput(PINA, RELAY_IN1));
				uint8_t stateAnswer[5] = {0x31, 0x42, 0x43, 0x44, stateRelay1};
				SendAnswer(stateAnswer);
			}
			else if(data[2] == RELAY2_STATE)
			{
				uint8_t stateRelay1 = !(CheckInput(PINA, RELAY_IN2));
				uint8_t stateAnswer[5] = {0x31, 0x42, 0x43, 0x44, stateRelay1};
				SendAnswer(stateAnswer);
			}
			else
			{
				uint8_t stateAnswer[5] = {0x31, 0x42, 0x43, 0x44, ERROR};
				SendAnswer(stateAnswer);
			}
		}
		
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

void initRelays(uint8_t *relayPins)
{
	for (int i = 0; i < (sizeof(relayPins) / sizeof(uint8_t)); i++)
	{
		set_bit(DDRA, relayPins[i]); // OUTPUT
		set_bit(PORTA, relayPins[i]); // HIGH
	}
}

void ToggleLights(uint8_t relay)
{
	switch(relay)
	{
		case RELAY_IN1:
			invert_bit(PORTA, RELAY_IN1);
			_delay_ms(50);
			break;
			
		case RELAY_IN2:
			invert_bit(PORTA, RELAY_IN2);
			_delay_ms(50);
			break;
	}
}



