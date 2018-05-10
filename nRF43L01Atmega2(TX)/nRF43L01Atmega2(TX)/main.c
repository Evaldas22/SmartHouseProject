/*
 * nRF43L01Atmega2(TX).c
 *
 * Created: 2018-02-24 10:51:12 AM
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
// ---------------------------------------------------------------

// ---------------------------defines-----------------------------
#define DEVICE_ADDR 0x16
#define DEVICE_ANSWER_START 0x41

#define DHT11_PIN PD6
#define RELAY1 PA1
#define PIR_SENSOR PA3
#define LIGHT_SWITCH PA0

#define MAX_RETRIES 10

#define RELAY1_STATE 0xF1
#define STATE_UPDATE_ANSWER 0x66
#define DHT11_REQUEST 0x70
#define RELAY_TOGGLE 0x51
#define FIND_STATE 0x52
#define PIR_STATE 0x30
#define ERROR 0x05
#define TURN_OFF_PIR 0xA1
#define TURN_ON_PIR 0xA0
// ---------------------------------------------------------------

// --------------------------Functions----------------------------
void CheckPirSensor();
void CheckLightSwitches();
void SendAnswerUntilReceived(uint8_t answerByte, uint8_t state);
void SendAnswer();
void SendDHT11Data();
void Request();
void Response();
uint8_t Receive_data_DHT11();
void CheckRelays();
void initRelays();
void ToggleLights(uint8_t relay);
void initPIRSensor();
void initLightSwitch();
uint8_t CheckInput(unsigned char PIN, unsigned char pinToCheck);
// ---------------------------------------------------------------
// ------------------------GLOBAL VARIABLES-----------------------------------
volatile uint8_t successfullySend = 0;

// global var for DHT11 sensor
uint8_t humidity, D_RH, temperature, D_Temp, CheckSum, temp = 0;
uint8_t lastRelay1State;
uint8_t lastSwitchState;
uint8_t PIR_Enabled = 1;

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
	
	// initialize all peripherals
	initRelays();
	initPIRSensor();
	initLightSwitch();
	
	EnableExternalInterrupt0();
	
	// flush any IRQs from nrf before begining
	resetNrf();
	
	Uart_Send_String("Working\n");

	lastRelay1State = CheckInput(PINA, RELAY1);
	lastSwitchState = CheckInput(PINA, LIGHT_SWITCH);
	
    while (1) 
    {
		receive_data();
		CheckLightSwitches();
		CheckRelays();
		CheckPirSensor();
    }
}

void CheckPirSensor()
{
	uint8_t PIRState = CheckInput(PINA, PIR_SENSOR);
		
	// if motion sensor pin is HIGH -> motion detected -> send that to server
	if(PIRState && PIR_Enabled)
	{
		SendAnswerUntilReceived(PIR_STATE, PIRState);
	}
}

void CheckLightSwitches()
{
	uint8_t currentSwitchState = CheckInput(PINA, LIGHT_SWITCH);
	
	// if switch was toggled
	if(currentSwitchState != lastSwitchState)
	{
		ToggleLights(RELAY1);
		lastSwitchState = currentSwitchState;
	}
}

void CheckRelays()
{
	uint8_t currentState = CheckInput(PINA, RELAY1);
	
	// if relay state changed
	if(lastRelay1State != currentState)
	{
		SendAnswerUntilReceived(RELAY1_STATE, currentState);
		lastRelay1State = currentState;
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
	
	uint8_t defaultAnswer[5] = {DEVICE_ANSWER_START, 0x42, 0x43, 0x44, 0x45}; // ABCDE
		
	cli(); // MUST disable interrupts
	clear_bit(PORTB, CE); // STOP LISTENING
	
	data = ReadWriteNRF(R, R_RX_PAYLOAD, data, 5);
		
	// if message is meant for this atmega
	if (data[4] == DEVICE_ADDR)
	{		
		if(data[3] == DHT11_REQUEST) SendDHT11Data();
		
		else if(data[3] == STATE_UPDATE_ANSWER) successfullySend = 1;
		
		else if(data[3] == RELAY_TOGGLE) 
		{
			if(data[2] == RELAY1_STATE)
			{
				ToggleLights(RELAY1);
				lastRelay1State = CheckInput(PINA, RELAY1);
			}
		}
		
		else if(data[3] == FIND_STATE)
		{
			if(data[2] == RELAY1_STATE)
			{
				uint8_t stateRelay1 = CheckInput(PINA, RELAY1);
				uint8_t stateAnswer[5] = {0x41, 0x42, 0x43, 0x44, stateRelay1};
				SendAnswer(stateAnswer);
			}
			else
			{
				uint8_t stateAnswer[5] = {0x41, 0x42, 0x43, 0x44, ERROR};
				SendAnswer(stateAnswer);
			}
		}
		
		else if(data[3] == TURN_OFF_PIR)
		{
			 PIR_Enabled = 0;
			 SendAnswer(defaultAnswer);
		}
		
		else if(data[3] == TURN_ON_PIR)
		{
			PIR_Enabled = 1;
			SendAnswer(defaultAnswer);
		}
		
		else SendAnswer(defaultAnswer);
	}
	
	sei(); // re-enable interrupts again
}

void SendDHT11Data()
{
	Request();
	Response();
	
	// after sucessful request collect all data
	humidity = Receive_data_DHT11();	
	D_RH =Receive_data_DHT11();		
	temperature = Receive_data_DHT11();	
	D_Temp = Receive_data_DHT11();	
	CheckSum = Receive_data_DHT11();
	
	// check sum
	if ((humidity + D_RH + temperature + D_Temp) != CheckSum){}
	
	// send actual data back
	uint8_t answer[5] = {0x41, 0x42, 0x43, temperature, humidity};
	SendAnswer(answer);
	
	_delay_ms(1100); // sampling period is 1 second. If less, DHT will fail
}

// Request function for custom DHT11/22 protocol
void Request()
{
	set_bit(DDRD, DHT11_PIN);
	clear_bit(PORTD, DHT11_PIN); 
	_delay_ms(20);
	set_bit(PORTD, DHT11_PIN);
}

// Response function for custom DHT11/22 protocol
void Response()
{
	clear_bit(DDRD, DHT11_PIN);	
	// because of poor wires these whiles can freeze whole program
	while(check_bit(PIND, DHT11_PIN));		// wait for first response(20-40uS)
	while((check_bit(PIND, DHT11_PIN))==0);	// wait for LOW response(80uS)
	while(check_bit(PIND, DHT11_PIN));		// wait for preparation HIGH(80uS)
}

uint8_t Receive_data_DHT11()
{
	for (int q=0; q<8; q++)
	{
		while(!(check_bit(PIND, DHT11_PIN)));  // while first part is LOW
		
		_delay_us(30); // after 30us take sample
		// if still high - means it's "1"
		if(check_bit(PIND, DHT11_PIN)) temp = (temp << 1) | 0x01;
		// if it's low - means other bit started and this one was 0
		else temp = temp << 1;
		
		while(check_bit(PIND, DHT11_PIN)); // wait for HIGH signal to end
	}
	return temp;
}

void initRelays()
{
	set_bit(DDRA, RELAY1);	  // OUTPUT
	clear_bit(PORTA, RELAY1); // LOW
}

void ToggleLights(uint8_t relay)
{
	if(relay == RELAY1)
	{
		invert_bit(PORTA, RELAY1);
		_delay_ms(50);
	}
}

void initPIRSensor()
{
	clear_bit(DDRA, PIR_SENSOR); // INPUT
}

void initLightSwitch()
{
	clear_bit(DDRA, LIGHT_SWITCH); // INPUT
	set_bit(PORTA, LIGHT_SWITCH);  // PULL-UP
}

uint8_t CheckInput(unsigned char PIN, unsigned char pinToCheck)
{
	return check_bit(PIN, pinToCheck);
}



