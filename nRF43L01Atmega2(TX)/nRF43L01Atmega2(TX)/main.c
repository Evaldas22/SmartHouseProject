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
#include "ButtonPressed.h"
#include "nrf24.h"
// ---------------------------------------------------------------

// ---------------------------defines-----------------------------
#define DHT11_PIN 6
#define RELAY1 PA1
#define PIR_SENSOR PA3

#define MAX_RETRIES 10

#define RELAY1_STATE 0xF1
#define EMERGENCY_ANSWER 0x66
#define DHT11_REQUEST 0x70
#define RELAY1_COMMAND 0x51
// ---------------------------------------------------------------

// --------------------------Functions----------------------------
void SendAnswer();
void SendDHT11Data();
void Request();
void Response();
uint8_t Receive_data_DHT11();
void CheckRelayState();
void initRelays();
void ToggleLights();
void initPIRSensor();
uint8_t CheckPIRSensor();
// ---------------------------------------------------------------

// global variable for storing any received data
volatile uint8_t *data;

volatile uint8_t successfullySend = 0;

// global var for DHT11 sensor
uint8_t I_RH,D_RH,I_Temp,D_Temp,CheckSum, temp = 0;
int8_t humidity, temperature;
uint8_t lastRelay1State = 0;

int main(void)
{
	
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
	
	initRelays();
	initPIRSensor();
	
    while (1) 
    {
		receive_data();
		//CheckRelayState();
    }
}

void CheckRelayState()
{
	uint8_t currentState = 0;
	
	if(!check_bit(PINA, RELAY1))
	{
		currentState = 0;
	}
	else currentState = 1;
	
	if(lastRelay1State != currentState)
	{
		uint8_t retries = 0;
		successfullySend = 0;
		
		// while data is not successfuly sent keep trying (no more than MAX_RETRIES)
		while((successfullySend != 1) && (retries < MAX_RETRIES))
		{
			Uart_Send_String("Sending that to Rpi\n");
			uint8_t buttonPressedData[5] = {0x41, 0x42, 0x43, 0x44, RELAY1_STATE};
			SendAnswer(buttonPressedData);
			// receive response
			receive_data();
			retries++;
		}
		lastRelay1State = currentState;
	}
}

void SendAnswer(uint8_t *data)
{
	resetNrf(); // clear all IRQs
	changeNrfToTX(); // change to TX mode
	transmit_data(data); // transmit data
	_delay_us(100); // some delay for safe transmission
	changeNrfToRX(); // change back to RX mode
}

// this interrupt will be triggered when received any data
// this handles all the answers that need to be sent
ISR(INT0_vect)
{
	// default answer
	uint8_t defaultAnswer[5] = {0x41, 0x42, 0x43, 0x44, 0x45}; // ABCDE
		
	cli(); // disable interrupt 
	clear_bit(PORTB, CE); // disable chip to stop listening
	
	// read data into global array
	data = ReadWriteNRF(R, R_RX_PAYLOAD, data, 5);
		
	// if message is meant for this atmega
	if (data[4] == 0x16)
	{
		Uart_Send_String("Received request\n");
		
		// if RPi is requesting for temperature and humidity
		if(data[3] == DHT11_REQUEST) SendDHT11Data();
		// if RPi successfully received emergency message
		else if(data[3] == EMERGENCY_ANSWER) successfullySend = 1;
		// if RPI is requesting to toggle relay1
		else if(data[3] == RELAY1_COMMAND) 
		{
			Uart_Send_String("RELAY1_COMMAND\n");
			ToggleLights();
		}
		// if command was not recognised it might be default so send default
		else SendAnswer(defaultAnswer);
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

void initRelays()
{
	set_bit(DDRA, RELAY1);	// output
	clear_bit(PORTA, RELAY1); // LOW
}

// this should send short signal to relay which will activate impulse relay
void ToggleLights()
{
	invert_bit(PORTA, RELAY1);
	_delay_ms(50);
	if(check_bit(PINA, RELAY1)) lastRelay1State = 1;
	else lastRelay1State = 0;
}

void initPIRSensor()
{
	clear_bit(DDRA, PIR_SENSOR); // input 
}

uint8_t CheckPIRSensor()
{
	if(check_bit(PINA, PIR_SENSOR)) return 1;
	else return 0;
}

