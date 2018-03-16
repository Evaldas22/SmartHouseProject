#ifndef ExternalInterrupts
#define ExternalInterrupts

#include <avr/interrupt.h>

//-----------METHODS-----------------------------------------------------

void EnableExternalInterrupt0(void);
void DisableExternalInterrupt0(void);

//-------------------------------------------------------------------------

void EnableExternalInterrupt0(void)
{
	DDRD &= ~(1 << 2); // external interrupt(input)
	// external pull-down resistor (10K) is needed for rising edge interrupt
	EICRA |= 1 << ISC01; // INT0 to be triggered at falling edge
	EIMSK |= (1 << INT0); // enable INT0 external interrupt 
	sei(); // enable global interrupts
}

void DisableExternalInterrupt0(void)
{
	EIMSK &= ~(1 << INT0); // disable external interupt 0
}


#endif