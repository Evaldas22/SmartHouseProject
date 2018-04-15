/*
 * MUST COMPILE THIS PROGRAM WITH -lbcm2835
 * gcc spi_test_BCM.c -o <output_file_name> -lbcm2835
*/
// ---------------------Includes-----------------------------
#include "nRF24L01.h"
#include <wiringPi.h>
#include <bcm2835.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
// ---------------------------------------------------------

// ---------------------Defines-----------------------------
#define _delay_us(x) bcm2835_delayMicroseconds(x)
#define _delay_ms(x) bcm2835_delay(x)
#define set_bit(pin) bcm2835_gpio_write(pin, HIGH)
#define clear_bit(pin) bcm2835_gpio_write(pin, LOW)

// defines
#define W 1
#define R 0
// command defines
#define REGULAR 0
#define DHT11 1
#define EMERGENCY_COM 2
#define RELAY1 3
#define ACK 4

#define BUTTON1 0xF1
#define EMERGENCY_ANSWER 0x66
#define DHT11_REQUEST 0x70
#define RELAY1_TOGGLE 0x51

#define TIME_INTERVAL_SEC 10 // 1 minute
#define MAX_RETRIES 10

// Some pins
#define CE RPI_GPIO_P1_22 // GPIO25
#define SS RPI_GPIO_P1_18 // GPIO24. I'll use this to manually control NRF Slave select pin
#define IRQ_PIN 29 // GPIO21
// ---------------------------------------------------------

//-------------------------functions------------------------
void RequestFrom(uint8_t *addr);
uint8_t ReadRegister(uint8_t reg);
uint8_t * ReadWriteNRF(uint8_t R_W, uint8_t reg, uint8_t *data, uint8_t size);
void Nrf24_init(uint8_t pipe, uint8_t *addrRX, uint8_t *addrTX,  char mode);
void transmit_data(uint8_t *data);
void receive_data(int command);
void ISR();
void resetNrf(void);
void changeNrfToRX();
void changeNrfToTX();
// ---------------------------------------------------------

volatile uint8_t sendSuccessfully = 0;
volatile uint8_t sendingRelayCommand = 0;

int main()
{
	printf("Starting the program!!\n");

    if (!bcm2835_init()){
		printf("Exiting\n");
		return 1;
	}

    // setup external interrupt
	if(wiringPiSetup() < 0) return 1;
	if(wiringPiISR(IRQ_PIN, INT_EDGE_FALLING, &ISR) < 0)
    {
        printf("Unable to setup ISR \n");
    }

	// configure SPI
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128); // 3.125MHz
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

    // CE pin is for nrf24l01
    bcm2835_gpio_fsel(CE, BCM2835_GPIO_FSEL_OUTP);
    clear_bit(CE);

    // SS pin output
    bcm2835_gpio_fsel(SS, BCM2835_GPIO_FSEL_OUTP);
    set_bit(SS);

    // set addresses of nrf. Must match to use auto ACK
	uint8_t rxAddrFirstTX[5] = {0x12, 0x12, 0x12, 0x12, 0x12};
	uint8_t txAddrFirstTX[5] = {0x12, 0x12, 0x12, 0x12, 0x12};

	// commands for atmega16
	uint8_t atmega16Address[5] = {0x41, 0x42, 0x43, 0x44, 0x16};
	uint8_t atmega16DHT11[5] = {0x41, 0x42, 0x43, DHT11_REQUEST, 0x16};
	uint8_t atmega16ToggleRelay[5] = {0x41, 0x42, 0x43, RELAY1_TOGGLE, 0x16};

    // command for atmega328p
	uint8_t atmega328Address[5] = {0x11, 0x11, 0x11, 0x11, 0x32};

    // Initialize nrf as transmitter
	Nrf24_init(0, rxAddrFirstTX, txAddrFirstTX, 'T');
	printf("CONFIG register: %02x\n", ReadRegister(CONFIG));

	// flush any IRQs from nrf before begining
	resetNrf();

	// variables to hold data about real time
	time_t myTime;

	time_t startTime;
	int firstTime = 1;

    while (1)
    {
        time(&myTime); // get current time
        // set first time startTime
        if (firstTime)
        {
            firstTime = 0;
            startTime = myTime;
            printf("\nTime when sent: %s\n", ctime(&myTime));
            RequestFrom(atmega16ToggleRelay);
            _delay_ms(2000);
            //RequestFrom(atmega328Address);
            //_delay_ms(2000);
        }

        // when set time interval is past do this
        if ((int)difftime(myTime, startTime) >= TIME_INTERVAL_SEC)
        {
            startTime = myTime;
            printf("\nTime when sent: %s\n", ctime(&myTime));
            RequestFrom(atmega16ToggleRelay);
            _delay_ms(2000);
            //RequestFrom(atmega328Address);
            //_delay_ms(2000);
        }

        // while not sending any commands listen for any emergency messages
        changeNrfToRX();
        receive_data(EMERGENCY_COM);
        changeNrfToTX();
    }

    bcm2835_close();
    return 0;
}

void RequestFrom(uint8_t *addr)
{
    uint8_t retries = 0;
    sendSuccessfully = 0;

    // while no response is received keep sending, but no more than MAX_RETRIES
    while((sendSuccessfully != 1) && (retries < MAX_RETRIES))
    {
        printf("Trying to send!!\n");
        transmit_data(addr);
        //_delay_ms(5);

        changeNrfToRX();

        // depends on command addr what command option to pass
        if(addr[3] == 0x44) receive_data(REGULAR);
        else if(addr[3] == DHT11_REQUEST) receive_data(DHT11);
        else if(addr[3] == RELAY1_TOGGLE)
        {
            sendingRelayCommand = 1;
        }
        delay(1000);
        changeNrfToTX();
        retries++;
    }
}

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

// command tells this function what data to expect
void receive_data(int command)
{
    uint8_t dummy[1];
    ReadWriteNRF(W, FLUSH_RX, dummy, 0); // clear the RX buffer

	set_bit(CE); // enable for listening
	_delay_ms(1000); // listen for 1s
	clear_bit(CE); // stop listening

	// create empty array
	uint8_t *receivedData;

	// if RX_DR is set that means data is ready
	if(ReadRegister(STATUS) & (1 << RX_DR))
	{
        // mark that response is received
        sendSuccessfully = 1;

        printf("Data is ready.\n");

        // read data from buffer
        receivedData = ReadWriteNRF(R, R_RX_PAYLOAD, receivedData, 5);

        // if it's command from atmega16
        if(receivedData[0] == 0x41)
        {
            printf("It's atmega16 responded.\n");

            // regular respnse arrived
            if(command == REGULAR)
            {
                printf("Data: %s\n\n\n", receivedData);
            }
            // dht11 sensor response arrived
            else if(command == DHT11)
            {
                printf("Temperature: %d\n", receivedData[3]);
                printf("Humdity: %d\n\n\n", receivedData[4]);
            }
            // emergency button message arrived
            else if(command == EMERGENCY_COM)
            {
                // if button 1 was pressed
                if(receivedData[4] == BUTTON1)
                {
                    printf("Button 1 was pressed.Update Database\n");
                    // send response that emergency message was received1

                    changeNrfToTX();
                    uint8_t gotMessage[5] = {0x41, 0x42, 0x43, EMERGENCY_ANSWER, 0x16};
                    transmit_data(gotMessage);
                    changeNrfToRX();
                }
                // make sure this is 0
                sendSuccessfully = 0;
            }
        }
        // if command is from atmega328p
        else if(receivedData[0] == 0x50)
        {
            printf("It's atmega328p responded: %s\n", receivedData);
        }

	}
	// else means there is no data waiting
	else
	{
        sendSuccessfully = 0;
        //printf("No data.\n");
        /*
        if (command == RELAY1)
        {
            sendSuccessfully = 1; // pretend that data was sent. Not very good solution
            printf("Lights switched!\n");
        }
        // only send debug info when it's RPi request, not emergency message

        else if(command != BUTTON1)
        {
            receivedData = ReadWriteNRF(R, R_RX_PAYLOAD, receivedData, 5);
            printf("No data for you!!\n");
            sendSuccessfully = 0;
            printf("\n------DEBUG INFO------\n");
            printf("STATUS REGISTER: %02x\n", ReadRegister(STATUS));
            printf("DATA: %s\n", receivedData);
            printf("------DEBUG INFO END------\n");
        }
        */
	}

	resetNrf();
}

// this ISR will be launched when packet succesfully sent
void ISR()
{
    printf("\nSent succesfully!!\n");
    // if relay command is being sent
    if(sendingRelayCommand)
    {
        printf("Lights toggled!\n");
        sendSuccessfully = 1;
    }
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
