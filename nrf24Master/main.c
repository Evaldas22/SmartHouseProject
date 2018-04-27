/*
 * MUST COMPILE THIS PROGRAM WITH -lbcm2835 and -lsqlite3
 * gcc spi_test_BCM.c -o <output_file_name> -lbcm2835 -lsqlite3
*/
// ---------------------Includes-----------------------------
#include "nRF24L01.h"
#include "nRF24.h"
#include <wiringPi.h>
#include <bcm2835.h>
#include <sqlite3.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
// -----------------------DB stuff----------------------------------
typedef struct {
   uint8_t      id; // relay id should the same as in db
   char         name[50]; // relay + id
   uint8_t      command; // commands start from 0xf1
   uint8_t      state; // state depend on value from db
} Relay;

typedef struct {
  Relay *array;
  size_t used;
  size_t size;
} Array;

void initArray(Array *a, size_t initialSize) {
  a->array = (Relay *)malloc(initialSize * sizeof(Relay));
  a->used = 0;
  a->size = initialSize;
}

void insertArray(Array *a, Relay element) {
  if (a->used == a->size) {
    a->size *= 2;
    a->array = (Relay *)realloc(a->array, a->size * sizeof(Relay));
  }
  a->array[a->used++] = element;
}

void freeArray(Array *a) {
  free(a->array);
  a->array = NULL;
  a->used = a->size = 0;
}

volatile int diffFound = 0;
// ---------------------Defines-----------------------------
#define _delay_us(x) bcm2835_delayMicroseconds(x)
#define _delay_ms(x) bcm2835_delay(x)
#define set_bit(pin) bcm2835_gpio_write(pin, HIGH)
#define clear_bit(pin) bcm2835_gpio_write(pin, LOW)

// command defines
#define REGULAR 0
#define DHT11 1
#define STATE_UPDATE 2
#define RELAY1 3
#define ACK 4
#define FIND_STATE_COM 5

#define RELAY1_STATE 0xF1
#define STATE_UPDATE_ANSWER 0x66
#define DHT11_REQUEST 0x70
#define RELAY_TOGGLE 0x51
#define FIND_STATE 0x52
#define PIR_STATE 0x30

#define TIME_INTERVAL_SEC 60
#define MAX_RETRIES 10

#define ATMEGA16 0x20

// Some pins
#define CE RPI_GPIO_P1_22 // GPIO25
#define SS RPI_GPIO_P1_18 // GPIO24. I'll use this to manually control NRF Slave select pin
#define IRQ_PIN 29 // GPIO21
// ---------------------------------------------------------

//-------------------------functions------------------------
void ToggleRelay(uint8_t whichDevice, Relay whichRelay);
void FindRelayStatus(uint8_t whichDevice, Relay whichRelay);
void SendRequestTo(uint8_t *addr);
void receive_data(int command);
void ISR();
static int callbackDummy(void *data, int columns, char **argv, char **colNames);
static int compareTables(void *data, int columns, char **argv, char **colNames);
static int InitializeAllData(void *data, int columns, char **argv, char **colNames);
void CopyAllDataToTempDB(sqlite3* db, char *message, char *errMsg);
// ---------------------------------------------------------

volatile uint8_t sendSuccessfully = 0;
volatile uint8_t sendingRelayCommand = 0;
Array arr;

uint8_t relayState;
uint8_t temperature, humidity;

// db variables
const char* message = "Callback function called";
char *errMsg = 0;
char *sql;

// database object
sqlite3 *db;

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
	uint8_t atmega16ToggleRelay[5] = {0x41, 0x42, RELAY1_STATE, RELAY_TOGGLE, 0x16};
	uint8_t atmega16FindRelayStatus[5] = {0x41, 0x42, RELAY1_STATE, FIND_STATE, 0x16};

    // command for atmega328p
	uint8_t atmega328Address[5] = {0x11, 0x11, 0x11, 0x11, 0x32};

    // Initialize nrf as transmitter
	Nrf24_init(0, rxAddrFirstTX, txAddrFirstTX, 'T');

	// flush any IRQs from nrf before begining
	resetNrf();

	// variables to hold data about real time
	time_t myTime;

	time_t startTime;
	int firstTime = 1;

	initArray(&arr, 1);
	printf("Start. There are total %d relays\n", arr.used);

    // open database
    int status = sqlite3_open("/home/pi/things.db", &db);

    if(status)
    {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        return(0);
    }
    else fprintf(stderr, "Opened database successfully\n");

    sql = "SELECT * FROM things;";
    status = sqlite3_exec(db, sql, InitializeAllData, (void *) message, &errMsg);

    printf("After query. There are total %d relays\n", arr.used);

    // make sure db has real statuses
    for(int i = 0; i < arr.used; i++){
        FindRelayStatus(ATMEGA16, arr.array[i]);
        if(relayState != arr.array[i].state){
            // update state in db
            printf("Updating state in DB\n");
            char *value = (relayState) ? "ON" : "OFF";
            char temp[200];
            sprintf(temp, "UPDATE things SET value = '%s' where id = %d;", value, arr.array[i].id);
            sql = temp;
            status = sqlite3_exec(db, sql, callbackDummy, (void *) message, &errMsg);
        }
    }

    //after relays are synced make a copy of data
    CopyAllDataToTempDB(db, message, errMsg);

    while (1)
    {
        time(&myTime); // get current time
        // set first time startTime

        if (firstTime)
        {
            firstTime = 0;
            startTime = myTime;
            printf("\nTime when sent: %s\n", ctime(&myTime));
            SendRequestTo(atmega16DHT11);
            printf("Updating DB\n");

            char temp[200];
            sprintf(temp, "UPDATE things SET value = '%d' where name = 'Temperature';", temperature);
            sql = temp;
            status = sqlite3_exec(db, sql, callbackDummy, (void *) message, &errMsg);

            sprintf(temp, "UPDATE things SET value = '%d' where name = 'Humidity';", humidity);
            sql = temp;
            status = sqlite3_exec(db, sql, callbackDummy, (void *) message, &errMsg);

            // toggle relay2
            ToggleRelay(ATMEGA16, arr.array[0]);
        }

        // when set time interval is past do this

        if ((int)difftime(myTime, startTime) >= TIME_INTERVAL_SEC)
        {
            startTime = myTime;
            printf("\nTime when sent: %s\n", ctime(&myTime));

        }

        // while not sending any commands listen for any emergency messages
        changeNrfToRX();
        receive_data(STATE_UPDATE);
        changeNrfToTX();
    }
    bcm2835_close();
    sqlite3_close(db);
    return 0;
}

void ToggleRelay(uint8_t whichDevice, Relay whichRelay)
{
    if(whichDevice == ATMEGA16)
    {
       uint8_t commandToSend[5] = {0x41, 0x42, whichRelay.command, RELAY_TOGGLE, 0x16};
       printf("Senging command: %02x\n", whichRelay.command);
       SendRequestTo(commandToSend);

       // we need to update the db
        char *value = (relayState) ? "ON" : "OFF";
        char temp[200];
        sprintf(temp, "UPDATE things SET value = '%s' where id = %d;", value, whichRelay.id);
        sql = temp;
        int status = sqlite3_exec(db, sql, callbackDummy, (void *) message, &errMsg);
    }
}

void FindRelayStatus(uint8_t whichDevice, Relay whichRelay)
{
    if(whichDevice == ATMEGA16)
    {
       uint8_t commandToSend[5] = {0x41, 0x42, whichRelay.command, FIND_STATE, 0x16};
       printf("Senging command: %02x\n", whichRelay.command);
       SendRequestTo(commandToSend);
       // relay state will be updated in ISR function
    }
}

void SendRequestTo(uint8_t *addr)
{
    uint8_t retries = 0;
    sendSuccessfully = 0;

    // while no response is received keep sending, but no more than MAX_RETRIES
    while((sendSuccessfully != 1) && (retries < MAX_RETRIES))
    {
        if(addr[3] == RELAY_TOGGLE)
        {
            sendingRelayCommand = 1; // must set this before sending
        }

        printf("Trying to send!!\n");
        transmit_data(addr);
        //_delay_ms(5);

        changeNrfToRX();

        // depends on command addr what command option to pass
        if(addr[3] == 0x44) receive_data(REGULAR);
        else if(addr[3] == DHT11_REQUEST) receive_data(DHT11);
        else if(addr[3] == FIND_STATE) receive_data(FIND_STATE_COM);

        delay(1000);
        changeNrfToTX();
        retries++;
    }
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

                temperature = receivedData[3];
                humidity = receivedData[4];
            }
            // emergency button message arrived
            else if(command == STATE_UPDATE)
            {
                // now I need to find which relay was updated
                for(int i = 0; i < arr.used; i++)
                {
                    // if one of relay commands match
                    if(receivedData[4] == arr.array[i].command)
                    {
                        printf("%s was toggled.\n", arr.array[i].name);
                        if(receivedData[3])
                        {
                            printf("Now %s is ON.\n", arr.array[i].name);
                            relayState = 1;
                        }
                        else
                        {
                            printf("Now %s is OFF.\n", arr.array[i].name);
                            relayState = 0;
                        }
                        printf("Updating state in DB\n");
                        char *value = (relayState) ? "ON" : "OFF";
                        char temp[200];
                        sprintf(temp, "UPDATE things SET value = '%s' where id = %d;", value, arr.array[i].id);
                        sql = temp;
                        int status = sqlite3_exec(db, sql, callbackDummy, (void *) message, &errMsg);
                        // send response that state update message was received

                        changeNrfToTX();
                        uint8_t gotMessage[5] = {0x41, 0x42, 0x43, STATE_UPDATE_ANSWER, 0x16};
                        transmit_data(gotMessage);
                        changeNrfToRX();
                    }
                }

                if(receivedData[4] == PIR_STATE)
                {
                    printf("PIR motion sensor was triggered.\n");
                    changeNrfToTX();
                    uint8_t gotMessage[5] = {0x41, 0x42, 0x43, STATE_UPDATE_ANSWER, 0x16};
                    transmit_data(gotMessage);
                    changeNrfToRX();
                }
                // make sure this is 0
                sendSuccessfully = 0;
            }
            else if(command == FIND_STATE_COM)
            {
                if(receivedData[4])
                {
                    printf("Now relay is ON.\n");
                    relayState = 1;
                }
                else
                {
                    printf("Now relay is OFF.\n");
                    relayState = 0;
                }
            }
        }
        // if command is from atmega328p
        else if(receivedData[0] == 0x50)
        {
            printf("It's atmega328p responded: %s\n", receivedData);
        }

	}
	// else means there is no data received
	else sendSuccessfully = 0;

	resetNrf();
}

// this ISR will be launched when packet succesfully sent
void ISR()
{
    // if relay command is being sent
    if(sendingRelayCommand)
    {
        printf("Lights toggled!\n");
        sendSuccessfully = 1;
        // invert relay1State
        relayState = (relayState == 1) ? 0 : 1;
    }
}

void CopyAllDataToTempDB(sqlite3* db, char *message, char *errMsg)
{
    printf("Now copying data to temp db\n");

    // this would be executed after all command have been sent
    char *sql = "DELETE FROM thingsTemp; INSERT INTO thingsTemp SELECT * FROM things;";
    sqlite3_exec(db, sql, callbackDummy, (void *) message, &errMsg);
}

static int InitializeAllData(void *data, int columns, char **argv, char **colNames)
{
    // print query results
    for(int i = 0; i < columns; i++)
    {
        // if this is light group
        if(strcmp(argv[i], "lights") == 0)
        {
            printf("Lights found\n");
            Relay r;
            r.id = atoi (argv[0]);

            char relayName[50];
            sprintf(relayName, "relay%d", r.id);
            strcpy(r.name, relayName);

            r.command = RELAY1_STATE + arr.used;
            r.state = (strcmp(argv[3], "ON") == 0) ? 1 : 0;
            insertArray(&arr, r);
        }
    }

    printf("\n");
    return 0;
}

static int compareTables(void *data, int columns, char **argv, char **colNames)
{
    // mark tark difference was found
    diffFound = 1;

    // print query results
    for(int i = 0; i < columns; i++)
    {
        if(strcmp(argv[i], "lights") == 0)
        {
            // get the light ID
            uint8_t id = atoi(argv[0]);
            // go through all relays to find the one that need to be updated
            for(int i = 0; i < arr.used; i++){
                if(arr.array[i].id == id){
                    printf("%s need to updated\n", arr.array[i].name);
                    printf("Before: state %d\n", arr.array[i].state);
                    arr.array[i].state = (strcmp(argv[3], "ON") == 0) ? 1 : 0;
                    printf("After: state %d\n", arr.array[i].state);
                }
            }
        }
    }

    printf("\n");
    return 0;
}

static int callbackDummy(void *data, int columns, char **argv, char **colNames)
{
    return 0;
}
