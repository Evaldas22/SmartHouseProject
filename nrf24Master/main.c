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
// -----------------------Data structures----------------------------------
typedef struct
{
   uint8_t      id; // relay id should the same as in db
   char         name[50]; // relay + id
   uint8_t      command; // commands start from 0xf1
   uint8_t      state; // state depend on value from db
   int          device; // device number is same as in db
} Relay;

typedef struct
{
  Relay *array;
  size_t used;
  size_t size;
} RelayArray;

void initRelayArray(RelayArray *a, size_t initialSize)
{
  a->array = (Relay *)malloc(initialSize * sizeof(Relay));
  a->used = 0;
  a->size = initialSize;
}

void insertRelayArray(RelayArray *a, Relay element)
{
  if (a->used == a->size)
  {
    a->size *= 2;
    a->array = (Relay *)realloc(a->array, a->size * sizeof(Relay));
  }
  a->array[a->used++] = element;
}

void freeRelayArray(RelayArray *a) {
  free(a->array);
  a->array = NULL;
  a->used = a->size = 0;
}
// ---------------------Define macros-----------------------------
#define _delay_us(x) bcm2835_delayMicroseconds(x)
#define _delay_ms(x) bcm2835_delay(x)
#define set_bit(pin) bcm2835_gpio_write(pin, HIGH)
#define clear_bit(pin) bcm2835_gpio_write(pin, LOW)

// STATUSES
#define ON 1
#define OFF 0
#define NOT_FOUND 2

// command defines
#define REGULAR 0
#define DHT 1
#define STATE_UPDATE 2
#define RELAY1 3
#define ACK 4
#define FIND_STATE_COM 5

#define RELAY1_STATE 0xF1
#define STATE_UPDATE_ANSWER 0x66
#define DHT_REQUEST_COM 0x70
#define RELAY_TOGGLE 0x51
#define FIND_STATE 0x52
#define PIR_STATE 0x30
#define ERROR 0x05
#define TURN_OFF_PIR 0xA1
#define TURN_ON_PIR 0xA0

#define TIME_INTERVAL_SEC 60
#define MAX_RETRIES 10

#define ATMEGA16 16
#define ATMEGA16_2 17
// Some pins
#define CE RPI_GPIO_P1_22 // GPIO25
#define SS RPI_GPIO_P1_18 // GPIO24. I'll use this to manually control NRF Slave select pin
#define IRQ_PIN 28 // GPIO20
// ---------------------------------------------------------

//-------------------------functions------------------------
void CheckIfDeviceIsOn(uint8_t whichDevice);
void ToggleRelay(uint8_t whichDevice, Relay whichRelay, int state);
void FindRelayStatus(uint8_t whichDevice, Relay whichRelay);
void SendRequestTo(uint8_t *addr);
void Receive_data(int command);
void ISR();
static int CallbackDummy(void *data, int columns, char **argv, char **colNames);
static int CompareTables(void *data, int columns, char **argv, char **colNames);
static int InitializeAllData(void *data, int columns, char **argv, char **colNames);
void CopyAllDataToTempDB();
void Setup();
int ConvertIntToHex(uint8_t integer);
// ---------------------------------------------------------

volatile uint8_t sendSuccessfully = 0;
volatile uint8_t sendingRelayCommand = 0;
RelayArray arr, notFoundRelays;

uint8_t relayState;
uint8_t ATMEGA16On = 0, ATMEGA16_2On = 0;
char temperature[7];
char humidity[7];

// db variables
const char* message = "Callback function called";
char *errMsg = 0;
char *sql;

// database object
sqlite3 *db;

int main()
{
	printf("Starting the program!!\n");

	Setup();

	// commands for atmega16
	uint8_t atmega16DHT11[5] = {0x41, 0x42, 0x43, DHT_REQUEST_COM, 0x16};
	uint8_t atmega16_2DHT11[5] = {0x41, 0x42, 0x43, DHT_REQUEST_COM, 0x17};

	// variables to hold data about real time
	time_t myTime;

	time_t startTime;
	int firstTime = 1;

    // open database
    int status = sqlite3_open("/home/pi/Desktop/Andriaus/SmartHome-Project/public/db/IoT.db", &db);

    //sqlite3_exec(db, "PRAGMA journal_mode = WAL", NULL, NULL, NULL);

    if(status)
    {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        return(0);
    }
    else printf("Opened database successfully\n");

    sql = "SELECT * FROM things;";
    status = sqlite3_exec(db, sql, InitializeAllData, (void *) message, &errMsg);

    printf("After query. There are total %d relays\n", arr.used);

    CheckIfDeviceIsOn(ATMEGA16);
    if(ATMEGA16On) printf("ATMEGA16 is ON\n");
    else printf("ATMEGA16 is OFF\n");

    //CheckIfDeviceIsOn(ATMEGA16_2);
    if(ATMEGA16_2On) printf("ATMEGA16_2 is ON\n");
    else printf("ATMEGA16_2 is OFF\n");

    // make sure db has real statuses
    for(int i = 0; i < arr.used; i++){
        relayState = NOT_FOUND;

        // find relay status of certain device
        if(arr.array[i].device == ATMEGA16 && ATMEGA16On)
        {
            FindRelayStatus(arr.array[i].device, arr.array[i]);
        }
        else if(arr.array[i].device == ATMEGA16_2 && ATMEGA16_2On)
        {
            FindRelayStatus(arr.array[i].device, arr.array[i]);
        }

        if(relayState == NOT_FOUND){
            // just add not found relay to array
            insertRelayArray(&notFoundRelays, arr.array[i]);
        }
        // if received relay state doesnt match with the one in db - update db
        else if(relayState != arr.array[i].state){
            // update state in db
            printf("Updating state in DB\n");
            char *value = (relayState) ? "ON" : "OFF";
            char temp[200];
            sprintf(temp, "UPDATE things SET value = '%s' WHERE id = %d;", value, arr.array[i].id);
            sql = temp;
            status = sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);
        }
    }

    // now it's time to send one UPDATE query for all not found relays
    char temp[200];
    sprintf(temp, "UPDATE things SET value = 'Not responded' WHERE id IN (");
    sql = temp;
    // go through all not found relays and finish constructing query
    for(int i = 0; i < notFoundRelays.used; i++){
        char temp1[10];
        // if this is the last item
        if(i == notFoundRelays.used - 1){
            sprintf(temp1, "%d);", notFoundRelays.array[i].id);
            strcat(sql, temp1);
        }
        else {
            sprintf(temp1, "%d, ", notFoundRelays.array[i].id);
            strcat(sql, temp1);
        }
    }
    // now exucute the UPDATE query
    status = sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);

    // after relays are synced make a copy of data
    CopyAllDataToTempDB();

    // send command to turn of PIR sensor
    printf("Turning OFF PIR sensor\n");
    uint8_t command[5] = {0x41, 0x42, 0x43, TURN_OFF_PIR, 0x16};
    SendRequestTo(command);

    // start server
    const char *cmd = "gnome-terminal --command=\"./runServer\" ";
    system(cmd);

    while (1)
    {
        time(&myTime); // get current time

        if (firstTime)
        {
            firstTime = 0;
            startTime = myTime;
            printf("\nTime when sent: %s\n", ctime(&myTime));
            SendRequestTo(atmega16DHT11);
            //SendRequestTo(atmega16_2DHT11);
            printf("Updating DB\n");

            char temp[200];
            sprintf(temp, "UPDATE things SET value = '%s' where name = 'Temperature';", temperature);
            sql = temp;
            status = sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);

            sprintf(temp, "UPDATE things SET value = '%s' where name = 'Humidity';", humidity);
            sql = temp;
            status = sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);

            //CopyAllDataToTempDB();
        }

        // when set time interval is past do this
        if ((int)difftime(myTime, startTime) >= TIME_INTERVAL_SEC)
        {
            startTime = myTime;
            printf("\nTime: %s\n", ctime(&myTime));
            SendRequestTo(atmega16DHT11);
            //SendRequestTo(atmega16_2DHT11);
            printf("Updating DB\n");

            char temp[200];
            sprintf(temp, "UPDATE things SET value = '%s' where name = 'Temperature';", temperature);
            sql = temp;
            status = sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);

            sprintf(temp, "UPDATE things SET value = '%s' where name = 'Humidity';", humidity);
            sql = temp;
            status = sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);

            //CopyAllDataToTempDB();
        }

        //--------------COMPARE TWO TABLES------------------------
        printf("Now comparing two tables\n");
        sql = "SELECT id, group_name, value, device FROM things EXCEPT SELECT id, group_name, value, device FROM thingsTemp;";
        status = sqlite3_exec(db, sql, CompareTables, (void *) message, &errMsg);

        // while not sending any commands listen for any state messages
        changeNrfToRX();
        Receive_data(STATE_UPDATE);
        changeNrfToTX();
    }

    bcm2835_close();
    sqlite3_close(db);
    return 0;
}

void CheckIfDeviceIsOn(uint8_t whichDevice)
{
    uint8_t device = ConvertIntToHex(whichDevice);

    uint8_t address[5] = {0x41, 0x42, 0x43, 0x44, device};

    SendRequestTo(address);
}

void ToggleRelay(uint8_t whichDevice, Relay whichRelay, int state)
{
    uint8_t device = ConvertIntToHex(whichDevice);
    uint8_t commandToSend[5] = {0x41, 0x42, whichRelay.command, RELAY_TOGGLE, device};

    int retries = 0;
    do{
        SendRequestTo(commandToSend);
        FindRelayStatus(whichDevice, whichRelay);

        //printf("State should be: %d and is right now: %d\n", state, relayState);
        retries++;
    } while((state != relayState) && retries < MAX_RETRIES);

    // we need to update the db
    char *value = (relayState == 1) ? "ON" : "OFF";
    char temp[200];
    sprintf(temp, "UPDATE things SET value = '%s' where id = %d; DELETE FROM thingsTemp; INSERT INTO thingsTemp SELECT * FROM things;", value, whichRelay.id);
    sql = temp;
    sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);
}

void FindRelayStatus(uint8_t whichDevice, Relay whichRelay)
{
    uint8_t device = ConvertIntToHex(whichDevice);

    uint8_t commandToSend[5] = {0x41, 0x42, whichRelay.command, FIND_STATE, device};
    SendRequestTo(commandToSend);
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

        printf("Retries: %d\n", retries);
        transmit_data(addr);
        //_delay_ms(5);

        changeNrfToRX();
        // depends on command addr what command option to pass
        if(addr[3] == 0x44) Receive_data(REGULAR);
        else if(addr[3] == DHT_REQUEST_COM) Receive_data(DHT);
        else if(addr[3] == FIND_STATE) Receive_data(FIND_STATE_COM);
        else if(addr[3] == TURN_OFF_PIR || addr[3] == TURN_ON_PIR ) Receive_data(REGULAR);

        delay(200);
        changeNrfToTX();
        retries++;
    }
}

// command tells this function what data to expect
void Receive_data(int command)
{
    uint8_t dummy[1];
    ReadWriteNRF(W, FLUSH_RX, dummy, 0); // clear the RX buffer

	set_bit(CE); // enable for listening
	_delay_ms(700); // listen for 1s
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

                ATMEGA16On = 1;
            }
            // dht11 sensor response arrived
            else if(command == DHT)
            {
                printf("Temperature: %d\n", receivedData[3]);
                printf("Humdity: %d\n\n\n", receivedData[4]);

                sprintf(temperature, "%d", receivedData[3]);
                sprintf(humidity, "%d", receivedData[4]);
            }
            // lights state update from sensor box
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

                        sendingRelayCommand = 0;
                        // send response that state update message was received
                        changeNrfToTX();
                        uint8_t gotMessage[5] = {0x41, 0x42, 0x43, STATE_UPDATE_ANSWER, 0x16};
                        printf("Sending answer back...\n");
                        transmit_data(gotMessage);
                        changeNrfToRX();
                        _delay_ms(100);

                        printf("Updating state in DB\n");
                        char *value = (relayState) ? "ON" : "OFF";
                        char temp[200];
                        sprintf(temp, "UPDATE things SET value = '%s' where id = %d;", value, arr.array[i].id);
                        sql = temp;
                        sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);

                        CopyAllDataToTempDB();
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
                if (receivedData[4] == ERROR)
                {
                    printf("Relay is not available\n");
                    relayState = 2;
                }
                else if(receivedData[4])
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
            else printf("Command not recognised\n");
        }
        // if command is from atmega16_2
        else if(receivedData[0] == 0x31)
        {
            printf("It's atmega16_2 responded.\n");

            // regular respnse arrived
            if(command == REGULAR)
            {
                ATMEGA16_2On = 1;
            }
            // dht11 sensor response arrived
            else if(command == DHT)
            {
                printf("Temperature: %d.%d\n", receivedData[1], receivedData[2]);
                printf("Humdity: %d.%d\n\n\n", receivedData[3], receivedData[4]);

                sprintf(temperature, "%d", receivedData[1]);
                sprintf(humidity, "%d", receivedData[3]);
            }
        }

	}
	// else means there is no data received
	else{
        sendSuccessfully = 0;
        relayState = 2;
    }

	resetNrf();
}

// this ISR will be launched when packet succesfully sent
void ISR()
{
    // if relay command is being sent
    if(sendingRelayCommand)
    {
        sendSuccessfully = 1;
        // invert relayState
        relayState = (relayState == 1) ? 0 : 1;
    }
}

void CopyAllDataToTempDB()
{
    //printf("Now copying data to temp db\n");
    // this would be executed after all command have been sent
    char *sql = "DELETE FROM thingsTemp; INSERT INTO thingsTemp SELECT * FROM things;";
    sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);
}

static int InitializeAllData(void *data, int columns, char **argv, char **colNames)
{
    // print query results
    for(int i = 0; i < columns; i++)
    {
        // if this is light group
        if(strcmp(argv[i], "lights") == 0)
        {
            Relay r;
            r.id = atoi (argv[0]);

            char relayName[50];
            sprintf(relayName, "relay%d", r.id);
            strcpy(r.name, relayName);

            r.command = RELAY1_STATE + arr.used;
            r.state = (strcmp(argv[3], "ON") == 0) ? 1 : 0;
            r.device = atoi (argv[4]); // must be number
            insertRelayArray(&arr, r);
        }
    }
    return 0;
}

// SELECT id, group_name, value, device
// this function get called on each matching result i.e. each row
static int CompareTables(void *data, int columns, char **argv, char **colNames)
{
    // print query results
    for(int i = 0; i < columns; i++)
    {
        if(strcmp(argv[i], "lights") == 0)
        {
            // get the light ID
            uint8_t id = atoi(argv[0]);
            // go through all relays to find the one that need to be updated
            for(int i = 0; i < arr.used; i++)
            {
                if(arr.array[i].id == id)
                {
                    printf("%s\n", argv[2]);
                    if(strcmp(argv[2], "Not responded") == 0) continue;
                    int stateThatShouldBe = (strcmp(argv[2], "ON") == 0) ? 1 : 0;
                    printf("State should be: %d\n", stateThatShouldBe);
                    ToggleRelay(arr.array[i].device, arr.array[i], stateThatShouldBe);
                    arr.array[i].state = stateThatShouldBe;
                    break;
                }
                if(i == arr.used-1)
                {
                    printf("\nAdding new relay!!\n");
                    Relay r;
                    r.id = id;

                    char relayName[50];
                    sprintf(relayName, "relay%d", r.id);
                    strcpy(r.name, relayName);

                    r.command = RELAY1_STATE + arr.used;
                    r.state = (strcmp(argv[2], "ON") == 0) ? 1 : 0;
                    r.device = atoi (argv[3]); // must be number
                    insertRelayArray(&arr, r);
                    printf("\nNew relay created!!\n");
                    printf("Now checking it's state\n");
                    FindRelayStatus(arr.array[i+1].device, arr.array[i+1]);

                    if(relayState == NOT_FOUND){
                        // just add not found relay to array
                        insertRelayArray(&notFoundRelays, arr.array[i+1]);
                        char temp[200];
                        sprintf(temp, "UPDATE things SET value = 'Not responded' WHERE id=%d;", arr.array[i+1].id);
                        printf("%s\n", temp);
                        sql = temp;
                        //int status = sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);
                        sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);
                        argv[2] = "Not responded";
                    }
                    // if received relay state doesnt match with the one in db - update db
                    else if(relayState != arr.array[i+1].state){
                        // update state in db
                        printf("Updating state in DB\n");
                        char *value = (relayState) ? "ON" : "OFF";
                        char temp[200];
                        sprintf(temp, "UPDATE things SET value = '%s' WHERE id = %d;", value, arr.array[i+1].id);
                        sql = temp;
                        //int status = sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);
                        sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);
                    }

                    CopyAllDataToTempDB();
                }
            }
        }
        else if(strcmp(argv[i], "sensors") == 0)
        {
            if(strcmp(argv[3], "16") == 0)
            {
                if(strcmp(argv[2], "ON") == 0)
                {
                    printf("PIR GOES ON\n");
                    uint8_t command[5] = {0x41, 0x42, 0x43, TURN_ON_PIR, 0x16};
                    SendRequestTo(command);

                    if(sendSuccessfully)
                    {
                        printf("Updating state in DB\n");
                        char temp[200];
                        sprintf(temp, "UPDATE things SET value = 'ON' where id = %d;", atoi(argv[0]));
                        sql = temp;
                        sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);

                        CopyAllDataToTempDB();
                    }
                }

                else if(strcmp(argv[2], "OFF") == 0)
                {
                    printf("PIR GOES OFF\n");
                    uint8_t command[5] = {0x41, 0x42, 0x43, TURN_OFF_PIR, 0x16};
                    SendRequestTo(command);

                    if(sendSuccessfully)
                    {
                        printf("Updating state in DB\n");
                        char temp[200];
                        sprintf(temp, "UPDATE things SET value = 'OFF' where id = %d;", atoi(argv[0]));
                        sql = temp;
                        sqlite3_exec(db, sql, CallbackDummy, (void *) message, &errMsg);

                        CopyAllDataToTempDB();
                    }
                }
            }
        }
    }
    return 0;
}

static int CallbackDummy(void *data, int columns, char **argv, char **colNames)
{
    return 0;
}

void Setup()
{
    if (!bcm2835_init()){
		printf("Exiting\n");
		exit(1);
	}

    // setup external interrupt
	if(wiringPiSetup() < 0) exit(1);
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

	// Initialize nrf as transmitter
	Nrf24_init(0, rxAddrFirstTX, txAddrFirstTX, 'T');

	// flush any IRQs from nrf before begining
	resetNrf();

	initRelayArray(&arr, 1);
	initRelayArray(&notFoundRelays, 1);
}

int ConvertIntToHex(uint8_t integer)
{
    char hex[3];
    sprintf(hex, "%d", integer);

    int newNum = (int)strtol(hex, NULL, 16);
    return newNum;
}
