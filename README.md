# SmartHouseProject
University bachelor final project

nRF24L01Atmega1 - acts as a master. It  sends commands to either Atmega3(328p) or Atmega2(16) and then receives data from them.
nRF24L01Atmega2 - acts as a slave. It receives command from master and responds to it.
nRF24L01Atmega3 - acts as a slave. Does the same as atmega2.

For now Atmega1 only send one command which consists of 5 bytes. Last byte indicates slave address e.g. [0x11, 0x11, 0x11, 0x11, 0x16\0x32].
All slaves will receive command and if that command is addressed to it, it will act up on it. It will simply send some data (5 bytes) back.

Atmega2 will respond with 'ABCDE'
Atmega3 will respond with 'PQRST'

------------------------------------------------------------------------------------------------------------------------------------------------
EDIT1:
Both atmegas respond to request from Raspberry Pi 3.
Atmega2 receives two types of commands - regular and dht11 sensor. To regular it responds with 'ABCDE', to dht11 sensor it read temperature and
humidity from dht11 sensor and sends it back

All functions that was done by atmega1 is now done by RPi 3.

Sometimes RPi doesn't receive packet from either one of two atmegas.

------------------------------------------------------------------------------------------------------------------------------------------------
EDIT2:
Atmega2 can now check the status of button attached to it. In the while loop it does the following: listen for 2s for commands from RPi, checks button state.
When button state is sent, it waits for response from RPi. If correct response is not received it'll send 9 more times.

------------------------------------------------------------------------------------------------------------------------------------------------
EDIT3:
Added RPi code. Solved RPi problem (not receiving packet from atmega sometimes) by placing code that send request and wait for response ir while 
loop that sends request (maximum 10 times) until it receives answer.
Also between commands, rpi listens for emergency messages (like button press on atmega) 

------------------------------------------------------------------------------------------------------------------------------------------------
EDIT4:
Added PIR motion sensor and relay on atmega2. RPi can now send data to toggle relay. Finally managed to get external interrupts working and can now monitor
whether packet was successfully sent to atmega or not. Using ISR (interrupt service routine) I no longer have to use complex algorithm to make sure relay was toggled.
If command successfully was sent to atmega, I know that it'll be toggled. Later will add functionality to send back relay status to be 100% sure.

------------------------------------------------------------------------------------------------------------------------------------------------
EDIT5:
Added functionality to control relay 1 with light switch and also when received command from RPi. Now when light switch changes position atmega2
recognises it, toggle relay and also send status to RPi. RPi has commands to toggle relay 1 and also check its state. When using light switch a medium delay
of about 2 seconds can be seen, because atmega listens for 2 seconds for commands and only then checks the state of ligth switch.