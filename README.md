# SmartHouseProject
University bachelor final project

nRF24L01Atmega1 - acts as a master. It  sends commands to either Atmega3(328p) or Atmega2(16) and then receives data from them.
nRF24L01Atmega2 - acts as a slave. It receives command from master and responds to it.
nRF24L01Atmega3 - acts as a slave. Does the same as atmega2.

For now Atmega1 only send one command which consists of 5 bytes. Last byte indicates slave address e.g. [0x11, 0x11, 0x11, 0x11, 0x16\0x32].
All slaves will receive command and if that command is addressed to it, it will act up on it. It will simply send some data (5 bytes) back.

Atmega2 will respond with 'ABCDE'
Atmega3 will respond with 'PQRST'
