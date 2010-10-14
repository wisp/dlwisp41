#ifndef RFID_H
#define RFID_H

volatile short state;
volatile unsigned char command;
unsigned short rn16;
unsigned int epc;
unsigned short divideRatio;
unsigned short linkFrequency;
unsigned char subcarrierNum;
unsigned char TRext;
unsigned char delimiterNotFound;
unsigned short ackReplyCRC, queryReplyCRC, readReplyCRC;
unsigned short Q = 0, slot_counter = 0, shift = 0;
unsigned int read_counter = 0;
unsigned int sensor_counter = 0;
unsigned char timeToSample = 0;

unsigned short inInventoryRound = 0;
unsigned char last_handle_b0, last_handle_b1;

#endif // RFID_H
