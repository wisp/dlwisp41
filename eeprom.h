/*
 * File:   eeprom.h
 * Author: ssclark
 */

#ifndef EEPROM_H
#define EEPROM_H

#include <msp430x21x2.h>
#include "pinDefWISP4.1DL.h"

void init_eeprom();
unsigned char write_eeprom(int address, unsigned char *data, int length);
unsigned char read_eeprom(int address, unsigned char *data, int length);

// Uses a timed LPM3 delay to wait for the EEPROM chip to complete operations.
void delay_cycles(unsigned int num_of_cycles);

#endif // EEPROM_H

