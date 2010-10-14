#ifndef DLWISP41_H
#define DLWISP41_H

/*
Copyright (c) 2009, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following
conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of Intel Corporation nor the names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Pin definitions
// WISP 4.1 DL
// "Blue WISP"

// MSP430F2132
#include <msp430x21x2.h>
#define USE_2132  1

// See wisp.wikispaces.com for a schematic.

// Port 1
#define TEMP_POWER     BIT0       // output
#define TX_PIN         BIT1       // output
#define RX_PIN         BIT2       // input
#define RX_EN_PIN      BIT3       // output
#define DEBUG_1_4      BIT4       // output unless externally driven
#define ACCEL_POWER    BIT5       // output
#define LED_POWER      BIT6       // output
#define CAP_SENSE      BIT7       // output/input

// Port 2
#define ACCEL_Z        BIT0       // input
#define ACCEL_Y        BIT1       // input
#define ACCEL_X        BIT2       // input
#define DEBUG_2_3      BIT3       // output unless externally driven
#define VOLTAGE_SV_PIN BIT4       // input
#define DEBUG_2_5      BIT5       // connect to SV_IN by 0 ohm
#define CRYSTAL_IN     BIT6       // input
#define CRYSTAL_OUT    BIT7       // output

// Port 3
#define CLK_A          BIT0       // output unless externally driven
#define SDA_B          BIT1       // input (connected to 10k pullup res)
#define SCL_B          BIT2       // input (connected to 10k pullup res)
#define VSENSE_POWER   BIT3       // output
#define TX_A           BIT4       // output unless externally driven
#define RX_A           BIT5       // output unless externally driven
#define VSENSE_IN      BIT6       // input
#define TEMP_EXT_IN    BIT7       // input

// Analog Inputs (ADC In Channel)
#define INCH_ACCEL_Z     INCH_0
#define INCH_ACCEL_Y     INCH_1
#define INCH_ACCEL_X     INCH_2
#define INCH_DEBUG_2_3   INCH_3
#define INCH_VSENSE_IN   INCH_6
#define INCH_TEMP_EXT_IN INCH_7

//#define INCH_2_4 INCH_4   // not accessible
// #define INCH_3_5 INCH_5  // ??

#define DRIVE_ALL_PINS  \
  P1OUT = 0;  \
  P2OUT = 0;  \
  P3OUT = 0;  \
  P1DIR = TEMP_POWER | TX_PIN | RX_EN_PIN | DEBUG_1_4 | LED_POWER | CAP_SENSE; \
  P2DIR = DEBUG_2_3 | CRYSTAL_OUT; \
  P3DIR = CLK_A | VSENSE_POWER | TX_A | RX_A;

#if DEBUG_PINS_ENABLED
#define DEBUG_PIN5_HIGH               P3OUT |= BIT5;
#define DEBUG_PIN5_LOW                P3OUT &= ~BIT5;
#else
#define DEBUG_PIN5_HIGH
#define DEBUG_PIN5_LOW
#endif

#endif // DLWISP41_H