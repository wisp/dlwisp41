

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

// these bit definitions are specific to WISP 4.1 DL

#define SENSOR_DATA_TYPE_ID       0x0B

#define ACCEL_ENABLE_BIT          BIT5   // 1.5
#define SET_ACCEL_ENABLE_DIR      P1DIR |= ACCEL_ENABLE_BIT
#define CLEAR_ACCEL_ENABLE_DIR    P1DIR &= ~ACCEL_ENABLE_BIT
#define TURN_ON_ACCEL_ENABLE      P1OUT |= ACCEL_ENABLE_BIT
#define TURN_OFF_ACCEL_ENABLE     P1OUT &= ~ACCEL_ENABLE_BIT

#define DATA_LENGTH_IN_WORDS      3
#define DATA_LENGTH_IN_BYTES      (DATA_LENGTH_IN_WORDS*2)

unsigned char sensor_busy = 0;

void init_sensor()
{
  return;
}

void read_sensor(unsigned char volatile *target) 
{
  
  // turn off comparator
  P1OUT &= ~RX_EN_PIN;
  
  // slow down clock
  BCSCTL1 = XT2OFF + RSEL1; // select internal resistor (still has effect when DCOR=1)
  DCOCTL = DCO1+DCO0; // set DCO step. 
  
  if(!is_power_good())
    sleep();
  
  // Clear out any lingering voltage on the accelerometer outputs
  ADC10AE0 = 0;
  
#if(WISP_VERSION == BLUE_WISP || WISP_VERSION == PURPLE_WISP)
  P2OUT &= ~(ACCEL_X | ACCEL_Y | ACCEL_Z);
  P2DIR |=   ACCEL_X | ACCEL_Y | ACCEL_Z;
  P2DIR &= ~(ACCEL_X | ACCEL_Y | ACCEL_Z);
#elif(WISP_VERSION == RED_WISP)
  P1OUT &= ~(ACCEL_X | ACCEL_Y | ACCEL_Z);
  P1DIR |=   ACCEL_X | ACCEL_Y | ACCEL_Z;
  P1DIR &= ~(ACCEL_X | ACCEL_Y | ACCEL_Z);
#endif
  
  
  P1DIR |= ACCEL_POWER;
  P1OUT |= ACCEL_POWER;
  ADC10AE0 |= ACCEL_X | ACCEL_Y | ACCEL_Z;
  
  // a little time for regulator to stabilize active mode current AND
  // filter caps to settle.
  for(int i = 0; i < 225; i++);
  RECEIVE_CLOCK;
    
  // GRAB DATA
  ADC10CTL0 &= ~ENC; // make sure this is off otherwise settings are locked.
  ADC10CTL0 = SREF_0 + ADC10SHT_1 + ADC10ON;
  ADC10CTL1 = ADC10DIV_2 + ADC10SSEL_0 + SHS_0 + CONSEQ_0 + INCH_ACCEL_X;
  ADC10CTL0 |= ENC;
  ADC10CTL0 |= ADC10SC;
  
  while (ADC10CTL1 & ADC10BUSY);    // wait while ADC finished work
  
  ackReply[4] = (ADC10MEM & 0xff);
  // grab msb bits and store it
  ackReply[3] = (ADC10MEM & 0x0300) >> 8;
  
  // GRAB DATA
  ADC10CTL0 &= ~ENC; // make sure this is off otherwise settings are locked.
  ADC10CTL0 = SREF_0 + ADC10SHT_1 + ADC10ON;
  ADC10CTL1 = ADC10DIV_2 + ADC10SSEL_0 + SHS_0 + CONSEQ_0 + INCH_ACCEL_Y;
  ADC10CTL0 |= ENC;
  ADC10CTL0 |= ADC10SC;
  
  while (ADC10CTL1 & ADC10BUSY);    // wait while ADC finished work
  
  ackReply[6] = (ADC10MEM & 0xff);
  // grab msb bits and store it
  ackReply[5] = (ADC10MEM & 0x0300) >> 8;
  
  // GRAB DATA
  ADC10CTL0 &= ~ENC; // make sure this is off otherwise settings are locked.
  ADC10CTL0 = SREF_0 + ADC10SHT_1 + ADC10ON;
  ADC10CTL1 = ADC10DIV_2 + ADC10SSEL_0 + SHS_0 + CONSEQ_0 + INCH_ACCEL_Z;
  ADC10CTL0 |= ENC;
  ADC10CTL0 |= ADC10SC;
  
  while (ADC10CTL1 & ADC10BUSY);    // wait while ADC finished work
  
  ackReply[8] = (ADC10MEM & 0xff);
  // grab msb bits and store it
  ackReply[7] = (ADC10MEM & 0x0300) >> 8;
  
  // Power off sensor and adc
  P1DIR &= ~ACCEL_POWER;
  P1OUT &= ~ACCEL_POWER;
  ADC10CTL0 &= ~ENC;
  ADC10CTL1 = 0;       // turn adc off
  ADC10CTL0 = 0;       // turn adc off
  
  // Store sensor read count
  sensor_counter++;
  ackReply[10] = (sensor_counter & 0x00ff);
  // grab msb bits and store it
  ackReply[9]  = (sensor_counter & 0xff00) >> 8;

  // turn on comparator
  P1OUT |= RX_EN_PIN;
}
