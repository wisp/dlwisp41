
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

#define SENSOR_DATA_TYPE_ID       0x0F

unsigned char sensor_busy = 0;

void init_sensor()
{
  return;
}

void read_sensor(unsigned char volatile *target) 
{
     
       // slow down clock
        BCSCTL1 = XT2OFF + RSEL1; // select internal resistor (still has effect when DCOR=1)
        DCOCTL = DCO1+DCO0; // set DCO step. 
            
        if(!is_power_good())
          sleep();
  
        // already off. Only needs to be done when READ has set
        P1OUT &= ~BIT_IN_ENABLE;   // turn off comparator
        
        // Set up ADC for internal temperature sensor  
        ADC10CTL0 &= ~ENC; // make sure this is off otherwise settings are locked.
        ADC10CTL1 = INCH_10 + ADC10DIV_3;         // Temp Sensor ADC10CLK/4
        ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
        
        // a little time for regulator to stabilize active mode current AND
        // filter caps to settle.
        for (int i = 0; i < 50; i++);
        
        // start conversion
        unsigned int k = 0;
        ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
        sensor_busy++; // prevent race condition - needed for all sampling with lpm4s
        LPM4;       // go to sleep
        sensor_busy = 0;
        
        *(target + k + 1 ) = (ADC10MEM & 0xff);
        // grab msb bits and store it
        *(target + k) = (ADC10MEM & 0x0300) >> 8;
        
        // Power off sensor and adc
        ADC10CTL0 &= ~ENC;
        ADC10CTL1 = 0;       // turn adc off
        ADC10CTL0 = 0;       // turn adc off
        
        return;
}

unsigned char is_sensor_sampling()
{
  if ( sensor_busy ) return 1;
  return 0;
}

// used to sleep while taking adc, this wakes us
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
  ADC10CTL0 &= ~ENC; // make sure this is off otherwise settings are locked.
  ADC10CTL0 &= ~(ADC10IFG | ADC10IE);
  LPM4_EXIT;
  return;
}
