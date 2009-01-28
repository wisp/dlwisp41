
/*
Copyright (c) 2009, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.
   * Neither the name of Intel Corporation nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// these bit definitions are specific to WISP 4.0 DL

#define ACCEL_ENABLE_BIT          BIT6   // 1.6
#define SET_ACCEL_ENABLE_DIR      P1DIR |= ACCEL_ENABLE_BIT
#define CLEAR_ACCEL_ENABLE_DIR    P1DIR &= ~ACCEL_ENABLE_BIT
#define TURN_ON_ACCEL_ENABLE      P1OUT |= ACCEL_ENABLE_BIT
#define TURN_OFF_ACCEL_ENABLE     P1OUT &= ~ACCEL_ENABLE_BIT

#define X_INCH                INCH_2  // A2
#define Y_INCH                INCH_1  // A1
#define Z_INCH                INCH_0  // A0

#define DATA_LENGTH_IN_WORDS      3
#define DATA_LENGTH_IN_BYTES      (DATA_LENGTH_IN_WORDS*2)

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
        
        // Power sensor, enable analog in     
        ADC10AE0 |= (X_INCH + Y_INCH + Z_INCH);
        SET_ACCEL_ENABLE_DIR;
        TURN_ON_ACCEL_ENABLE;
        
        // a little time for regulator to stabilize active mode current AND
        // filter caps to settle.
        for(int i = 0; i < 350; i++);
        
        // Grab data
        unsigned int k = 0;
        for (int i = 0; i < DATA_LENGTH_IN_WORDS; i++)
        {
        
          ADC10CTL0 &= ~ENC; // make sure this is off otherwise settings are locked.
          ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ADC10IE;
          if (i == 0)
            ADC10CTL1 = ADC10DIV_4 + ADC10SSEL_0 + SHS_0 + CONSEQ_0 + X_INCH;
          else if (i == 1)
            ADC10CTL1 = ADC10DIV_4 + ADC10SSEL_0 + SHS_0 + CONSEQ_0 + Y_INCH;
          else
            ADC10CTL1 = ADC10DIV_4 + ADC10SSEL_0 + SHS_0 + CONSEQ_0 + Z_INCH;
          ADC10CTL0 |= ENC;
          ADC10CTL0 |= ADC10SC;
          LPM4;
          
          *(target + k + 1 ) = (ADC10MEM & 0xff);
          // grab msb bits and store it
          *(target + k) = (ADC10MEM & 0x0300) >> 8;
          k += 2;
        }
        
        // Power off sensor and adc
        CLEAR_ACCEL_ENABLE_DIR;
        TURN_OFF_ACCEL_ENABLE;
        ADC10AE0 &= ~(X_INCH + Y_INCH + Z_INCH);
        ADC10CTL0 &= ~ENC;
        ADC10CTL1 = 0;       // turn adc off
        ADC10CTL0 = 0;       // turn adc off
        
        return;
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
