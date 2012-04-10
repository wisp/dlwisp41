/* See license.txt for license information. */

#include "dlwisp41.h"
#include "temp_sensor.h"

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
        P1OUT &= ~RX_EN_PIN;   // turn off comparator
        
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
