/* See license.txt for license information. */
#include "mywisp.h"
#if (ACTIVE_SENSOR == SENSOR_ACCEL_QUICK)

#include "dlwisp41.h"
#include "rfid.h"
#include "quick_accel_sensor.h"

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

  *(target+1) = (ADC10MEM & 0xff);
  // grab msb bits and store it
  *(target) =   (ADC10MEM & 0x0300) >> 8;

  // GRAB DATA
  ADC10CTL0 &= ~ENC; // make sure this is off otherwise settings are locked.
  ADC10CTL0 = SREF_0 + ADC10SHT_1 + ADC10ON;
  ADC10CTL1 = ADC10DIV_2 + ADC10SSEL_0 + SHS_0 + CONSEQ_0 + INCH_ACCEL_Y;
  ADC10CTL0 |= ENC;
  ADC10CTL0 |= ADC10SC;

  while (ADC10CTL1 & ADC10BUSY);    // wait while ADC finished work

  *(target+3) = (ADC10MEM & 0xff);
  // grab msb bits and store it
  *(target+2) = (ADC10MEM & 0x0300) >> 8;

  // GRAB DATA
  ADC10CTL0 &= ~ENC; // make sure this is off otherwise settings are locked.
  ADC10CTL0 = SREF_0 + ADC10SHT_1 + ADC10ON;
  ADC10CTL1 = ADC10DIV_2 + ADC10SSEL_0 + SHS_0 + CONSEQ_0 + INCH_ACCEL_Z;
  ADC10CTL0 |= ENC;
  ADC10CTL0 |= ADC10SC;

  while (ADC10CTL1 & ADC10BUSY);    // wait while ADC finished work

  *(target+5) = (ADC10MEM & 0xff);
  // grab msb bits and store it
  *(target+4) = (ADC10MEM & 0x0300) >> 8;

  // Power off sensor and adc
  P1DIR &= ~ACCEL_POWER;
  P1OUT &= ~ACCEL_POWER;
  ADC10CTL0 &= ~ENC;
  ADC10CTL1 = 0;       // turn adc off
  ADC10CTL0 = 0;       // turn adc off

  // Store sensor read count
  sensor_counter++;
  *(target+7) = (sensor_counter & 0x00ff);
  // grab msb bits and store it
  *(target+6)  = (sensor_counter & 0xff00) >> 8;

  // turn on comparator
  P1OUT |= RX_EN_PIN;
}

#endif // (ACTIVE_SENSOR == SENSOR_ACCEL_QUICK)
