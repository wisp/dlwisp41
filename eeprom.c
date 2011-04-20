/* See license.txt for license information. */

/*
 * ssclark@cs.umass.edu notes: "The write_eeprom and read_eeprom code was
 * contributed to the WISP wiki by M.B. Moessner.  I separated the code into 2
 * files, added a hardware delay, and added comments."
 */

#include "eeprom.h"

extern unsigned char deviceaddress=0x50;
extern unsigned char byteaddress=0x00;
extern unsigned char block=0x00;

void init_eeprom() {
  P3SEL |= 0x06;							// Assign I2C pins to USCI_B0
  UCB0CTL1 |= UCSWRST;						// Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;		// I2C master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;			// Use SMCLK, keep SW reset
  UCB0BR0 =12;								// fSCL = SMCLK/12 = ~100kHz
  UCB0BR1 = 0;
  UCB0CTL1 &= ~UCSWRST;						// Clear SW reset, resume operation
}

unsigned char write_eeprom(int address, unsigned char *data, int length){
  int i = 0;

  UCB0I2CSA = 0x50 | ((address >> 8) & 0x03);		// Set slave address

  UCB0CTL1 |= UCTR + UCTXSTT;         // I2C TX, start condition, transmit mode
  while(!(IFG2 & UCB0TXIFG));			// Wait for the TXT flag to set
  IFG2 &= ~UCB0TXIFG;					// Clear the TX buffer flag
  UCB0TXBUF = byteaddress;			// Send the write address
  while(UCB0CTL1 & UCTXSTT);			// Wait for the start condition to clear

  if(UCB0STAT & UCNACKIFG)			// If a NACK is received
  {
    UCB0STAT &= ~UCNACKIFG;			// Clear the NACK
    UCB0CTL1 |= UCTXSTP;			// Send STP
    while(UCB0CTL1 & UCTXSTP);		// Wait for STP to clear
    return 0;						// Return 0 (failure)
  }

  for(i=0;i<length;i++){
    while(!(IFG2 & UCB0TXIFG));		// Wait for the TX flag to set
    IFG2 &= ~UCB0TXIFG;				// Clear the TX buffer flag
    UCB0TXBUF = *data;				// Send a byte of data
    data++;
  }
  while(!(IFG2 & UCB0TXIFG));			// Wait for the TX flag to set
  UCB0CTL1 |= UCTXSTP;				// Send stop condition
  while(UCB0CTL1 & UCTXSTP);			// Wait for the stop condition to clear


  delay_cycles(4000);					// Sleep 4k cycles
  IFG2 &= ~UCB0TXIFG;					// Clear TX flag

  return 1;							// Return 1 (success)
}

unsigned char read_eeprom(int address, unsigned char *data, int length){
  int i=0;

  UCB0I2CSA = 0x50 | ((address >> 8) & 0x03);		// Set slave address

  for(i=0;i<length;i++){

    UCB0CTL1 |= UCTR + UCTXSTT;		// I2C startion condition, transmit mode
    while(!(IFG2 & UCB0TXIFG));		// Wait for TX flag to set
    IFG2 &= ~UCB0TXIFG;				// Clear the TX buffer flag
    UCB0TXBUF = byteaddress+i;		// Send the read address
    while(UCB0CTL1 & UCTXSTT);		// Wait for start condition to clear

    if(UCB0STAT & UCNACKIFG)			// If a NACK is received
    {
      UCB0STAT &= ~UCNACKIFG;			// Clear the NACK
      UCB0CTL1 |= UCTXSTP;			// Send STP
      while(UCB0CTL1 & UCTXSTP);		// Wait for STP to clear
      return 0;						// Return 0 (failure)
    }

    while(!(IFG2 & UCB0TXIFG));		// Wait for TX flag to set
    IFG2 &= ~UCB0TXIFG;				// Clear the TX flag

    UCB0CTL1 &= ~UCTR;				// I2C start condition, receive mode
    IFG2 &= ~UCB0RXIFG;				// Clear the RX flag
    IFG2 &= ~UCB0TXIFG;				// Clear the TX flag
    UCB0CTL1 |= UCTXSTT;			// Send a start condition
    while(UCB0CTL1 & UCTXSTT);		// Wait for the start to clear
    UCB0CTL1 |= UCTXSTP;			// Send a stop condition

    while(UCB0CTL1 & UCTXSTP);		// Wait for the stop condition to set

    *data = UCB0RXBUF;				// Send byte of data
    data++;
    IFG2 &= ~UCB0RXIFG;				// Clear receive flag

    delay_cycles(4000);				// Sleep 4k cycles
  }

  return 1;							// Return 1 (success)
}

// Enter LPM3 for <i>num_of_cycles</i> cycles, giving the EEPROM chip time to
// complete operations. If we do not sleep long enough, the EEPROM will get into
// a bad state recoverable only by power cycling.
void delay_cycles(unsigned int num_of_cycles) {
  TA1CCTL1 = CCIE;                       // TA1CCR1 interrupt enabled
  TA1CCR1 = num_of_cycles;              // Cycles to count before triggering
  TA1CTL = TASSEL_1 + MC_2 + TAIE;      // ACLCK, continuous, interrupt enabled
  __bis_SR_register(LPM3_bits + GIE);   // Enter LPM3 with interrupt
}

//Timer1_A1 interrupt vector handler
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1(void)
{
  TA1CCTL1 = 0;             // Clear interrupt flag
  TA1CTL = 0;               // Clear control register (disables interrupts)
  TA1R = 0;                 // Reset overflow
  TA1CCR1 = 0;              // Reset capture/compare register
  LPM3_EXIT;                // Wake up
}
