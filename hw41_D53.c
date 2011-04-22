/* See license.txt for license information. */

//**********************************Timer setup**********************************
//******************************************************************************
//  WISP G2.0
//  Device: MSP430-2012
//
//  Pinout:
//             P1.1 = Data out signal
//             P1.2 = Data in signal
//             P1.3 = Data in enable
//
//             P2.4 = Supervisor In
//             P4.3/5/7 = Debug pin
//
//  
//******************************************************************************

////////////////////////////////////////////////////////////////////////////////
// Step 1: pick an application
// simple hardcoded query-ack
#define SIMPLE_QUERY_ACK              1
// return sampled sensor data as epc. best for range.
#define SENSOR_DATA_IN_ID             0
// support read commands. returns one word of 0x0304
#define SIMPLE_READ_COMMAND           0
// return sampled sensor data in a read command. returns three words of accel data
#define SENSOR_DATA_IN_READ_COMMAND   0
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Step 2: pick a reader
#define ALIEN_9800_READER             0           // FIXME FIXME - needs fixing
#define IMPINJ_READER                 1
////////////////////////////////////////////////////////////////////////////////

#if SIMPLE_QUERY_ACK
#define ENABLE_READS                  0
#define READ_SENSOR                   0
#warning "compiling simple query-ack application"
#endif
#if SENSOR_DATA_IN_ID
#define ENABLE_READS                  0
#define READ_SENSOR                   1
#warning "compiling sensor data in id application"
#endif
#if SIMPLE_READ_COMMAND
#define ENABLE_READS                  1
#define READ_SENSOR                   0
#warning "compiling simple read command application"
#endif
#if SENSOR_DATA_IN_READ_COMMAND
#define ENABLE_READS                  1
#define READ_SENSOR                   1
#warning "compiling sensor data in read command application"
#endif

// pick only one
#define MILLER_2_ENCODING             0             // not tested ... use ayor
#define MILLER_4_ENCODING             1

// make sure this syncs with project target
#define USE_2132  1
#if USE_2132
#include <msp430x21x2.h>
#else
#include <msp430x22x4.h>
#endif

#define VOLTAGE_SV_PIN                BIT4          // (of port 2) = P2.4 - input pin for voltage supervisor
#define VOLTAGE_SV_ALT_PIN            BIT3       // (of port 2) = P2.3 - input pin for voltage supervisor
#define INPUT_PIN                     BIT2          // (of port 1) = P1.2
#define BIT_IN_ENABLE                 BIT3          // P1.3
#define OUTPUT_PIN                    BIT1          // P1.1

// used for debugging only
#define SIMULATE_SV_INTERRUPT         0
int power_counter = 0;
#define FAKE_VOLTAGE_SV_PIN           BIT7      // (of port 1) = 1.7. port 2.4 isn't physically exposed, so if you want to fake
                                        // out the sv interrupt pin, you have to redefine it to a pin that is physically exposed
#define DEBUG_PINS_ENABLED            1
#define DEBUG_PIN5_HIGH               P3OUT |= BIT5;
#define DEBUG_PIN5_LOW                P3OUT &= ~BIT5;


// ------------------------------------------------------------------------

#if ALIEN_9800_READER
#define SEND_CLOCK  \
  BCSCTL1 = XT2OFF + RSEL3 + RSEL1 ; \
  DCOCTL = 0;
  //DCOCTL = DCO2;
  // DCO seems to work from dco0 up to dco2
#define RECEIVE_CLOCK \
  BCSCTL1 = XT2OFF + RSEL3 + RSEL2;  \
  DCOCTL = DCO1+DCO0; \
  BCSCTL2 = 0; // Rext = ON
  // for rsel 3 + 2, dco works from 0 to DCO2
  // have not tested rsel 3 + 1
#else
#define SEND_CLOCK  \
  BCSCTL1 = XT2OFF + RSEL3 + RSEL0 ; \
    DCOCTL = DCO2 + DCO1 ;
  //BCSCTL1 = XT2OFF + RSEL3 + RSEL1 ; \
  //DCOCTL = 0;
#define RECEIVE_CLOCK \
  BCSCTL1 = XT2OFF + RSEL3 + RSEL1 + RSEL0; \
  DCOCTL = 0; \
  BCSCTL2 = 0; // Rext = ON
#if 0
  // old 5.7 Mhz clock
  BCSCTL1 = XT2OFF + RSEL3 + RSEL2; /* DCOCTL = DCO1; */ \
  DCOCTL = DCO1+DCO0; \
  BCSCTL2 = 0; // Rext = ON
#endif
#endif

#define BUFFER_SIZE 16                         // max of 16 bytes rec. from reader
#define MAX_BITS (BUFFER_SIZE * 8)
#define POLY5 0x48
volatile unsigned char cmd[BUFFER_SIZE+1];          // stored cmd from reader
//volatile unsigned char reply[BUFFER_SIZE+1]= { 0x30, 0x35, 0xaa, 0xab, 0x55,0xff,0xaa,0xab,0x55,0xff,0xaa,0xab,0x55,0xff,0x00, 0x00};
volatile unsigned char* destorig = &cmd[0];         // pointer to beginning of cmd

// #pragma data_alignment=2 is important in sendResponse() when the words are copied into arrays.
// Sometimes the compiler puts reply[0] on an
// odd address, which cannot be copied as a word and thus screws everything up.
#pragma data_alignment=2

volatile unsigned char queryReply[]= { 0xf0, 0x0f, 0x00, 0x00};

// ackReply:  First two bytes are the preamble.  Last two bytes are the crc.
volatile unsigned char ackReply[]  = { 0x30, 0x00, 0x0D, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

volatile unsigned char readReply[] = { 
                                    // header - 1 bit - 0 if successful, 1 if error code follows
                                    // memory words - hardcoded to 16 bits of 0xffff for now
                                    // rn - 16 bits - hardcoded to 0xf00f for now
                                    // crc-16 - 16 bits - precomputed as 0x06 0x72
                                    // filler - 15 bits of nothing (don't send)
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19};

unsigned char RN16[23];

// compiler uses working register 4 as a global variable
// Pointer to &cmd[bits]
extern volatile __no_init __regvar unsigned char* dest @ 4;

// compiler uses working register 5 as a global variable
// count of bits received from reader
extern volatile __no_init __regvar unsigned short bits @ 5;
unsigned short TRcal=0;

#define STATE_READY               0
#define STATE_ARBITRATE           1
#define STATE_REPLY               2
#define STATE_ACKNOWLEDGED        3
#define STATE_OPEN                4
#define STATE_SECURED             5
#define STATE_KILLED              6
#define STATE_READ_SENSOR         7
#define FLASH_SEGMENT_ADDRESS 0x1000

#define ENABLE_SLOTS  0         // if you want to be a good protocol citizen, set to 1. if you want maximum range, set to 0

#if ENABLE_SLOTS
#define NUM_QUERY_BITS          20
#define MAX_NUM_QUERY_BITS      25
#define NUM_QUERYADJ_BITS       9
#define MAX_NUM_QUERYADJ_BITS   9
#define NUM_READ_BITS           53  // 60 bits actually received, but need to break off early for computation
#define MAX_NUM_READ_BITS       60
#else
#define NUM_QUERY_BITS          24
#define MAX_NUM_QUERY_BITS      25
#define NUM_QUERYADJ_BITS       9
#define MAX_NUM_QUERYADJ_BITS   9
#define NUM_READ_BITS           55   // 60 bits actually received, but need to break off early for computation
#define MAX_NUM_READ_BITS       60
#endif
#define NUM_ACK_BITS            20
#define NUM_REQRN_BITS          41
#define NUM_NAK_BITS            10

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
unsigned short Q = 0, shift = 0;
unsigned short readTempSensor;
unsigned char readAddress;
unsigned int counter = 0;
unsigned char timeToSample = 0;

volatile unsigned short LPM_settings = 0;
volatile unsigned short inSleepMode = 0;
volatile unsigned short waitingForBits = 0;

void sendToReader(volatile unsigned char *data, unsigned char numOfBits);
unsigned short crc16_ccitt(volatile unsigned char *data, unsigned short n);
unsigned char crc5(volatile unsigned char *buf, unsigned short numOfBits);
void setup_to_receive();
void sleep();
unsigned short is_power_good();
void lfsr();
inline void loadRN16(), mixupRN16();
void crc16_ccitt_readReply(unsigned int);
int i;
inline void handle_query(volatile short nextState);
inline void handle_queryrep(volatile short nextState);
inline void handle_queryadjust(volatile short nextState);
inline void handle_select(volatile short nextState);
inline void handle_ack(volatile short nextState);
inline void handle_request_rn(volatile short nextState);
inline void handle_read(volatile short nextState);
inline void handle_nak(volatile short nextState);
inline void do_nothing();

#if READ_SENSOR
#include "accel_sensor.h"
#endif

int main(void)
{
  //**********************************Timer setup**********************************
  WDTCTL = WDTPW + WDTHOLD;            // Stop Watchdog Timer 
  
  P1SEL = 0;
  P2SEL = 0;
  
  P1IE = 0;
  P1IFG = 0;
  P2IFG = 0;
  P2IES = VOLTAGE_SV_ALT_PIN;
  P2IE |= (VOLTAGE_SV_PIN + VOLTAGE_SV_ALT_PIN);
  // VOLTAGE_SV_PIN fires when VS goes from low to high;
  // VOLTAGE_SV_ALT_PIN fires when VS goes from high to low
  
#if !SIMULATE_SV_INTERRUPT
  // Check power on bootup, decide to receive or sleep.
  if(!is_power_good())
    sleep();
#endif
  
  RECEIVE_CLOCK;
  
#if DEBUG_PINS_ENABLED
#if USE_2132
  DEBUG_PIN5_LOW;
  P3DIR |= (BIT7+BIT5+BIT3);
#else
  P4DIR |= (BIT7+BIT5+BIT3);
#endif
#endif 
  
#if 1
  // setup int epc
  epc = ackReply[2]<<8;
  epc |= ackReply[3];
  
  // calculate RN16_1 table
  for (Q = 0; Q < 16; Q++)
  {
    rn16 = epc^Q;
    lfsr();
    
    if (Q > 8)
    {
      RN16[(Q<<1)-9] = __swap_bytes(rn16);
      RN16[(Q<<1)-8] = rn16;
    }
    else
    {
      RN16[Q] = rn16;
    }
  }
#endif
  
  TACTL = 0;
  //TBCTL = 0;
  
#if DEBUG_PINS_ENABLED
#if USE_2132
#if 0
  P3OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
  P3OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
  P3OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
  P3OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
  P3OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
  P3OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
  P3OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
  P3OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
  P3OUT &= ~BIT5;
#endif
#else
  P4OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
  P4OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
  P4OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
  P4OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
  P4OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
  P4OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
  P4OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
  P4OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
  P4OUT &= ~BIT5;
#endif
#endif
  
  RECEIVE_CLOCK;

  P1OUT = 0;
  //P1DIR = 0xFF;               // by default all P1 pins are output
  P1DIR &= ~ INPUT_PIN;
  P1DIR |= BIT_IN_ENABLE;
  //P2DIR = (BIT0+BIT1+BIT2+BIT3+BIT5);               // by default all P2 pins are output
  //P2DIR &= ~VOLTAGE_SV_PIN;

//  P1IES &= ~BIT2; // initial state is POS edge to find start of CW
//  P1IFG = 0x00;       // clear interrupt flag after changing edge trigger

  asm("MOV #0000h, R9");
  // dest = destorig;
  
#if READ_SENSOR
  init_sensor();
#endif

#if !(ENABLE_SLOTS)
  queryReplyCRC = crc16_ccitt(&queryReply[0],2);
  queryReply[3] = (unsigned char)queryReplyCRC;
  queryReply[2] = (unsigned char)__swap_bytes(queryReplyCRC);
#endif
  
#if SENSOR_DATA_IN_ID
  // this branch is for sensor data in the id
  state = STATE_READ_SENSOR;
  timeToSample++;
#else
  ackReplyCRC = crc16_ccitt(&ackReply[0], 14);
  ackReply[15] = (unsigned char)ackReplyCRC;
  ackReply[14] = (unsigned char)__swap_bytes(ackReplyCRC);
#endif
  
  state = STATE_ARBITRATE;
  setup_to_receive();
  
  while(1)
  {   
    
    // TIMEOUT!  reset timer
    if (TAR > 0x256 || delimiterNotFound)   // was 0x1000
    {  
      if(!is_power_good()) {
        sleep();
      }
#if SENSOR_DATA_IN_ID
    // this branch is for sensor data in the id
      if ( timeToSample++ == 10 ) {
      state = STATE_READ_SENSOR;
      timeToSample = 0;
    }
    //else {z
    //  state = STATE_ARBITRATE;
    //}
#else
#if !(ENABLE_READS)
    if(!is_power_good()) 
        sleep();
#endif
     state = STATE_ARBITRATE;
#endif
    
#if 1
    if (shift < 4)
        shift += 1;
    else
        shift = 0;
#endif
       
      setup_to_receive();
    }
    
    //DEBUG_PIN5_HIGH;
    //DEBUG_PIN5_LOW;
 
    switch (state)
    {
      case STATE_READY:
      {
        //////////////////////////////////////////////////////////////////////
        // process the QUERY command
        //////////////////////////////////////////////////////////////////////
        if ( bits == NUM_QUERY_BITS  && ( ( cmd[0] & 0xF0 ) == 0x80 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_query(STATE_REPLY);
          //DEBUG_PIN5_LOW;
          //if ( counter == 0xffff ) counter = 0; else counter++;
          setup_to_receive();
        }
        //////////////////////////////////////////////////////////////////////
        // process the SELECT command
        //////////////////////////////////////////////////////////////////////
        // @ short distance has slight impact on performance
        else if ( bits >= 44  && ( ( cmd[0] & 0xF0 ) == 0xA0 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_select(STATE_READY);
          //DEBUG_PIN5_LOW;
          delimiterNotFound = 1;
        } // select command
        //////////////////////////////////////////////////////////////////////
        // got >= 22 bits, and it's not the beginning of a select. just reset.
        //////////////////////////////////////////////////////////////////////
        else if ( bits >= MAX_NUM_QUERY_BITS && ( ( cmd[0] & 0xF0 ) != 0xA0 ) )
        { 
          do_nothing();
          state = STATE_READY;
          delimiterNotFound = 1;
        }   
        break;
      }
      case STATE_ARBITRATE:		
      {     
        //////////////////////////////////////////////////////////////////////
        // process the QUERY command
        //////////////////////////////////////////////////////////////////////
#if ENABLE_SLOTS
        if ( bits == 21  && ( ( cmd[0] & 0xF0 ) == 0x80 ) )
#else
        if ( bits == NUM_QUERY_BITS  && ( ( cmd[0] & 0xF0 ) == 0x80 ) )
#endif
        {
          //DEBUG_PIN5_HIGH;
          handle_query(STATE_REPLY);
          //DEBUG_PIN5_LOW;
          //if ( counter == 0xffff ) counter = 0; else counter++;
          setup_to_receive();
        }
        //////////////////////////////////////////////////////////////////////
        // got >= 22 bits, and it's not the beginning of a select. just reset.
        //////////////////////////////////////////////////////////////////////
        //else if ( bits >= NUM_QUERY_BITS )
        else if ( bits >= MAX_NUM_QUERY_BITS && ( ( cmd[0] & 0xF0 ) != 0xA0 ) )
        {
          //DEBUG_PIN5_HIGH;
          do_nothing();
          state = STATE_READY;
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        }
        // this state handles query, queryrep, queryadjust, and select commands.
        //////////////////////////////////////////////////////////////////////
        // process the QUERYREP command
        //////////////////////////////////////////////////////////////////////
        else if ( bits == 4 && ( ( cmd[0] & 0x03 ) == 0x00 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_queryrep(STATE_REPLY);
          //DEBUG_PIN5_LOW;
          //setup_to_receive();
          delimiterNotFound = 1;
        } // queryrep command
        //////////////////////////////////////////////////////////////////////
        // process the QUERYADJUST command
        //////////////////////////////////////////////////////////////////////
        else if ( bits == NUM_QUERYADJ_BITS  && ( ( cmd[0] & 0xF8 ) == 0x48 ) )
        {
          // at short distance, you get better performance (~52 t/s) if you
          // do setup_to_receive() rather than dnf =1. not sure that this holds
          // true at distance though - need to recheck @ 2-3 ms.
          //DEBUG_PIN5_HIGH;
          handle_queryadjust(STATE_REPLY);
          //DEBUG_PIN5_LOW;
          setup_to_receive();
          //delimiterNotFound = 1;
        } // queryadjust command
        //////////////////////////////////////////////////////////////////////
        // process the SELECT command
        //////////////////////////////////////////////////////////////////////
        // @ short distance has slight impact on performance
        else if ( bits >= 44  && ( ( cmd[0] & 0xF0 ) == 0xA0 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_select(STATE_READY);
          //DEBUG_PIN5_LOW;
          delimiterNotFound = 1;
        } // select command
       
      break;
      }
    
      case STATE_REPLY:		
      {
        // this state handles query, query adjust, ack, and select commands
        ///////////////////////////////////////////////////////////////////////
        // process the ACK command
        ///////////////////////////////////////////////////////////////////////
        // spec sez this is 18 bits -- where do the extra two bits come from?!?
        if ( bits == NUM_ACK_BITS  && ( ( cmd[0] & 0xC0 ) == 0x40 ) )
        {
#if ENABLE_READS
          //DEBUG_PIN5_HIGH;
          handle_ack(STATE_ACKNOWLEDGED);
          //DEBUG_PIN5_LOW;
          setup_to_receive();
#elif SENSOR_DATA_IN_ID
          handle_ack(STATE_READY);
          delimiterNotFound = 1; // reset
#else
          // this branch for hardcoded query/acks
          //DEBUG_PIN5_HIGH;
          handle_ack(STATE_ACKNOWLEDGED);
          //DEBUG_PIN5_LOW;
          //delimiterNotFound = 1; // reset
          setup_to_receive();
#endif
        }
        //////////////////////////////////////////////////////////////////////
        // process the QUERY command
        //////////////////////////////////////////////////////////////////////
        //else if ( bits == 20  && ( ( cmd[0] & 0xF0 ) == 0x80 ) )
        // STOP!!!! do not change this lightly!!!!!!!!!!!!
        else if ( bits == NUM_QUERY_BITS  && ( ( cmd[0] & 0xF0 ) == 0x80 ) )
        {
          // i'm supposed to stay in state_reply when I get this, but if I'm
          // running close to 1.8v then I really need to reset and get in the
          // sleep, which puts me back into state_arbitrate. this is complete
          // a violation of the protocol, but it sure does make everything
          // work better. - polly 8/9/2008
          //DEBUG_PIN5_HIGH;
          handle_query(STATE_REPLY);
          //DEBUG_PIN5_LOW;
          //delimiterNotFound = 1;
          setup_to_receive();
        }
        //////////////////////////////////////////////////////////////////////
        // process the QUERYREP command
        //////////////////////////////////////////////////////////////////////
        else if ( bits == 4 && ( ( cmd[0] & 0x03 ) == 0x00 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_queryrep(STATE_ARBITRATE);
          //DEBUG_PIN5_LOW;
          setup_to_receive();
          //delimiterNotFound = 1; // reset
        } // queryrep command
        //////////////////////////////////////////////////////////////////////
        // process the QUERYADJUST command
        //////////////////////////////////////////////////////////////////////
          else if ( bits == NUM_QUERYADJ_BITS  && ( ( cmd[0] & 0xF8 ) == 0x48 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_queryadjust(STATE_REPLY);
          //DEBUG_PIN5_LOW;
          //setup_to_receive();
          delimiterNotFound = 1;
        } // queryadjust command
        // (maybe fixed?)
        // FIXME FIXME - if this is enabled is completely KILLS performance
        // @ 18 inches - 50 t/s if commented out
        // 8 t/s if dnf = 1 15 t/s if setup_to_receive()
        //////////////////////////////////////////////////////////////////////
        // process the SELECT command
        //////////////////////////////////////////////////////////////////////
        else if ( bits >= 44  && ( ( cmd[0] & 0xF0 ) == 0xA0 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_select(STATE_READY); 
          //DEBUG_PIN5_LOW;
          delimiterNotFound = 1;
          //setup_to_receive();
        } // select command
        else if ( bits >= MAX_NUM_QUERY_BITS && ( ( cmd[0] & 0xF0 ) != 0xA0 ) && ( ( cmd[0] & 0xF0 ) != 0x80 ) )
        {
          //DEBUG_PIN5_HIGH;
          do_nothing();
          state = STATE_READY;
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        }
        break;
      }
      case STATE_ACKNOWLEDGED:		
      {      
        // responds to query, ack, request_rn cmds
        // takes action on queryrep, queryadjust, and select cmds
        /////////////////////////////////////////////////////////////////////
        // process the REQUEST_RN command
        //////////////////////////////////////////////////////////////////////
        // FIXME FIXME - why 42 instead of 40?!?
        if ( bits >= NUM_REQRN_BITS && ( cmd[0] == 0xC1 ) )
        //if ( bits >= 40  && ( cmd[0] == 0xC1 ) )
        {
#if 1
          //DEBUG_PIN5_HIGH;
          handle_request_rn(STATE_OPEN);
          //DEBUG_PIN5_LOW;
          setup_to_receive();
#else
          handle_request_rn(STATE_READY);
          delimiterNotFound = 1;
#endif
        }
        
#if 1
                 
#if 1
        //////////////////////////////////////////////////////////////////////
        // process the QUERY command
        //////////////////////////////////////////////////////////////////////
        else if ( bits == NUM_QUERY_BITS  && ( ( cmd[0] & 0xF0 ) == 0x80 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_query(STATE_REPLY);
          //DEBUG_PIN5_LOW;
          delimiterNotFound = 1;
          //setup_to_receive();
        }
#endif
#if 1
        ///////////////////////////////////////////////////////////////////////
        // process the ACK command
        ///////////////////////////////////////////////////////////////////////
        // this code doesn't seem to get exercised in the real world. if i ever
        // ran into a reader that generated an ack in an acknowledged state,
        // this code might need some work.
        //else if ( bits == 20  && ( ( cmd[0] & 0xC0 ) == 0x40 ) )
        else if ( bits == NUM_ACK_BITS  && ( ( cmd[0] & 0xC0 ) == 0x40 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_ack(STATE_ACKNOWLEDGED);
          //DEBUG_PIN5_LOW;
          setup_to_receive();
        }
#endif
#if 1

        //////////////////////////////////////////////////////////////////////
        // process the QUERYREP command
        //////////////////////////////////////////////////////////////////////
        else if ( bits == 4 && ( ( cmd[0] & 0x03 ) == 0x00 ) )
        {
          // in the acknowledged state, rfid chips don't respond to queryrep commands
          //DEBUG_PIN5_HIGH;
          do_nothing();
          state = STATE_READY; // a terrible idea that kills reads ...
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        } // queryrep command

        //////////////////////////////////////////////////////////////////////
        // process the QUERYADJUST command
        //////////////////////////////////////////////////////////////////////
        else if ( bits == NUM_QUERYADJ_BITS  && ( ( cmd[0] & 0xF8 ) == 0x48 ) )
        {
          //DEBUG_PIN5_HIGH;
          do_nothing();
          state = STATE_READY; // a terrible idea that kills reads ...
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        } // queryadjust command
        //////////////////////////////////////////////////////////////////////
        // process the SELECT command
        //////////////////////////////////////////////////////////////////////
        else if ( bits >= 44  && ( ( cmd[0] & 0xF0 ) == 0xA0 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_select(STATE_READY); 
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        } // select command   
        //////////////////////////////////////////////////////////////////////
        // process the NAK command
        //////////////////////////////////////////////////////////////////////
        else if ( bits >= 10 && ( cmd[0] == 0xC0 ) )
        //else if ( bits >= NUM_NAK_BITS && ( cmd[0] == 0xC0 ) )
        {
          DEBUG_PIN5_HIGH;
          do_nothing();
          state = STATE_ARBITRATE; 
          delimiterNotFound = 1;
          DEBUG_PIN5_LOW;
        } 
        //////////////////////////////////////////////////////////////////////
        // process the READ command
        //////////////////////////////////////////////////////////////////////
        // warning: won't work for read addrs > 127d
        if ( bits == NUM_READ_BITS && ( cmd[0] == 0xC2 ) )
        {
          DEBUG_PIN5_HIGH;
          handle_read(STATE_ARBITRATE);
          state = STATE_ARBITRATE;
          delimiterNotFound = 1 ;
          DEBUG_PIN5_LOW;
        }
        // FIXME: need write, kill, lock, blockwrite, blockerase
        //////////////////////////////////////////////////////////////////////
        // process the ACCESS command
        //////////////////////////////////////////////////////////////////////
        if ( bits >= 56  && ( cmd[0] == 0xC6 ) )
        {
          //DEBUG_PIN5_HIGH;
          do_nothing();
          state = STATE_ARBITRATE;
          delimiterNotFound = 1 ;
          //DEBUG_PIN5_LOW;
        }
#endif
        //else if ( bits == 9999 )
        else if ( bits >= MAX_NUM_READ_BITS )
        {
          //DEBUG_PIN5_HIGH;
          //do_nothing();
          state = STATE_ARBITRATE;
          delimiterNotFound = 1 ;
          //DEBUG_PIN5_LOW;
        }
#endif
          
        
#if 0
        // kills performance ...
        else if ( bits >= 44 ) 
        {
          do_nothing();
          state = STATE_ARBITRATE;
          delimiterNotFound = 1;
        }
#endif
        break;
      }
      case STATE_OPEN:		
      {

#if 1
        //DEBUG_PIN5_HIGH;
        //DEBUG_PIN5_LOW;
        
        // responds to query, ack, req_rn, read, write, kill, access, blockwrite, and blockerase cmds
        // processes queryrep, queryadjust, select cmds
        //////////////////////////////////////////////////////////////////////
        // process the READ command
        //////////////////////////////////////////////////////////////////////
        // warning: won't work for read addrs > 127d
        if ( bits == NUM_READ_BITS  && ( cmd[0] == 0xC2 ) )
        {
          DEBUG_PIN5_HIGH;
          handle_read(STATE_OPEN);
          DEBUG_PIN5_LOW;
          // note: setup_to_receive() et al handled in handle_read
        }
#if 1
        //////////////////////////////////////////////////////////////////////
        // process the REQUEST_RN command
        //////////////////////////////////////////////////////////////////////
        else if ( bits >= NUM_REQRN_BITS  && ( cmd[0] == 0xC1 ) )
          //else if ( bits >= 30  && ( cmd[0] == 0xC1 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_request_rn(STATE_OPEN);
          setup_to_receive();
          //DEBUG_PIN5_HIGH;
          //delimiterNotFound = 1; // more bad reads??
         }
#endif
        //////////////////////////////////////////////////////////////////////
        // process the QUERY command
        //////////////////////////////////////////////////////////////////////
        else if ( bits == NUM_QUERY_BITS  && ( ( cmd[0] & 0xF0 ) == 0x80 ) )
        //else if ( bits >= 16  && ( ( cmd[0] & 0xF0 ) == 0x80 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_query(STATE_REPLY);
          //setup_to_receive();
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        }
        //////////////////////////////////////////////////////////////////////
        // process the QUERYREP command
        //////////////////////////////////////////////////////////////////////
        else if ( bits == 4 && ( ( cmd[0] & 0x03 ) == 0x00 ) ) 
        {
          //DEBUG_PIN5_HIGH;
          do_nothing();
          state = STATE_READY;
          //delimiterNotFound = 1;
          setup_to_receive();
          //DEBUG_PIN5_LOW;
        } // queryrep command
        //////////////////////////////////////////////////////////////////////
        // process the QUERYADJUST command
        //////////////////////////////////////////////////////////////////////
        //else if ( bits == NUM_QUERYADJ_BITS  && ( ( cmd[0] & 0xF0 ) == 0x90 ) )
          else if ( bits == 9  && ( ( cmd[0] & 0xF8 ) == 0x48 ) )
        {
          //DEBUG_PIN5_HIGH;
          do_nothing();
          state = STATE_READY;
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        } // queryadjust command
        ///////////////////////////////////////////////////////////////////////
        // process the ACK command
        ///////////////////////////////////////////////////////////////////////
        //else if ( bits >= 18  && ( ( cmd[0] & 0xC0 ) == 0x40 ) )
        else if ( bits == NUM_ACK_BITS  && ( ( cmd[0] & 0xC0 ) == 0x40 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_ack(STATE_OPEN);
          //setup_to_receive();
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        }
        //////////////////////////////////////////////////////////////////////
        // process the SELECT command
        //////////////////////////////////////////////////////////////////////
        else if ( bits >= 44  && ( ( cmd[0] & 0xF0 ) == 0xA0 ) )
        {
          //DEBUG_PIN5_HIGH;
          handle_select(STATE_READY); 
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        } // select command
#if 1
        //////////////////////////////////////////////////////////////////////
        // process the NAK command
        //////////////////////////////////////////////////////////////////////
        //else if ( bits >= NUM_NAK_BITS && ( cmd[0] == 0xC0 ) )
        else if ( bits >= 10 && ( cmd[0] == 0xC0 ) )
        {
          DEBUG_PIN5_HIGH;
          handle_nak(STATE_ARBITRATE); 
          delimiterNotFound = 1;
          DEBUG_PIN5_LOW;
        }
#else
#if 1
        else if ( bits > MAX_NUM_READ_BITS )
        //else if ( bits >= 57 )
        //else if ( bits >= 8 &&  ( cmd[0] != 0xC1 ) && (cmd[0] != 0xC2) && ( cmd[0] != 0xC0)  )
        {
          //DEBUG_PIN5_HIGH;
          do_nothing();
          state = STATE_ARBITRATE;
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        }
#endif
#endif
#if 0
        //
        if ( bits >= 8 &&  ( cmd[0] == 0xC1)  )
        {
          P4OUT &= ~BIT7;
        }
#endif
#else
        if ( bits > 16  && ( ( cmd[0] & 0xC0 ) == 0x80 ) ) {
          //DEBUG_PIN5_HIGH;
          do_nothing();
          state = STATE_ARBITRATE;
          delimiterNotFound = 1;
          //DEBUG_PIN5_LOW;
        }
        // get 60 bits with impinj, but need to break off
        // early to do computation
        //else if ( bits > 16 && cmd[0] == 0xC2 ) {
        else if ( bits == 56 && cmd[0] == 0xC2 ) {
          DEBUG_PIN5_HIGH;
          //do_nothing();
          handle_read(STATE_OPEN);
          state = STATE_ARBITRATE;
          delimiterNotFound = 1;
          DEBUG_PIN5_LOW;
        }
        
        else if ( bits > (58 + 2) ) {
          do_nothing();
          state = STATE_ARBITRATE;
          delimiterNotFound = 1;
        }
#endif

        break;
      }
    
    case STATE_READ_SENSOR:
      {
        
#if SENSOR_DATA_IN_READ_COMMAND
        read_sensor(&readReply[0]);
        // crc is computed in the read state
        RECEIVE_CLOCK;
        state = STATE_ARBITRATE;  
        delimiterNotFound = 1; // reset
#elif SENSOR_DATA_IN_ID
        read_sensor(&ackReply[6]);
        RECEIVE_CLOCK;
        ackReplyCRC = crc16_ccitt(&ackReply[0], 14);
        ackReply[15] = (unsigned char)ackReplyCRC;
        ackReply[14] = (unsigned char)__swap_bytes(ackReplyCRC);
        state = STATE_ARBITRATE;
        delimiterNotFound = 1; // reset
#endif
        
        //state = STATE_ARBITRATE;  
        //delimiterNotFound = 1; // reset
        
        break;
      } // end case  
    } // end switch
    
  } // while loop
}

inline void handle_query(volatile short nextState)
{
  TAR = 0;
#if (!ENABLE_SLOTS)
  //if ( NUM_QUERY_BITS == 22 )
    while ( TAR < 90 ); // if bit test is 22
  //P1OUT &= ~BIT_IN_ENABLE;   // turn off comparator
  TACCTL1 &= ~CCIE;     // Disable capturing and comparing interrupt
  TAR = 0;
#endif

  // set up for TRcal
  if ( cmd[0] & BIT3)
  {
    divideRatio = 21;//64/3;
  } else
  {
    divideRatio = 8;
  }
  // set up for subcarrier symbol
  subcarrierNum = cmd[0] & (BIT2 | BIT1);
  if (subcarrierNum == 0)
  {
    subcarrierNum = 1;
  } else if ( subcarrierNum == 2 )
  {
    subcarrierNum = 2;
  } else if (subcarrierNum == 4)
  {
    subcarrierNum = 4;
  } else
  {
    subcarrierNum = 8;
  }
          
  // set up for TRext
  if (cmd[0] & BIT0)
  {
    TRext = 1;
  } else
  {
    TRext = 0;
  }
  
#if ENABLE_SLOTS
    // parse for Q number and choose a Q value randomly
  Q = (cmd[1] & 0x07)<<1;
  if ((cmd[2] & 0x80) == 0x80)
    Q += 0x01;
          
  // pick Q randomly
  Q >>= shift;
  //Q = 0;
          
  if (Q == 0)
  {
    loadRN16();   
    
    queryReplyCRC = crc16_ccitt(&queryReply[0],2);
    queryReply[3] = (unsigned char)queryReplyCRC;
    queryReply[2] = (unsigned char)__swap_bytes(queryReplyCRC);
    
    TACCTL1 &= ~CCIE;     // Disable capturing and comparing interrupt
    TAR = 0;
    
    sendToReader(&queryReply[0], 17); 
    state = nextState;
    
    mixupRN16();
  }
  else
  {
    cmd[0] = 0;
    //P4OUT ^= BIT7;
    state = STATE_ARBITRATE;
  }
#else
  sendToReader(&queryReply[0], 17); 
  state = nextState;
#endif
}

inline void handle_select(volatile short nextState)
{
  // when i implement the sl assert/deassert, that code goes here
  do_nothing();
  state = nextState;
}

inline void handle_queryrep(volatile short nextState)
{
  // queryrep command - impinj 3.0.2 sends many more of these than just plain queries
  TAR = 0;
  //P4OUT |= BIT7;
  while ( TAR < 150 );
  //while ( TAR < 300 );
  //P4OUT &= ~BIT7;
  //P1OUT &= ~BIT_IN_ENABLE;   // turn off comparator
  TACCTL1 &= ~CCIE; 
  TAR = 0;
  Q -= 1;
  sendToReader(&queryReply[0], 17);
  state = nextState;
}

inline void handle_queryadjust(volatile short nextState)
{
  TAR = 0;
#if !(ENABLE_SLOTS)
  while ( TAR < 300 );
  //P1OUT &= ~BIT_IN_ENABLE;   // turn off comparator
  TACCTL1 &= ~CCIE; 
  TAR = 0;
#endif
  
  
#if ENABLE_SLOTS
  //FIXME: check if Q is 15 and if so rotate value according to spec (p. 50)
  // parse for Q up or down number
  if ((cmd[1] & 0x03) == 0x03)
    Q += 1;
  else if ((cmd[1] & 0x03) == 0x01)
    Q -= 1;
          
  // pick Q randomly
  Q >>= shift;
          
  if (Q == 0)
  {
    loadRN16();   
    
    queryReplyCRC = crc16_ccitt(&queryReply[0],2);
    queryReply[3] = (unsigned char)queryReplyCRC;
    queryReply[2] = (unsigned char)__swap_bytes(queryReplyCRC);
    
    TACCTL1 &= ~CCIE;     // Disable capturing and comparing interrupt
    TAR = 0;
    
    sendToReader(&queryReply[0], 17); 
    state = nextState;
    
    mixupRN16();
  }
  else
  {
    cmd[0] = 0;
    //P4OUT ^= BIT7;
    state = STATE_ARBITRATE;
  }
#else
  sendToReader(&queryReply[0], 17); 
  state = nextState;
#endif
}

inline void handle_ack(volatile short nextState)
{
  TACCTL1 &= ~CCIE;
  TAR = 0;
  if ( NUM_ACK_BITS == 20 )
    while ( TAR < 90 );
  else
    while ( TAR < 400 );          // on the nose for 3.5MHz
  TAR = 0;
  //P1OUT &= ~BIT_IN_ENABLE;   // turn off comparator
  // after that sends tagResponse
  sendToReader(&ackReply[0], 129);
  state = nextState;
}

inline void do_nothing()
{
  TACCTL1 &= ~CCIE;
  TAR = 0;
  //P1OUT &= ~BIT_IN_ENABLE;   // turn off comparator
}

inline void handle_request_rn(volatile short nextState)
{
  TACCTL1 &= ~CCIE;
  TAR = 0;
  // FIXME FIXME
  // here's a mystery: if I enable this line below, I clobber the follow-up
  // read command. specifically, the read command's cmd[0] shows up as 0xFF.
  // if i leave this line commented out, everything's fine.
  // theory #1 was that the bit_in_enable line doesn't switch around fast
  // enough. but i do the same thing for all the other commands, and i don't
  // have any problems. also, the time space between request_rn and the read
  // is *much larger* than betwen the other commands. theory #1 disproved.
  // theory #2 was that i had fallen asleep and wasn't processing the incoming
  // bits. but it turns out that i am wide awake. theory #2 disproven.
  // theory #3. generally when i see 0xff in the cmd[0] field it means that
  // the bit buffer got overwritten. as far as I can tell, it hasn't, and there's
  // plenty of room in the receiving buffer. theory #3 disproven.
  // hmmm.
  //P1OUT &= ~BIT_IN_ENABLE;   // turn off comparator
  if ( NUM_REQRN_BITS == 42 )
    while ( TAR < 80 );
  else if ( NUM_REQRN_BITS == 41 )
    while ( TAR < 170 );
  TAR = 0;
  //P4OUT |= BIT7;
  sendToReader(&queryReply[0], 33);
  if ( counter == 0xffff ) counter = 0; else counter++;
  state = nextState;
  //P4OUT &= ~BIT7;
}

inline void handle_read(volatile short nextState)
{
            
#if SENSOR_DATA_IN_READ_COMMAND
  
  TACCTL1 &= ~CCIE;
  TAR = 0;
  
  readReply[DATA_LENGTH_IN_BYTES] = 0xf0;        // remember to restore correct RN before doing crc()
  readReply[DATA_LENGTH_IN_BYTES+1] = 0x0f;        // because crc() will shift bits to add
  crc16_ccitt_readReply(DATA_LENGTH_IN_BYTES);    // leading "0" bit.
            
  //while ( TAR < 800 ); 
  //P1OUT &= ~BIT_IN_ENABLE;   // turn off comparator
  TAR = 0;
            
  // 48 bits for data + 16 bits for the handle + 16 bits for the CRC + leading 0 + add one to number of bits for seong's xmit code
  sendToReader(&readReply[0], 82);
  // this branch is for sensor data in read command
  state = STATE_READ_SENSOR;
  //delimiterNotFound = 1;
            
#elif SIMPLE_READ_COMMAND

  //TACCTL1 &= ~CCIE;
  TAR = 0;
#if ENABLE_SLOTS
  while ( TAR < 0x55 );
#else
#endif
  while ( TAR < 0x25 );
  TAR = 0;
  
#define USE_COUNTER 1
#if USE_COUNTER
  readReply[0] = __swap_bytes(counter);
  readReply[1] = counter;
#else
  readReply[0] = 0x03;
  readReply[1] = 0x04;
#endif
  readReply[2] = queryReply[0];        // remember to restore correct RN before doing crc()
  readReply[3] = queryReply[1];        // because crc() will shift bits to add
  crc16_ccitt_readReply(2);    // leading "0" bit.
            
  //P1OUT &= ~BIT_IN_ENABLE;   // turn off comparator
  TACCTL1 &= ~CCIE;
  TAR = 0;
            
  // after that sends tagResponse
  // 16 bits for data + 16 bits for the handle + 16 bits for the CRC + leading 0 + add one to number of bits for seong's xmit code
  sendToReader(&readReply[0], 50);
  state = nextState;
  //if ( counter == 0xffff ) counter = 0; else counter++;
  delimiterNotFound = 1; // reset
#endif          
}

inline void handle_nak(volatile short nextState)
{
  TACCTL1 &= ~CCIE;
  TAR = 0;
  state = nextState;
}


//****************************** SETUP TO RECEIVE  *************************************
// note: port interrupt can also reset, but it doesn't call this function
//       because function call causes PUSH instructions prior to bit read
//       at beginning of interrupt, which screws up timing.  so, remember
//       to change things in both places.
inline void setup_to_receive()
{
  //P4OUT &= ~BIT3;
  _BIC_SR(GIE); // temporarily disable GIE so we can sleep and enable interrupts at the same time
  
  P1OUT |= BIT_IN_ENABLE;
  
  delimiterNotFound = 0;
  // setup port interrupt on pin 1.2
  P1SEL &= ~BIT2;  //Disable TimerA2, so port interrupt can be used
  // Setup timer. It has to setup because there is no setup time after done with port1 interrupt.
  TACTL = 0;
  TAR = 0;
  TACCR0 = 0xFFFF;    // Set up TimerA0 register as Max
  TACCTL0 = 0;
  TACCTL1 = SCS + CAP;   //Synchronize capture source and capture mode
  TACTL = TASSEL1 + MC1 + TAIE;  // SMCLK and continuous mode and Timer_A interrupt enabled.

  // initialize bits
  bits = 0;
  // initialize dest
  dest = destorig;  // = &cmd[0]
  // clear R6 bits of word counter from prior communications to prevent dest++ on 1st port interrupt
  asm("CLR R6");
  
  P1IE = 0;
  P1IES &= ~INPUT_PIN; // Make positive edge for port interrupt to detect start of delimiter
  P1IFG = 0;  // Clear interrupt flag
            
  P1IE  |= INPUT_PIN; // Enable Port1 interrupt
  P2IE |= (VOLTAGE_SV_PIN + VOLTAGE_SV_ALT_PIN);
  
  //if ( (P1OUT & BIT_IN_ENABLE ) != BIT_IN_ENABLE  )
  //if ( (P1IE & INPUT_PIN ) != INPUT_PIN  )
  //{
  //P4OUT |= BIT7;
  //}
  //else
  //{
  //    P4OUT &= ~BIT7;
  //}
  
  //P4OUT |= BIT7;
  //P4OUT |= BIT3;
  waitingForBits = 1;
  _BIS_SR(LPM4_bits | GIE);
  waitingForBits = 0;
  //P4OUT &= ~BIT7;
  return;
}

inline void sleep()
{

  int i;
  
  //P4OUT &= ~BIT3;
  _BIC_SR(GIE);
  
#if SIMULATE_SV_INTERRUPT
  P1IES &= ~FAKE_VOLTAGE_SV_PIN;
  P1IFG &= ~FAKE_VOLTAGE_SV_PIN;
  P1IE |= FAKE_VOLTAGE_SV_PIN;
#endif
  
  P1OUT &= ~BIT_IN_ENABLE;

#if !(SIMULATE_SV_INTERRUPT)
  // enable port interrupt for voltage supervisor
  //P2IES = 0;
  //P2IFG = 0;
  //  P2IFG = 0;
  //P2IE |= (VOLTAGE_SV_PIN);
  //P2IES |= VOLTAGE_SV_ALT_PIN;
  //P2IE |= (VOLTAGE_SV_PIN + VOLTAGE_SV_ALT_PIN);
  P2IFG &= ~(VOLTAGE_SV_PIN + VOLTAGE_SV_ALT_PIN);
  // only allow vs ints that tell me we have enough power
  P2IE &= ~VOLTAGE_SV_ALT_PIN;
  P1IE = 0;
  P1IFG = 0;
#endif
  
  TACTL = 0;
  inSleepMode = 1;
  //P4OUT |= BIT5;

  //P4OUT |= BIT3;
  
#if !SIMULATE_SV_INTERRUPT
  _BIS_SR(GIE);
  if (is_power_good()) {
    P2IFG &= ~(VOLTAGE_SV_PIN + VOLTAGE_SV_ALT_PIN);
    //P4OUT |= BIT3;
    //_BIS_SR(GIE);
    //P2IFG |= VOLTAGE_SV_PIN;
    inSleepMode = 0;
    P1IE |= INPUT_PIN;
    P1OUT |= BIT_IN_ENABLE;
    //P4OUT &= ~BIT7;
    //P4OUT |= BIT3;
    return;
  }
#endif
  
  //P4OUT |= BIT7;
  _BIS_SR(LPM4_bits | GIE);
  //P4OUT &= ~BIT7;

  //P4OUT &= ~BIT3;
  _BIC_SR(GIE);
  P2IE |= VOLTAGE_SV_ALT_PIN;
  inSleepMode = 0;
  P1IE |= INPUT_PIN;
  P1OUT |= BIT_IN_ENABLE;
  _BIS_SR(GIE);
  //P4OUT |= BIT3;
    
  return;
}

unsigned short is_power_good()
{
#if SIMULATE_SV_INTERRUPT
  if ( ++power_counter < 2) return 1;
  power_counter = 0; return 0;
#else
  return P2IN & VOLTAGE_SV_PIN;
#endif
}


//************************ PORT 2 INTERRUPT *******************************

// Pin Setup :
// Description : Port 2 interrupt wakes on power good signal from supervisor.

#if !(SIMULATE_SV_INTERRUPT)
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)   // (5-6 cycles) to enter interrupt
{
  
  // page 2-11 of the x2xx family user's guide:
  //  "Interrupt nesting is enabled if the GIE bit is set inside an interrupt
  //  service route. When interrupt nesting is enabled, any interrupt occurring
  //  during an interrupt service routine with interrupt the routine, regardless
  //  of the interrupt priorities."
  //P4OUT &= ~BIT3;
  _BIC_SR(GIE);
  //P2IE &= ~VOLTAGE_SV_ALT_PIN;

  if ( ( P2IFG & VOLTAGE_SV_PIN ) == VOLTAGE_SV_PIN )
  {
	// VS pin went from low to high --
  	if ( inSleepMode ) 
	{
		// this is an orderly wake out of sleep mode
    		P2IFG &= ~(VOLTAGE_SV_PIN + VOLTAGE_SV_ALT_PIN);
    		P1IFG = 0;
    		P1IE = 0;
    		TACTL = 0;
    		TACCTL0 = 0;
    		TACCTL1 = 0;
    		TAR = 0;
    		state = STATE_ARBITRATE;
                inSleepMode = 0;
                P1IE |= INPUT_PIN;
                P1OUT |= BIT_IN_ENABLE;
                //P4OUT &= ~BIT7;
    		LPM4_EXIT;
                // reenable interrupts
                //P4OUT |= BIT3;
                //P2IE |= VOLTAGE_SV_ALT_PIN;
                _BIS_SR(GIE);
  	}
	else 
	{
		// this is an unexpected interrupt -- we never got the chance 
		// to drop into LPM4 before running out of power. Reset to recover.
#if DEBUG_PINS_ENABLED
#if USE_2132
                P3OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT &= ~BIT5;
		P3OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P3OUT &= ~BIT5;
#else
		P4OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT &= ~BIT5;
		P4OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT |= BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT &= ~BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT &= ~BIT7; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT |= BIT5; for ( i = 0 ; i < 0xff ; i++ );
		P4OUT &= ~BIT5;
#endif
#endif
                // force a PUC
		WDTCTL = 0;
  	}
  }
  else if ( ( P2IFG & VOLTAGE_SV_ALT_PIN ) == VOLTAGE_SV_ALT_PIN )
  {
	// VS went from high to low - force us to go to sleep in main loop
    	P2IFG &= ~VOLTAGE_SV_ALT_PIN;
	delimiterNotFound = 1;
        //P4OUT |= BIT7;
        
	LPM4_EXIT;
        // GIE bit will be reenabled as a side effect of the trip
        // through the delimiterNotFound == 1 loop
        if ( waitingForBits == 1 ) {
          _BIS_SR(GIE);
        }
        
  }
  else
  {
    // I have no idea what this is, but clear it and reenabke interrupts
    P2IFG = 0;
    _BIS_SR(GIE);
  }
}
#endif

#if USE_2132
#pragma vector=TIMER0_A0_VECTOR
#else
#pragma vector=TIMERA0_VECTOR
#endif
__interrupt void TimerA0_ISR(void)   // (5-6 cycles) to enter interrupt
{
  TACTL = 0;    // have to manually clear interrupt flag
  TACCTL0 = 0;  // have to manually clear interrupt flag
  TACCTL1 = 0;  // have to manually clear interrupt flag
  LPM4_EXIT;
}

#if 0

#pragma vector=TIMERB0_VECTOR
__interrupt void TimerB0_ISR(void)   // (5-6 cycles) to enter interrupt
{
  //P4OUT |= BIT7;
  TBCTL = 0;    // have to manually clear interrupt flag
  TBCCTL0 = 0;  // have to manually clear interrupt flag
  TBCCTL1 = 0;  // have to manually clear interrupt flag
  LPM4_EXIT;
  //P4OUT &= ~BIT7;
}

#endif



//*************************************************************************
//************************ PORT 1 INTERRUPT *******************************

// warning   :  Whenever the clock frequency changes, the value of TAR should be changed in aesterick lines
// Pin Setup :  P1.2
// Description : Port 1 interrupt is used as finding delimeter.

#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)   // (5-6 cycles) to enter interrupt
{
  
#if SIMULATE_SV_INTERRUPT
  if ( (P1IFG & FAKE_VOLTAGE_SV_PIN) == FAKE_VOLTAGE_SV_PIN ) {
    P1IFG &= ~FAKE_VOLTAGE_SV_PIN;
    P1IE &= ~FAKE_VOLTAGE_SV_PIN;
    TACTL = 0;
    TACCTL0 = 0;
    TACCTL1 = 0;
    TAR = 0;
    state = STATE_RECEIVE_QUERY; // FIXME FIXME
    LPM4_EXIT;
    return;
  }
#endif  

#if USE_2132
  asm("MOV TA0R, R7");  // move TAR to R7(count) register (3 CYCLES)
#else
  asm("MOV TAR, R7");  // move TAR to R7(count) register (3 CYCLES)
#endif
  P1IFG = 0x00;       // 4 cycles
  TAR = 0;            // 4 cycles
  LPM4_EXIT;

  asm("CMP #0000h, R5\n");          // if (bits == 0) (1 cycle)
  asm("JEQ bit_Is_Zero_In_Port_Int\n");                // 2 cycles
  // bits != 0:
  asm("MOV #0000h, R5\n");          // bits = 0  (1 cycles)

  asm("CMP #0010h, R7\n");          //************ this is finding delimeter (12.5us)  (2 cycles)*********/   2d  ->   14
  asm("JNC delimiter_Value_Is_wrong\n");            //(2 cycles)
  asm("CMP #0040h, R7");             //************ this is finding delimeter (12.5us)  (2 cycles)*********  43H
  asm("JC  delimiter_Value_Is_wrong\n");
  asm("CLR P1IE");
#if USE_2132
  asm("BIS #8010h, TA0CCTL1\n");     // (5 cycles)   TACCTL1 |= CM1 + CCIE
#else
  asm("BIS #8010h, TACCTL1\n");     // (5 cycles)   TACCTL1 |= CM1 + CCIE
#endif
  asm("MOV #0004h, P1SEL\n");       // enable TimerA1    (4 cycles)
  asm("RETI\n");

  asm("delimiter_Value_Is_wrong:\n");
  asm("BIC #0004h, P1IES\n");
  asm("MOV #0000h, R5\n");          // bits = 0  (1 cycles)
  delimiterNotFound = 1;
  asm("RETI");

  asm("bit_Is_Zero_In_Port_Int:\n");                 // bits == 0
#if USE_2132
  asm("MOV #0000h, TA0R\n");     // reset timer (4 cycles)
#else
  asm("MOV #0000h, TAR\n");     // reset timer (4 cycles)
#endif
  asm("BIS #0004h, P1IES\n");   // 4 cycles  change port interrupt edge to neg
  asm("INC R5\n");            // 1 cycle
  asm("RETI\n");

}
//*************************************************************************
//************************ Timer INTERRUPT *******************************

// Pin Setup :  P1.2
// Description :

#if USE_2132
#pragma vector=TIMER0_A1_VECTOR
#else
#pragma vector=TIMERA1_VECTOR
#endif
__interrupt void TimerA1_ISR(void)   // (6 cycles) to enter interrupt
{

    asm("MOV 0174h, R7");  // move TACCR1 to R7(count) register (3 CYCLES)
    TAR = 0;               // reset timer (4 cycles)
    TACCTL1 &= ~CCIFG;      // must manually clear interrupt flag (4 cycles)

    //<--------------up to here 26 cycles + 6 cyles of Interrupt == 32 cycles ---------------->
    asm("CMP #0003h, R5\n");      // if (bits >= 3).  it will do store bits
    asm("JGE bit_Is_Over_Three\n");
    // bit is not 3
    asm("CMP #0002h, R5\n");   // if ( bits == 2)
    asm("JEQ bit_Is_Two\n");         // if (bits == 2).

    // <----------------- bit is not 2 ------------------------------->
    asm("CMP #0001h, R5\n");      // if ( bits == 1). it will measure RTcal value.
    asm("JEQ bit_Is_One\n");          // bits == 1

    // <-------------------- this is bit == 0 case --------------------->
    asm("bit_Is_Zero_In_Timer_Int:");
    asm("CLR R6\n");
    asm("INC R5\n");        // bits++
    asm("RETI");
    // <------------------- end of bit 0  --------------------------->

    // <-------------------- this is bit == 1 case --------------------->
    asm("bit_Is_One:\n");         // bits == 1.  calculate RTcal value
    asm("MOV R7, R9\n");       // 1 cycle
    asm("RRA R7\n");    // R7(count) is divided by 2.   1 cycle
    asm("MOV #0FFFFh, R8\n");   // R8(pivot) is set to max value    1 cycle
    asm("SUB R7, R8\n");        // R8(pivot) = R8(pivot) -R7(count/2) make new R8(pivot) value     1 cycle
    asm("INC R5\n");        // bits++
    asm("CLR R6\n");
    asm("RETI\n");
    // <------------------ end of bit 1 ------------------------------>

    // <-------------------- this is bit == 2 case --------------------->
    asm("bit_Is_Two:\n");
    asm("CMP R9, R7\n");    // if (count > (R9)(180)) this is hardcoded number, so have  to change to proper value
    asm("JGE this_Is_TRcal\n");
    // this is data
    asm("this_Is_Data_Bit:\n");
    asm("ADD R8, R7\n");   // count = count + pivot
    // store bit by shifting carry flag into cmd[bits]=(dest*) and increment dest*  // (5 cycles)
    asm("ADDC.b @R4+,-1(R4)\n"); // roll left (emulated by adding to itself == multiply by 2 + carry)
    // R6 lets us know when we have 8 bits, at which point we INC dest*            // (1 cycle)
    asm("INC R6\n");
    asm("CMP #0008,R6\n\n");   // undo increment of dest* (R4) until we have 8 bits
    asm("JGE out_p\n");
    asm("DEC R4\n");
    asm("out_p:\n");           // decrement R4 if we haven't gotten 16 bits yet  (3 or 4 cycles)
    asm("BIC #0008h,R6\n");   // when R6=8, this will set R6=0   (1 cycle)
    asm("INC R5\n");
    asm("RETI");
    // <------------------ end of bit 2 ------------------------------>

    asm("this_Is_TRcal:\n");
    asm("MOV R7, R5\n");    // bits = count. use bits(R5) to assign new value of TRcal
    TRcal = bits;       // assign new value     (4 cycles)
    asm("MOV #0003h, R5\n");      // bits = 3..assign 3 to bits, so it will keep track of current bits    (2 cycles)
    asm("CLR R6\n"); // (1 cycle)
    asm("RETI");

   // <------------- this is bits >= 3 case ----------------------->
    asm("bit_Is_Over_Three:\n");     // bits >= 3 , so store bits
    asm("ADD R8, R7\n");    // R7(count) = R8(pivot) + R7(count),
    // store bit by shifting carry flag into cmd[bits]=(dest*) and increment dest*  // (5 cycles)
    asm("ADDC.b @R4+,-1(R4)\n"); // roll left (emulated by adding to itself == multiply by 2 + carry)
    // R6 lets us know when we have 8 bits, at which point we INC dest*            // (1 cycle)
    asm("INC R6\n");
    asm("CMP #0008,R6\n");   // undo increment of dest* (R4) until we have 8 bits
    asm("JGE out_p1\n");
    asm("DEC R4\n");
    asm("out_p1:\n");           // decrement R4 if we haven't gotten 16 bits yet  (3 or 4 cycles)
    asm("BIC #0008h,R6\n");   // when R6=8, this will set R6=0   (1 cycle)
    asm("INC R5\n");              // bits++
    asm("RETI\n");
    // <------------------ end of bit is over 3 ------------------------------>
}



//
//
// experimental M4 code
//
//

/******************************************************************************
*   Pin Set up
*   P1.1 - communication output
*******************************************************************************/
void sendToReader(volatile unsigned char *data, unsigned char numOfBits)
{

  SEND_CLOCK;

  TACTL &= ~TAIE;
  TAR = 0;
  // assign data address to dest
  dest = data;
  // Setup timer
  P1SEL |= OUTPUT_PIN; //  select TIMER_A0
  P1DIR |= OUTPUT_PIN;
  TACTL |= TACLR;   //reset timer A
  TACTL = TASSEL1 + MC0;     // up mode

  TACCR0 = 5;  // this is 1 us period( 3 is 430x12x1)

  TAR = 0;
  TACCTL0 = OUTMOD2; // RESET MODE

#if MILLER_4_ENCODING
  BCSCTL2 |= DIVM_1;
#endif

  //TACTL |= TASSEL1 + MC1 + TAIE;
  //TACCTL1 |= SCS + CAP;	//initially, it set up as capturing rising edge.

/**************************************************************************************************
*   The starting of the transmitting code. Transmitting code must send 4 or 16 of M/LF, then send
*   010111 preamble before sending data package. TRext determines how many M/LFs are sent.
*
*   Used Register
*   R4 = CMD address, R5 = bits, R6 = counting 16 bits, R7 = 1 Word data, R9 = temp value for loop
*   R10 = temp value for the loop, R13 = 16 bits compare, R14 = timer_value for 11, R15 = timer_value for 5
***************************************************************************************************/


  //<-------------- The below code will initiate some set up ---------------------->//
    //asm("MOV #05h, R14");
    //asm("MOV #02h, R15");
    bits = TRext;       // 6 cycles
    asm("CMP #0001h, R5");  // 1 cycles
    asm("JEQ TRextIs_1");   // 2 cycles
    asm("MOV #0004h, R9");   // 1 cycles
    asm("JMP otherSetup");   // 2 cycles

    // initialize loop for 16 M/LF
    asm("TRextIs_1:");
    asm("MOV #000fh, R9");    // 2 cycles    *** this will chagne to right value
    asm("NOP");

    //
    asm("otherSetup:");
    bits = numOfBits;                // (3 cycles).  This value will be adjusted. if numOfBit is constant, it takes 2 cycles
    asm("MOV #0bh, R14");     // (2 cycles) R14 is used as timer value 11, it will be 2 us in 6 MHz
    asm("MOV #05h, R15");      // (2 cycles) R15 is used as tiemr value 5, it will be 1 us in 6 MHz
    asm("MOV @R4+, R7");      // (2 cycles) Assign data to R7
    asm("MOV #0010h, R13");   // (2 cycles) Assign decimal 16 to R13, so it will reduce the 1 cycle from below code
    asm("MOV R13, R6");       // (1 cycle)
    asm("SWPB R7");           // (1 cycle)    Swap Hi-byte and Low byte
    asm("NOP");
    asm("NOP");
    // new timing needs 11 cycles
    asm("NOP");
    //asm("NOP");       // up to here, it make 1 to 0 transition.
    //<----------------1 us --------------------------------
    //asm("NOP");   // 1
    //asm("NOP");   // 2
    //asm("NOP");   // 3
    //asm("NOP");   // 4
    //asm("NOP");   // 5
    //asm("NOP");   // 6
    //asm("NOP");   // 7
    //asm("NOP");   // 8
    //asm("NOP");   // 9
    // <---------- End of 1 us ------------------------------
    // The below code will create the number of M/LF.  According to the spec,
    // if the TRext is 0, there are 4 M/LF.  If the TRext is 1, there are 16 M/LF
    // The upper code executed 1 M/LF, so the count(R9) should be number of M/LF - 1
    //asm("MOV #000fh, R9");    // 2 cycles    *** this will chagne to right value
    asm("MOV #0001h, R10");   // 1 cycles
    // The below code will create the number base encoding waveform., so the number of count(R9) should be times of M
    // For example, if M = 2 and TRext are 1(16, the number of count should be 32.
    asm("M_LF_Count:");
    asm("NOP");   // 1
    asm("NOP");   // 2
    asm("NOP");   // 3
    asm("NOP");   // 4
    asm("NOP");   // 5
    asm("NOP");   // 6
    asm("NOP");   // 7
    asm("NOP");   // 8
    asm("NOP");   // 9
    asm("NOP");   // 10
    asm("NOP");   // 11
    asm("NOP");   // 12
    asm("NOP");   // 13
    asm("NOP");   // 14
    asm("NOP");   // 15
    asm("NOP");   // 16
    // asm("NOP");   // 17

    asm("CMP R10, R9");       // 1 cycle
    asm("JEQ M_LF_Count_End"); // 2 cycles
    asm("INC R10");           // 1 cycle
    asm("NOP");   // 22
    asm("JMP M_LF_Count");      // 2 cycles

    asm("M_LF_Count_End:");
    // this code is preamble for 010111 , but for the loop, it will only send 01011
    asm("MOV #5c00h, R9");      // 2 cycles
    asm("MOV #0006h, R10");     // 2 cycles
    // this should be counted as 0. Therefore, Assembly DEC line should be 1 after executing
    asm("Preamble_Loop:");
    asm("DEC R10");               // 1 cycle
    asm("JZ last_preamble_set");          // 2 cycle
    asm("RLC R9");                // 1 cycle
    asm("JNC preamble_Zero");     // 2 cycle      .. up to 6
    // this is 1 case for preamble
    asm("NOP");
#if USE_2132
    asm("MOV R14, TA0CCR0");       // 4 cycle      .. 10
#else
    asm("MOV R14, TACCR0");       // 4 cycle      .. 10
#endif
    asm("NOP");
    asm("NOP");
    asm("NOP");
#if USE_2132
    asm("MOV R15, TA0CCR0");       // 4 cycle      .. 19
#else
    asm("MOV R15, TACCR0");       // 4 cycle      .. 19
#endif
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");                   // .. 22
    asm("JMP Preamble_Loop");     // 2 cycles   .. 24

    // this is 0 case for preamble
    asm("preamble_Zero:");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");


    asm("JMP Preamble_Loop");     // 2 cycles .. 24

    asm("last_preamble_set:");
    asm("NOP");			// 4
    asm("NOP");
    asm("NOP");    // TURN ON
    asm("NOP");
#if USE_2132
    asm("MOV.B R14, TA0CCR0");// 4 cycles
#else
    asm("MOV.B R14, TACCR0");// 4 cycles
#endif
    asm("NOP");
    asm("NOP");
    asm("NOP");
#if USE_2132
    asm("MOV.B R15, TA0CCR0");
#else
    asm("MOV.B R15, TACCR0");
#endif
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    //asm("NOP");
    //<------------- end of initial set up

/***********************************************************************
*   The main loop code for transmitting data in 6 MHz.  This will transmit data in real time.
*   R5(bits) and R6(word count) must be 1 bigger than desired value.
*   Ex) if you want to send 16 bits, you have to store 17 to R5.
************************************************************************/

    // this is starting of loop
    asm("LOOPAGAIN:");
    asm("DEC R5");                              // 1 cycle
    asm("JEQ Three_Cycle_Loop_End");                // 2 cycle
    //<--------------loop condition ------------
    asm("NOP");                                 // 1 cycle
    asm("RLC R7");                              // 1 cycle
    asm("JNC bit_is_zero");	                // 2 cycles  ..7

    // bit is 1
    asm("bit_is_one:");
#if USE_2132
    asm("MOV R14, TA0CCR0");                   // 4 cycles   ..11
#else
    asm("MOV R14, TACCR0");                   // 4 cycles   ..11
#endif                // 4 cycles   ..11
    asm("DEC R6");                              // 1 cycle  ..12
    asm("JNZ bit_Count_Is_Not_16");              // 2 cycle    .. 14
    // This code will assign new data from reply and then swap bytes.  After that, update R6 with 16 bits
    //asm("MOV @R4+, R7");
#if USE_2132
    asm("MOV R15, TA0CCR0");                   // 4 cycles   .. 20
#else
    asm("MOV R15, TACCR0");                   // 4 cycles   .. 20
#endif
    asm("MOV R13, R6");                         // 1 cycle    .. 22
    //asm("MOV R15, TACCR0");                   // 4 cycles   .. 20
    asm("MOV @R4+, R7");

    asm("SWPB R7");                             // 1 cycle    .. 21
    //asm("MOV R13, R6");                         // 1 cycle    .. 22
    // End of assigning data byte
    asm("JMP LOOPAGAIN");                       // 2 cycle    .. 24

    asm("seq_zero:");
    asm("NOP");                         // 1 cycle   .. 3
#if USE_2132
    asm("MOV R15, TA0CCR0");         // 4 cycles       ..7
#else
    asm("MOV R15, TACCR0");         // 4 cycles       ..7
#endif

    // bit is 0, so it will check that next bit is 0 or not
    asm("bit_is_zero:");				// up to 7 cycles
    asm("DEC R6");                      // 1 cycle   .. 8
    asm("JNE bit_Count_Is_Not_16_From0");           // 2 cycles  .. 10
    // bit count is 16
    asm("DEC R5");                      // 1 cycle   .. 11
    asm("JEQ Thirteen_Cycle_Loop_End");     // 2 cycle   .. 13
    // This code will assign new data from reply and then swap bytes.  After that, update R6 with 16 bits
    asm("MOV @R4+,R7");                 // 2 cycles     15
    asm("SWPB R7");                     // 1 cycle      16
    asm("MOV R13, R6");                 // 1 cycles     17
    // End of assigning new data byte
    asm("RLC R7");		        // 1 cycles     18
    asm("JC nextBitIs1");	        // 2 cycles  .. 20
    // bit is 0
#if USE_2132
    asm("MOV R14, TA0CCR0");             // 4 cycles  .. 24
#else
    asm("MOV R14, TACCR0");             // 4 cycles  .. 24
#endif
    // Next bit is 0 , it is 00 case	
    asm("JMP seq_zero");

// <---------this code is 00 case with no 16 bits.
    asm("bit_Count_Is_Not_16_From0:");                  // up to 10 cycles
    asm("DEC R5");                          // 1 cycle      11
    asm("JEQ Thirteen_Cycle_Loop_End");         // 2 cycle    ..13
    asm("NOP");         	            // 1 cycles    ..14
    asm("NOP");                             // 1 cycles    ..15
    asm("NOP");                             // 1 cycles    ..16
    asm("NOP");                             // 1 cycles    ..17
    asm("RLC R7");	                    // 1 cycle     .. 18
    asm("JC nextBitIs1");	            // 2 cycles    ..20
#if USE_2132
    asm("MOV R14, TA0CCR0");               // 4 cycles   .. 24
#else
    asm("MOV R14, TACCR0");               // 4 cycles   .. 24
#endif
    asm("JMP seq_zero");        // 2 cycles    .. 2

// whenever current bit is 0, then next bit is 1
    asm("nextBitIs1:");     // 20
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");       // 24

    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("JMP bit_is_one");  // end of bit 0 .. 7

    asm("bit_Count_Is_Not_16:");       // up to here 14
    asm("NOP");
#if USE_2132
    asm("MOV R15, TA0CCR0");             // 4 cycles   .. 20
#else
    asm("MOV R15, TACCR0");             // 4 cycles   .. 20
#endif
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("JMP LOOPAGAIN");     // 2 cycle          .. 24

    // below code is the end of loop code
    asm("Three_Cycle_Loop_End:");
    asm("JMP lastBit");     // 2 cycles   .. 5

    asm("Thirteen_Cycle_Loop_End:");
    asm("NOP");   // 1
    asm("NOP");   // 2
    asm("NOP");   // 3
    asm("NOP");   // 4
    asm("NOP");   // 5
    asm("NOP");   // 6
    asm("NOP");   // 7
    asm("NOP");   // 8
    asm("NOP");   // 9
    asm("NOP");   // 10
    asm("NOP");   // 11 ..24
    asm("NOP");   // 12
    asm("NOP");   // 13
    asm("NOP");   // 14
    asm("JMP lastBit");
/***********************************************************************
*   End of main loop
************************************************************************/
// this is last data 1 bit which is dummy data
    asm("lastBit:");
    asm("NOP");
    asm("NOP");
#if USE_2132
    asm("MOV.B R14, TA0CCR0");// 4 cycles
#else
    asm("MOV.B R14, TACCR0");// 4 cycles
#endif
    asm("NOP");
    asm("NOP");
    asm("NOP");
#if USE_2132
    asm("MOV.B R15, TA0CCR0");
#else
    asm("MOV.B R15, TACCR0");
#endif
    asm("NOP");
    asm("NOP");
    // experiment

    asm("NOP");

    //TACCR0 = 0;

    TACCTL0 = 0;  // DON'T NEED THIS NOP
    RECEIVE_CLOCK;

}




/**
 * This code comes from the Open Tag Systems Protocol Reference Guide version 1.1
 * dated 3/23/2004. (http://www.opentagsystems.com/pdfs/downloads/OTS_Protocol_v11.pdf)
 * No licensing information accompanied the code snippet.
 **/
unsigned short crc16_ccitt(volatile unsigned char *data, unsigned short n) {
  register unsigned short i, j;
  register unsigned short crc_16;

  crc_16 = 0xFFFF; // Equivalent Preset to 0x1D0F
  for (i=0; i<n; i++) {
    crc_16^=data[i] << 8;
    for (j=0;j<8;j++) {
      if (crc_16&0x8000) {
        crc_16 <<= 1;
        crc_16 ^= 0x1021; // (CCITT) x16 + x12 + x5 + 1
      }
      else {
        crc_16 <<= 1;
      }
    }
  }
  return(crc_16^0xffff);
}

inline void crc16_ccitt_readReply(unsigned int numDataBytes)
{
  
  // shift everything over by 1 to accomodate leading "0" bit.
  // first, grab address of beginning of array
  readReply[numDataBytes + 2] = 0; // clear out this spot for the loner bit of handle
  readReply[numDataBytes + 4] = 0; // clear out this spot for the loner bit of crc
  bits = (unsigned short) &readReply[0];
  // shift all bytes and later use only data + handle
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");
  asm("RRC.b @R5+");  
  // store loner bit in array[numDataBytes+2] position
  asm("RRC.b @R5+");
  // make first bit 0
  readReply[0] &= 0x7f;
  
  // compute crc on data + handle bytes
  readReplyCRC = crc16_ccitt(&readReply[0], numDataBytes + 2);
  readReply[numDataBytes + 4] = readReply[numDataBytes + 2];
  // XOR the MSB of CRC with loner bit.
  readReply[numDataBytes + 4] ^= __swap_bytes(readReplyCRC); // XOR happens with MSB of lower nibble
  // Just take the resulting bit, not the whole byte
  readReply[numDataBytes + 4] &= 0x80;
  
  unsigned short mask = __swap_bytes(readReply[numDataBytes + 4]);
  mask >>= 3;
  mask |= (mask >> 7);
  mask ^= 0x1020;
  mask >>= 1;  // this is because the loner bit pushes the CRC to the left by 1
  // but we don't shift the crc because it should get pushed out by 1 anyway
  readReplyCRC ^= mask;
  
  readReply[numDataBytes + 3] = (unsigned char) readReplyCRC;
  readReply[numDataBytes + 2] |= (unsigned char) (__swap_bytes(readReplyCRC) & 0x7F);
}

unsigned char crc5(volatile unsigned char *buf, unsigned short numOfBits)
{
  register unsigned char shift;
  register unsigned char data, val;
  register unsigned short i;
  shift = 0x48;
  for (i = 0; i < numOfBits; i++)
  {
    if ( (i%8) == 0)
      data = *buf++;
    val = shift ^ data;
    shift = shift << 1;
    data = data << 1;
    if (val&0x80)
      shift = shift ^ POLY5;
  }
  shift = shift >>3;
  return (unsigned char)(shift);

}

#if 1


void lfsr()
{ 
    // calculate LFSR
    rn16 = (rn16 << 1) | (((rn16 >> 15) ^ (rn16 >> 13) ^ (rn16 >> 9) ^ (rn16 >> 8)) & 1);
    rn16 = rn16 & 0xFFFF;
    
    // fit 2^Q-1
    rn16 = rn16>>(15-Q);
}

inline void loadRN16()
{
#if 1
  if (Q > 8)
  {
    queryReply[0] = RN16[(Q<<1)-9];
    queryReply[1] = RN16[(Q<<1)-8];
  }
  else
  {
    int index = ((Q+shift) & 0xF);
    queryReply[0] = RN16[index];
    queryReply[1] = RN16[index+1];
  }
#else
    queryReply[0] = 0xf0;
    queryReply[1] = 0x0f;
#endif
  
}

inline void mixupRN16()
{
  unsigned short tmp;
  unsigned short newQ = 0;
  unsigned short swapee_index = 0;
  
  newQ = RN16[shift] & 0xF;
  swapee_index = RN16[newQ];
  tmp = RN16[shift];
  RN16[Q] = RN16[swapee_index];
  RN16[swapee_index] = tmp;  
}


#endif


