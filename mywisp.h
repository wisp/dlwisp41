/* See license.txt for license information. */

#ifndef MYWISP_H
#define MYWISP_H

/*
 * This file provides access to WISP application settings. It is intended to be
 * modified by the user as per the step-by-step instructions given below.
 */

////////////////////////////////////////////////////////////////////////////////
// Step 1: Pick an application mode
//
// 1(a): Only one of the four application modes below should be selected (set
//        to 1). Make sure all the others are disabled (set to 0).
//
// Simple hardcoded query-ack
#define SIMPLE_QUERY_ACK              0
//
// Return sampled sensor data as EPC. Best for longer range sensing.
#define SENSOR_DATA_IN_ID             1
//
// Support read commands. Returns one word of counter data
#define SIMPLE_READ_COMMAND           0
//
// Return sampled sensor data in a read command. Returns three words of accel
// data
#define SENSOR_DATA_IN_READ_COMMAND   0
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Step 2: Pick a sensor type
//
// NOTE:  If you're not using the SENSOR_DATA_IN_ID or 
//        SENSOR_DATA_IN_READ_COMMAND apps, then this is a don't-care.
//
// 2(a) Determine which of the following sensor choices you would like to use:
//
// SENSOR_NULL(0x0C)
//  static data - for test purposes only.
#define SENSOR_NULL                   0
//
// SENSOR_ACCEL(0x0D)
//  3-axis accelerometer sampled with 10-bit ADC. Allows for
//  full RC settling time. More precision, but lower read rate.
#define SENSOR_ACCEL                  1
//
// SENSOR_ACCEL_QUICK(0x0B)
//  3-axis accelerometer sampled with 10-bit ADC, partial RC settling ("quick").
//  Less power, faster read rate.
#define SENSOR_ACCEL_QUICK            2
//
// SENSOR_INTERNAL_TEMP(0x0F)
//  Built-in temperature sensor sampled with a 10-bit ADC
#define SENSOR_INTERNAL_TEMP          3
//
// SENSOR_EXTERNAL_TEMP(0x0E)
//  External temperature sensor sampled with a 10-bit ADC
#define SENSOR_EXTERNAL_TEMP          4
//
// SENSOR_COMM_STATS(0x0A)
//  [Not implemented] Get information about the communication link.
#define SENSOR_COMM_STATS             5
//
// 2(b) Change the value of ACTIVE_SENSOR to the desired sensor title 
//      from the list above:
#define ACTIVE_SENSOR                 SENSOR_ACCEL_QUICK
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Step 3: Pick a reader and wisp hardware
// make sure this syncs with project target
#define BLUE_WISP                     0x41
#define WISP_VERSION                  BLUE_WISP
#define IMPINJ_READER                 1
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Step 4: Select desired protocol features
//
// The spec actually requires all these features, but as a practical matter
// supporting things like slotting and sessions requires extra power and thus
// limits range. Another factor is if you are running out of room on flash --
// e.g., you're using the flash-limited free IAR kickstart compiler -- you
// probably want to leave these features out unless you really need them.
//
// You only need ENABLE_SLOTS when you're working with more than one WISP. The
// code will run fine without ENABLE_SLOTS if you're using one WISP with
// multiple non-WISP rfid tags.
//
// ENABLE_SESSIONS is new code that hasn't been tested in a multiple reader
// environment. Also note a workaround I use in handle_queryrep to deal with
// what appears to be unexpected session values in the reader I'm using.
//
// Known issue: ENABLE_SLOTS and ENABLE_SESSIONS won't work together;
// it's probably just a matter of finding the right reply timing in handle_query
// 
// 4(a) Enable the desired protocol features (set to 1) below. For best
//      performance of this WISP, disable all features below.
//
#define ENABLE_SLOTS                  0
#define ENABLE_SESSIONS               0
#define ENABLE_HANDLE_CHECKING        0 // [NOT IMPLEMENTED]
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Step 5: set EPC and TID identifiers (optional)
////////////////////////////////////////////////////////////////////////////////
//
// 5(a) Select an ID for your WISP. If using a numbered WISP, 
//      enter that number here. The high/low byte will be computed by the
//      preprocessor.
#define WISP_EZ_ID 321
//
// 5(b) If desired, customize the first 9 bytes of the EPC ("User Field"):
#define EPC_USER_FIELD 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Step 6: pick either Miller-2 or Miller-4 encoding
////////////////////////////////////////////////////////////////////////////////
// 6(a) Select a baseband coding method for T->R communications.
//      Miller 4 is recommended!
//
#define MILLER_2_ENCODING 0 // [NOT TESTED ... use at your own risk]
#define MILLER_4_ENCODING 1
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Step 7: Enable or disable DEBUG mode(s)
////////////////////////////////////////////////////////////////////////////////
// 7(a) If you happen to have a WISP Monitor device, you can enable this mode
//      for simple debugging of WISP operation. If you don't, disable this mode.
//
#define DEBUG_PINS_ENABLED            0
//
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// --Your WISP is now configured!--
//
// Nothing below this point needs to be modified by the user. 
////////////////////////////////////////////////////////////////////////////////

// WISP ID high/low byte computation (preprocessor)
#define WISP_ID WISP_EZ_ID/256, WISP_EZ_ID%256

// EPC sequencing
#define EPC EPC_USER_FIELD, WISP_VERSION, WISP_ID

// Other WISP info
#define TID_DESIGNER_ID_AND_MODEL_NUMBER 0xFF, 0xF0, 0x01

// This section will produce compile warnings indicating which application
// is in use. These warnings are perfectly normal, no need to be alarmed!
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

// Conditional include of sensor files. Not a pleasant structure, but it works!
#if READ_SENSOR
  #if (ACTIVE_SENSOR == SENSOR_ACCEL_QUICK)
    #include "quick_accel_sensor.h"
  #elif (ACTIVE_SENSOR == SENSOR_ACCEL)
    #include "accel_sensor.h"
  #elif (ACTIVE_SENSOR == SENSOR_INTERNAL_TEMP)
    #include "int_temp_sensor.h"
  #elif (ACTIVE_SENSOR == SENSOR_EXTERNAL_TEMP)
    #error "SENSOR_EXTERNAL_TEMP not yet implemented"
  #elif (ACTIVE_SENSOR == SENSOR_NULL)
    #include "null_sensor.h"
  #elif (ACTIVE_SENSOR == SENSOR_COMM_STATS)
    #error "SENSOR_COMM_STATS not yet implemented"
  #endif
  extern unsigned char sensor_busy;
  void init_sensor();
  void read_sensor(unsigned char volatile *target);
#endif

#endif // MYWISP_H
