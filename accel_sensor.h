/* See license.txt for license information. */

#ifndef ACCEL_SENSOR_H
#define ACCEL_SENSOR_H

// these bit definitions are specific to WISP 4.1 DL

#define SENSOR_DATA_TYPE_ID       0x0D

#define ACCEL_ENABLE_BIT          BIT5   // 1.5
#define SET_ACCEL_ENABLE_DIR      P1DIR |= ACCEL_ENABLE_BIT
#define CLEAR_ACCEL_ENABLE_DIR    P1DIR &= ~ACCEL_ENABLE_BIT
#define TURN_ON_ACCEL_ENABLE      P1OUT |= ACCEL_ENABLE_BIT
#define TURN_OFF_ACCEL_ENABLE     P1OUT &= ~ACCEL_ENABLE_BIT

#define X_INCH                    INCH_2  // A2
#define Y_INCH                    INCH_1  // A1
#define Z_INCH                    INCH_0  // A0

#define DATA_LENGTH_IN_WORDS      3
#define DATA_LENGTH_IN_BYTES      (DATA_LENGTH_IN_WORDS*2)

#define CHECK_FOR_GOOD_VOLTAGE    0
#define DEBUG_BAD_SAMPLES         0
#if DEBUG_BAD_SAMPLES
extern short lastx, lasty, lastz;
extern short x, y, z;
extern short diff;
#endif

#endif // ACCEL_SENSOR_H
