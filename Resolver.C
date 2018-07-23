// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Resolver.c
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "RS232.h"
#include "Rs232Out.h"
#include "Timer0.h"
#include "TaskMgr.h"
#include "stdbool.h"            // needed for bool data types
#include "CPLD.H"
#include "CanOpen.H"
#include "DigIO.H"
#include "TaskMgr.h"


Uint16 res_FixedShaftAngle;
Uint16 res_attenuation;               // 1-> sin & cos are attenuated 50% vs Resolver Reference
                                      // 0-> no attenuation
                                      // IMPORTANT: in all likelyhood, CAN Command 0x200C.06 will have no effect on the TS3
                                      // Resolver Simulator.  It will probably be hard-coded to always apply 50% attenuation
                                      // from the resolver reference signal to the output Sin & Cos signals,
                                      // which it does by merely right-shifting by 1 bit the DAC values for Sin and Cos
                                      // before writing to the multiplying DAC (multiplies x resolver reference signal.)

Uint16 res_velocity;                  // 0-> no constant-velocity rotation
                                      // <other> -> increment res_hi_precis_shaft_angle by res_velocity each 200uSec

Uint16 res_HiPrecisShaftAngle;        // 360/65536 degrees

Uint16 res_AmpRefInOut;    // ls bit indicates whether Amplifier Reference signal is
                           //  internal=0 or exrternal=1
                           // IMPORTANT: in the Manual Test PC software the sense of the "Internal" and "External" radio buttons,
                           // appears to be from the point of view of the Amplifier-under-test (as opposed to being from the
                           // point of view of the test station.  EG: selecting "External" enables the Test Station's internal
                           // Resolver Reference circuitry, whereas "Internal" disables the Test Station's internal Resolver
                           // Reference circuitry, so that the Test Station uses the Amplifier's "internal" Resolver Reference signal.

                           // This feature is only for compatibility w/ classic Test Station commands:
                           // TS3 hardware does not provide capability to generate a resolver reference signal
                           // to send to the amplifier-under-test since all AMC resolver
                           // drives provide their own "internal" reference signal which
                           // they send to the test station.

Uint16 res_RefFreq;
                           // This feature is only for compatibility w/ classic Test Station commands:
                           // TS3 hardware does not provide an "internal" reference source
                           // since all AMC resolver drives provide an "external" reference signal.

Uint16 res_RefDacValue;
                           // This feature is only for compatibility w/ classic Test Station commands:
                           // TS3 hardware does not provide an "internal" reference source
                           // since all AMC resolver drives provide an "external" reference signal.

void res_init(void){
	res_velocity = 0;
	res_HiPrecisShaftAngle = 0;
	res_attenuation = 1;
	res_FixedShaftAngle = 0;
	res_RefFreq = 0;
	res_RefDacValue = 0;
}
//===========================================================================
// Algorithms to take shaft angle and generate values for Sin and cos DACs
// were lifted from Classic test station firmware and modified for TS3 hardware.
//===========================================================================

// data is a value from 0 to 65535 representing a resolver shaft angle
//  in units of 360/65536 degrees
// Use a table to compute Sin and Cos values to write to DACs.
// Increase precision of DAC outputs via linear interpolation.
// Table values computed as ROUND(Sin(N * ((PI( ) / 2) / 256)) * 2047,0) as "N" varies from 0 to 257 by 1.
// Redundant Table entry for N = 257 allows easy fetch of (n+1) table
//  value for purposes of interpolation
void res_calcSinCosFromHiPrecisShaftAngle(Uint16 data,Uint16* sin_ret,Uint16* cos_ret){
// Given data = res_HiPrecisShaftAngle
// Return sin_ret;  // DAC value for 1st appx for sin amplitude for <data>
// Return cos_ret;  // DAC value for 1st appx for cos amplitude for <data>

   Uint16 cos;
   Uint16 sin;
   Uint16 n;    // index into sine_tbl
   Uint16 remainder; // 0 to 255 -- LS bits truncated when we >>8 to get table index

   static Uint16 sine_tbl[] = {
   0,13,25,38,50,63,75,88,100,113,
   126,138,151,163,176,188,201,213,226,238,
   251,263,275,288,300,313,325,338,350,362,
   375,387,399,412,424,436,449,461,473,485,
   497,510,522,534,546,558,570,582,594,606,
   618,630,642,654,666,678,690,701,713,725,
   737,748,760,772,783,795,807,818,830,841,
   852,864,875,887,898,909,920,932,943,954,
   965,976,987,998,1009,1020,1031,1042,1052,1063,
   1074,1085,1095,1106,1116,1127,1137,1148,1158,1168,
   1179,1189,1199,1209,1219,1229,1239,1249,1259,1269,
   1279,1289,1299,1308,1318,1328,1337,1347,1356,1365,
   1375,1384,1393,1402,1411,1421,1430,1439,1447,1456,
   1465,1474,1483,1491,1500,1508,1517,1525,1533,1542,
   1550,1558,1566,1574,1582,1590,1598,1606,1614,1621,
   1629,1637,1644,1652,1659,1666,1674,1681,1688,1695,
   1702,1709,1716,1723,1729,1736,1743,1749,1756,1762,
   1769,1775,1781,1787,1793,1799,1805,1811,1817,1823,
   1828,1834,1840,1845,1850,1856,1861,1866,1871,1876,
   1881,1886,1891,1896,1901,1905,1910,1914,1919,1923,
   1927,1932,1936,1940,1944,1948,1951,1955,1959,1962,
   1966,1969,1973,1976,1979,1983,1986,1989,1992,1994,
   1997,2000,2003,2005,2008,2010,2012,2015,2017,2019,
   2021,2023,2025,2027,2028,2030,2032,2033,2035,2036,
   2037,2038,2039,2040,2041,2042,2043,2044,2045,2045,
   2046,2046,2046,2047,2047,2047,2047,
   2047};  // redundant (n=257) table entry

   // For each of 4 quadrants: 0x0000 -- 0x4000 -- 0x8000 -- 0xFFFF
   // Compute n, index into sine-tbl -- different for sine & cosine, depending on quadrant
   // Remainder = LS 6 bits of number, discarded when >>6 to compute table index
   // Fetch value from sine_tbl(n) and sin_tbl(n+1)
   // Interpolate between the two: X(n) + [{X(n+1) - X(n)}*remainder/64]
   // Add, subtract, or subtarct-from 0x800 as appropriate for on quadrant
   if (data < 0x4000)
   {
      n = data>>6;
      remainder = data & 0x003f;
      sin = sine_tbl[n] + 0x800 + (((sine_tbl[n+1] - sine_tbl[n]) * remainder)>>6);
      n = (0x4000 - data)>>6;
      remainder = (0x4000 - data) & 0x003f;
      cos = sine_tbl[n] + 0x800 + (((sine_tbl[n+1] - sine_tbl[n]) * remainder)>>6);
   }
   else if (data < 0x8000)
   {
      n = (0x8000 - data)>>6;
      remainder = (0x8000 - data) & 0x003f;
      sin = sine_tbl[n] + 0x800 + (((sine_tbl[n+1] - sine_tbl[n]) * remainder)>>6);
      n = (data - 0x4000)>>6;
      remainder = (data - 0x4000) & 0x003f;
      cos = 0x800 - (sine_tbl[n] + (((sine_tbl[n+1] - sine_tbl[n]) * remainder)>>6));
   }
   else if (data < 0xC000)
   {
      n = (data - 0x8000)>>6;
      remainder = (data - 0x8000) & 0x003f;
      sin = 0x800 -  (sine_tbl[n] + (((sine_tbl[n+1] - sine_tbl[n]) * remainder)>>6));
      n = (0xC000 - data)>>6;
      remainder = (0xC000 - data) & 0x003f;
      cos = 0x800 - (sine_tbl[n] + (((sine_tbl[n+1] - sine_tbl[n]) * remainder)>>6));
   }
   else
   {
      n = ((0xFFFF ^ data) + 1)>>6;
      remainder = ((0xFFFF ^ data) + 1) & 0x003f;
      sin = 0x800 -  (sine_tbl[n] + (((sine_tbl[n+1] - sine_tbl[n]) * remainder)>>6));
      n = (data - 0xC000)>>6;
      remainder = (data - 0xC000) & 0x003f;
      cos = sine_tbl[n] + 0x800 + (((sine_tbl[n+1] - sine_tbl[n]) * remainder)>>6);
   }

   // algorithm from classic test station (code above) produces result that is sign-reversed
   // from what's needed for multiplying DAC in TS3.
   // So here we reverse the signs to work with TS3 DAC.
   cos = 0xFFF - cos;
   sin = 0xFFF - sin;

   //digio_writeDacOutputValue(FPGA1_WRITE_DAC_RES_COS, cos); // Cos
   //digio_writeDacOutputValue(FPGA1_WRITE_DAC_RES_SIN, sin); // Sin

   // return sin & cos values, used to output them here, generalized to support both SSEnc and RES
   *sin_ret = sin;
   *cos_ret = cos;

}

// data is a value from 0 to 360 representing a resolver shaft angle.
// Use a table to compute Sin and Cos values and write them to DACs.
// Table values computed as (Sin(N) * 2047), with N as integer degrees from 0 to 90
void write_RES_Shaft_Angle(Uint16 data)
{
   Uint16 sin;
   Uint16 cos;
   static Uint16 sine_tbl[] = {
      0,   36,   71,  107,  143,  178,  214,  249,  285,  320,
    355,  391,  426,  460,  495,  530,  564,  598,  633,  666,
    700,  734,  767,  800,  833,  865,  897,  929,  961,  992,
   1024, 1054, 1085, 1115, 1145, 1174, 1203, 1232, 1260, 1288,
   1316, 1343, 1370, 1396, 1422, 1447, 1472, 1497, 1521, 1545,
   1568, 1591, 1613, 1635, 1656, 1677, 1697, 1717, 1736, 1755,
   1773, 1790, 1807, 1824, 1840, 1855, 1870, 1884, 1898, 1911,
   1924, 1935, 1947, 1958, 1968, 1977, 1986, 1995, 2002, 2009,
   2016, 2022, 2027, 2032, 2036, 2039, 2042, 2044, 2046, 2047,
   2047};

   if (data >360)   // Bad data, just ignore it
      return;

   res_FixedShaftAngle = data;
   if (data < 90)
   {
      sin = sine_tbl[data] + 0x800;
      cos = sine_tbl[90 - data] + 0x800;
   }
   else if (data < 180)
   {
      sin = sine_tbl[180 - data] + 0x800;
      cos = 0x800 - sine_tbl[data - 90];
   }
   else if (data < 270)
   {
      sin = 0x800 - sine_tbl[data - 180];
      cos = 0x800 - sine_tbl[270 - data];
   }
   else if (data < 361)
   {
      sin = 0x800 - sine_tbl[360 - data];
      cos = sine_tbl[data - 270] + 0x800;
   }

   //if (res_attenuation != 0)
   //{
   //  // resolver sin & cos out are attenuated 50% compared to resolver reference
   //   sin = ((sin - 0x800) >> 1) + 0x800;
   //   cos = ((cos - 0x800) >> 1) + 0x800;
   //}

   // algorithm from classic test station (code above) produces result that is sign-reversed
   // from what's needed for multiplying DAC in TS3.
   // So here we reverse the signs to work with TS3 DAC.
   cos = 0xFFF - cos;
   sin = 0xFFF - sin;

   digio_writeDacOutputValue(FPGA1_WRITE_DAC_RES_COS, cos); // Cos
   digio_writeDacOutputValue(FPGA1_WRITE_DAC_RES_SIN, sin); // Sin
}

//===========================================================================
// Routines run to service CAN commands
//
//===========================================================================

enum CANOPEN_STATUS res_recvAmpRef(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received data from host, store in memory at dest = (Uint16*)can_command->datapointer
	// And act on it.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *dest;
	Uint16 switchValue;
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC
	// MboxD ignored

	// Act on received data . . .
	// ls bit (of MboxC, saved in res_amp_ref_in_out) indicates whether Amplifier Reference signal is
	//  internal=0 or external=1

    // IMPORTANT: in the Manual Test PC software the sense of the "Internal" and "External" radio buttons,
    // appears to be from the point of view of the Amplifier-under-test (as opposed to being from the
    // point of view of the test station.  EG: selecting "External" enables the Test Station's internal
    // Resolver Reference circuitry, whereas "Internal" disables the Test Station's internal Resolver
    // Reference circuitry, so that the Test Station uses the Amplifier's "internal" Resolver Reference signal.

    // This feature is only for compatibility w/ classic Test Station commands:
    // TS3 hardware does not provide capability to generate a resolver reference signal
    // to send to the amplifier-under-test since all AMC resolver
    // drives provide their own "internal" reference signal which
    // they send to the test station.

	// We write to FPGA2 to Open (external=1), or Close (internal=0) the Res_Ref_In_Fm_Pin_SW switch
	switchValue = *CPLD_F2_XA(FPGA2_READ_IO_PIN_SWITCHES);
	if (res_AmpRefInOut == 1) {
		switchValue &= 0xFFF7;  // Switch open
	} else {
		switchValue |= 0x0008; // Res_Ref_In_Fm_Pin_SW Switch closed by bit 0x0008;
	}
	*CPLD_F2_XA(FPGA2_WRITE_IO_PIN_SWITCHES) = switchValue;
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS res_recvRefFreq(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received data from host, store in memory at dest = (Uint16*)can_command->datapointer
	// No action on this as it is only here for compatibiliyu with classic test station communications.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *dest;
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC
	// MboxD ignored

    // This feature is only for compatibility w/ classic Test Station commands:
    // TS3 hardware does not provide an "internal" reference source
    // since all AMC resolver drives provide an "external" reference signal.

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS res_recvRefDacValue(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received data from host, store in memory at dest = (Uint16*)can_command->datapointer
	// In classic test station, this would set voltage for an internal resolver reference signal.
	// No action on this as it is only here for compatibiliy with classic test station communications.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *dest;
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC
	// MboxD ignored

    // This feature is only for compatibility w/ classic Test Station commands:
    // TS3 hardware does not provide an "internal" reference source
    // since all AMC resolver drives provide an "external" reference signal.

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS res_recvShaftAngle(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received data from host, store in memory at dest = (Uint16*)can_command->datapointer
	// Call appropriate routine to convert shaft angle data to sin/cos DAC outputs.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *dest;
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC
	// MboxD ignored

	write_RES_Shaft_Angle(*dest);
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS res_recvShaftAngleIincreasedPrecision(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received data from host, store in memory at dest = (Uint16*)can_command->datapointer
	// Launch a background task to . . .
	// call appropriate routine to convert shaft angle data to sin/cos DAC outputs.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *dest;
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC
	// MboxD ignored

	taskMgr_setTaskRoundRobin(TASKNUM_res_ShaftAngleIincreasedPrecision, 0);

	return CANOPEN_NO_ERR;
}

//===========================================================================
// Constant Velocity Feature
// Routines kicked off by Timer0.
//===========================================================================

void res_ShaftAngleOutTask(void){
// Runs as a background task
// Take shaft-angle, convert to sin/cos, write output to DACs for Resolver Simulator

	Uint16 sin;
	Uint16 cos;

	res_calcSinCosFromHiPrecisShaftAngle(res_HiPrecisShaftAngle,&sin,&cos);

    digio_writeDacOutputValue(FPGA1_WRITE_DAC_RES_COS, cos); // Cos
    digio_writeDacOutputValue(FPGA1_WRITE_DAC_RES_SIN, sin); // Sin

}

void res_ConstVelocityTimerRoutine(void){
// Called from a hardware timer routine to periodically . . .
// if res_velocity != 0, then . . .
// increment res_HiPrecisShaftAngle and launch a background task
// to take shaft-angle, convert to sin/cos, write output to DACs for Resolver Simulator.

	if (res_velocity != 0){
		res_HiPrecisShaftAngle += res_velocity; // no problem when res_HiPrecisShaftAngle wraps
		                                        // from 0xFFFF to 0x0000 or vis versa, it is
		                                        // analogous to shaft angle wrapping from 364 to 0 deg.
		taskMgr_setTaskRoundRobin(TASKNUM_res_ShaftAngleIincreasedPrecision, 0);
	}

}



//---------BELOW THIS POINT WAS LIFTED FROM CLASSIC TEST STATION, MAY NEED ADJUSTMENTS ----------------
/**************************************************************************************
 *
 *    FILE: Resolver.C
 *
 *    DESCRIPTION: Test station generates simulated resolver signals
 *
 *    HISTORY:
 *
 *    2/14/2006 -- DH -- Created
 *
 *************************************************************************************/

/***

void resolver_constant_velocity_lo_priority_task(void)
{
// Called from main loop
// If res_velocity is not 0, then call write_RES_Shaft_Angle_increased_precision()
//   to output a new shaft angle to the resolver simulator.
// This implements a constant velocity rotation in the resolver simulator.
// Note that res_hi_precis_shaft_angle is regularly incremented by res_velocity
//   in the t2 timer interrupt, every 200 U sec.
// A res_velocity of 1, produces a rotation velocity of .07629 Hz
// A res_velocity of 0x0D produces a rotation velocity of app 1 Hz

    write_RES_Shaft_Angle_increased_precision(0, res_hi_precis_shaft_angle);
}


// Called from the Timer interrupt
// Increment shaft angle for resolver constant-velocity function.
// Remember, res_velocity is set via CAN Index, and 0 is the default value.
// normalize res_velocity to a 200uS interrupt
void increment_res_hi_precis_shaft_angle()
{
   WORD resolver_velocity;    // variable for holding normalized resolver velocity

   resolver_velocity = ((abs(res_velocity) * TIMER_ONE_PERIOD) / 200E-6);
   if(res_velocity & 0x8000)  // user entered in negative number
       res_hi_precis_shaft_angle -= resolver_velocity;
   else
       res_hi_precis_shaft_angle += resolver_velocity;
}

// data is a 16-bit value to write to *SIN_CLK, setting the
//  resolver reference frequency.  Then we shift data right
//  6 bits (divide by 64) and sat that value in *RES_FLTR_CLK
//  setting the frequency for the resolver filter clock.
void write_to_RES_Ref_Freq(WORD id, WORD data)
{

   *SIN_CLK      = data;
   *RES_FLTR_CLK = data>>6;
}


    ***/
