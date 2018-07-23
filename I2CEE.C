// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   I2CEE.C
//
//   Read, Write, 24LC01B I2C EEProm on TB3IOMA
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
// T I M I N G
//
// Time neasurements using the Log.C/H module report using i2cee_read() and
// i2cee_write(), to read/write 8 consecutive bytes from the I2CEEProm:
//  Read 8 Bytes:  3.0 mSec
//  Write 8 bytes: 2.8 mSec
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "Cpld.h"
#include "GpioUtil.H"
#include "Rs232Out.H"
#include "HexUtil.H"
#include "StrUtil.H"
#include "TaskMgr.h"
#include "CanOpen.h"
#include "CanFile.H"
#include "I2CEE.H"
#include "log.h"

Uint16 i2ceeDiagAddr;
Uint16 i2ceeSelectedEeprom;

Uint16 i2cee_programmingToken;
Uint16 i2ceeCanFileIndex;
enum I2CEE_CANFILESTATUS i2ceeCanFileStatus;

void i2cee_Init(void){
// Called from main() at startup
	i2cee_selectEEProm(0); // select NONE of 3 i2c eeproms
	i2ceeCanFileStatus = I2CEE_CFS_IDLE;
}

void i2cee_selectEEProm(Uint16 selection){
// designate one of 3 i2c eeproms for future r/w operations
	i2ceeSelectedEeprom = selection;
	if (selection > I2CEE_SEL_TB3PM) {
		i2ceeSelectedEeprom = 0;
	}
}

void i2cee_disableClkToAllEEProms(void){
// In current configuration, TB3CM shares I2C Clk & Data lines between
// 3 EEProm devices, one located on each module: TB3CM, TB3IOM, and TB3PM.
// For each device, it's Clk line runs through a buffer and can be turned on
// or off by enabling or disabling the buffer.
// Here we make sure that none of those buffers is enabled.
	volatile Uint16 word;	// this is here to keep this function NOT INLINE
	Uint16 *extData;

	// TB3CM -- set a DSP GPIO bit
	GpioDataRegs.GPASET.all      |= 0x4000; // Data = HI, disabling the buffer (~OE)

	// TB3IOM -- Tell CPLD to set line
	extData = CPLD_XINTF_ADDR(TBIOM_EE_I2C_CLK_ENA);
	*extData = 0;	// write sets line HI, disabling the buffer (~OE) (data doesn't mater)

	// TB3PM -- (not yet designed, probably a CPLD function)

}

void i2cee_enableClkToSelectedEEProm(void){
// In current configuration, TB3CM shares I2C Clk & Data lines between
// 3 EEProm devices, one located on each module: TB3CM, TB3IOM, and TB3PM.
// For each device, it's Clk line runs through a buffer and can be turned on
// or off by enabling or disabling the buffer.
// Here we make sure that no more than 1 of those buffers is enabled,
// corresponding to previously set value of i2ceeSelectedEeprom.
	volatile Uint16 word;	// this is here to keep this function NOT INLINE
	Uint16 *extData;

	//First lets disable all of those buffers.
	i2cee_disableClkToAllEEProms();

	//Second, enable at most 1 device
	if (i2ceeSelectedEeprom == I2CEE_SEL_TB3CM){
		// TB3CM -- set a DSP GPIO bit
		GpioDataRegs.GPACLEAR.all      |= 0x4000; // Data = HI, disabling the buffer (~OE)
	} else if (i2ceeSelectedEeprom == I2CEE_SEL_TB3IOM) {
		// TB3IOM -- Tell CPLD to set line
		extData = CPLD_XINTF_ADDR(TBIOM_EE_I2C_CLK_ENA);
		word = *extData;// read sets line LOW, enabling the buffer (~OE) (data doesn't mater)
	} else if (i2ceeSelectedEeprom == I2CEE_SEL_TB3PM) {
		// TB3PM -- (not yet designed, probably a CPLD function)
	}

}

void i2cee_delay(void){
	// For first pass, we use calls to this to pace our clocking
	// loopCount = 10 -> 6.4 uSec delay, yielsd 40kHz clock on I2C
	Uint16 loopCount;
	for (loopCount=0;loopCount<10;loopCount++){
		// not sure if this needs EALLOW/EDIS around it
		// just using it for timing, not for configuration effects

		// * * * * * * * * * * * * * * * * * * * * * * * * * * *
		//  AS OF TB3CM3: change I2C Data Line to GPIOA11 (was GPIOE0 in TB3CMA)
		//                change I2C Clk Line to GPIOA8 (was GPIOF14 in TB3CMA)
		// * * * * * * * * * * * * * * * * * * * * * * * * * * *

	    GpioMuxRegs.GPADIR.all     |= 0x0100; // Clk bit 8, as output
	}
}

#ifdef TB3CMA_GPIO
// We reassigned DSP IO Pins for I2C EEProm signals between TB3CMA and B
bool i2cee_write1Byte(Uint16 data, bool send_start, bool send_stop){
	// send Start Condition (option)
	// send 8 bits data -- could be command, address or data
	// Assume Clk Hi to start
	// return ACK value -- 1 == true
	// send Stop Condition (option)
	// trying to change or read data only in the middle of a clock HI or LOW, not on edge

	// * * * * * * * * * * * * * * * * * * * * * * * * * * *
	//  AS OF TB3CM3: change I2C Data Line to GPIOA11 (was GPIOE0 in TB3CMA)
	//                change I2C Clk Line to GPIOA8 (was GPIOF14 in TB3CMA)
	// * * * * * * * * * * * * * * * * * * * * * * * * * * *

	Uint16 i;
	bool ackValue;

    EALLOW;
	GpioMuxRegs.GPEDIR.all     |= 0x0001; // Data, bit 0, as output
	EDIS;
	// Start Condition: while Clk is HI, Data makes a Hi->Low transition
	if (send_start == true){
		// lets try setting clk LO, and waiting before doing our start byte
		GpioDataRegs.GPFCLEAR.all      |= 0x4000; // CLK = LOW
		i2cee_delay();
		GpioDataRegs.GPESET.all      |= 0x0001; // Data = HI
		i2cee_delay();
		GpioDataRegs.GPFSET.all      |= 0x4000; // CLK = HI
		i2cee_delay();
		GpioDataRegs.GPECLEAR.all    |= 0x0001; // Data = LO
		i2cee_delay();
	}

	// whether or not we send a START condition,
	// we want the Clk line low before starting to shift data
	GpioDataRegs.GPFCLEAR.all    |= 0x4000; // CLK = LOW

    // shift out our 8 bits
	for (i=0;i<8;i++){
		i2cee_delay();
		if ((data & 0x80) == 0) {
			GpioDataRegs.GPECLEAR.all  |= 0x0001; // Data = LOW
		} else {
			GpioDataRegs.GPESET.all    |= 0x0001; // Data = HI
		}
	    i2cee_delay();
	    GpioDataRegs.GPFSET.all        |= 0x4000; // CLK = HI
	    i2cee_delay();
	    i2cee_delay();
	    GpioDataRegs.GPFCLEAR.all      |= 0x4000; // CLK = LOW
	    data <<=1;
	}

	// Read ACK bit from EEProm
    EALLOW;
    GpioMuxRegs.GPEDIR.all     &= 0xFFFFE; // Data, bit 0, as input
    EDIS;
    i2cee_delay();
    i2cee_delay();
    GpioDataRegs.GPFSET.all        |= 0x4000; // CLK = HI
    i2cee_delay();
    if ((GpioDataRegs.GPEDAT.all & 0x0001) == 0) {
    	ackValue = false;
    } else {
    	ackValue = true;
    }
    i2cee_delay();

	// At this point: Clk = HI, Data = In, HI/LOW?

	// Stop: while Clk is HI, Data makes a LOW->HI transition
	if (send_stop == true){
		GpioDataRegs.GPFCLEAR.all    |= 0x4000; // CLK = LOW
	    i2cee_delay();
	    EALLOW;
	    GpioMuxRegs.GPEDIR.all       |= 0x0001; // Data, bit 0, as output
	    EDIS;
		GpioDataRegs.GPECLEAR.all    |= 0x0001; // Data = LO
		i2cee_delay();
	    GpioDataRegs.GPFSET.all      |= 0x4000; // CLK = HI
		i2cee_delay();
		i2cee_delay();
	    GpioDataRegs.GPESET.all      |= 0x0001; // Data = HI
		// leaves Clk = HI, Data = Out, HI
	}

    return ackValue;
}
#endif

#ifdef TB3CMB_GPIO
// We reassigned DSP IO Pins for I2C EEProm signals between TB3CMA and B
bool i2cee_write1Byte(Uint16 data, bool send_start, bool send_stop){
	// send Start Condition (option)
	// send 8 bits data -- could be command, address or data
	// Assume Clk Hi to start
	// return ACK value -- 1 == true
	// send Stop Condition (option)
	// trying to change or read data only in the middle of a clock HI or LOW, not on edge

	// * * * * * * * * * * * * * * * * * * * * * * * * * * *
	//  AS OF TB3CM3: change I2C Data Line to GPIOA11 (was GPIOE0 in TB3CMA)
	//                change I2C Clk Line to GPIOA8 (was GPIOF14 in TB3CMA)
	// * * * * * * * * * * * * * * * * * * * * * * * * * * *

	Uint16 i;
	bool ackValue;

    EALLOW;
	GpioMuxRegs.GPADIR.all     |= 0x0800; // Data bit as output
	EDIS;
	// Start Condition: while Clk is HI, Data makes a Hi->Low transition
	if (send_start == true){
		// lets try setting clk LO, and waiting before doing our start byte
		GpioDataRegs.GPACLEAR.all      |= 0x0100; // CLK = LOW
		i2cee_delay();
		GpioDataRegs.GPASET.all      |= 0x0800; // Data = HI
		i2cee_delay();
		GpioDataRegs.GPASET.all      |= 0x0100; // CLK = HI
		i2cee_delay();
		GpioDataRegs.GPACLEAR.all    |= 0x0800; // Data = LO
		i2cee_delay();
	}

	// whether or not we send a START condition,
	// we want the Clk line low before starting to shift data
	GpioDataRegs.GPACLEAR.all    |= 0x0100; // CLK = LOW

    // shift out our 8 bits
	for (i=0;i<8;i++){
		i2cee_delay();
		if ((data & 0x80) == 0) {
			GpioDataRegs.GPACLEAR.all  |= 0x0800; // Data = LOW
		} else {
			GpioDataRegs.GPASET.all    |= 0x0800; // Data = HI
		}
	    i2cee_delay();
	    GpioDataRegs.GPASET.all        |= 0x0100; // CLK = HI
	    i2cee_delay();
	    i2cee_delay();
	    GpioDataRegs.GPACLEAR.all      |= 0x0100; // CLK = LOW
	    data <<=1;
	}

	// Read ACK bit from EEProm
    EALLOW;
    GpioMuxRegs.GPADIR.all     &= 0xF7FF; // Data bit as input
    EDIS;
    i2cee_delay();
    i2cee_delay();
    GpioDataRegs.GPASET.all        |= 0x0100; // CLK = HI
    i2cee_delay();
    if ((GpioDataRegs.GPADAT.all & 0x0800) == 0) {
    	ackValue = false;
    } else {
    	ackValue = true;
    }
    i2cee_delay();

	// At this point: Clk = HI, Data = In, HI/LOW?

	// Stop: while Clk is HI, Data makes a LOW->HI transition
	if (send_stop == true){
		GpioDataRegs.GPACLEAR.all    |= 0x0100; // CLK = LOW
	    i2cee_delay();
	    EALLOW;
	    GpioMuxRegs.GPADIR.all       |= 0x0800; // Data bit as output
	    EDIS;
		GpioDataRegs.GPACLEAR.all    |= 0x0800; // Data = LO
		i2cee_delay();
	    GpioDataRegs.GPASET.all      |= 0x0100; // CLK = HI
		i2cee_delay();
		i2cee_delay();
	    GpioDataRegs.GPASET.all      |= 0x0800; // Data = HI
		// leaves Clk = HI, Data = Out, HI
	}

    return ackValue;
}
#endif


#ifdef TB3CMA_GPIO
// We reassigned DSP IO Pins for I2C EEProm signals between TB3CMA and B
Uint16 i2cee_read1Byte(bool send_stop){
	// read 8 bits data
	// Assume Clk Hi
	// return ACK value -- 1 == true
	// send Stop Condition (option)
	// trying to change or read data only in the middle of a clock HI or LOW, not on edge

	// * * * * * * * * * * * * * * * * * * * * * * * * * * *
	//  AS OF TB3CM3: change I2C Data Line to GPIOA11 (was GPIOE0 in TB3CMA)
	//                change I2C Clk Line to GPIOA8 (was GPIOF14 in TB3CMA)
	// * * * * * * * * * * * * * * * * * * * * * * * * * * *

	Uint16 i;
	Uint16 data_in;

    EALLOW;
    GpioMuxRegs.GPEDIR.all     &= 0xFFFFE; // Data, bit 0, as input
    EDIS;

    // shift in our 8 bits
    data_in = 0;
	for (i=0;i<8;i++){
	    GpioDataRegs.GPFCLEAR.all      |= 0x4000; // CLK = LOW
	    i2cee_delay();
	    i2cee_delay();
	    GpioDataRegs.GPFSET.all        |= 0x4000; // CLK = HI
	    i2cee_delay();
		data_in <<=1;
		data_in &=0xFE;
		data_in |= (GpioDataRegs.GPEDAT.all & 0x0001);
	    i2cee_delay();
	}

	// Spec indicates that we don't send an ACK before a STOP condition
	// in READ operations
	if (send_stop != true){
		// Send (0) ACK bit to
		GpioDataRegs.GPFCLEAR.all      |= 0x4000; // CLK = LOW
		i2cee_delay();
	    EALLOW;
		GpioMuxRegs.GPEDIR.all         |= 0x0001; // Data, bit 0, as output
	    EDIS;
		GpioDataRegs.GPECLEAR.all      |= 0x0001; // Data = LOW
		i2cee_delay();
		GpioDataRegs.GPFSET.all        |= 0x4000; // CLK = HI
		i2cee_delay();
		i2cee_delay();
		// Following line is very important . . .
		// We must set the clock line low before the next sequential read releases
		// the DATA line, otherwise, the EEProm my drive the data line HI, and think
		// it has seen a STOP (DATA:LO-->HI when CLK is HI.)
	    GpioDataRegs.GPFCLEAR.all      |= 0x4000; // CLK = LOW

		// At this point: Clk = HI, Data = Out, LOW
	} else {
		// Stop: while Clk is HI, Data makes a LOW->HI transition
	    GpioDataRegs.GPFCLEAR.all    |= 0x4000; // CLK = LOW
	    i2cee_delay();
	    EALLOW;
		GpioMuxRegs.GPEDIR.all       |= 0x0001; // Data, bit 0, as output
	    EDIS;
		GpioDataRegs.GPECLEAR.all    |= 0x0001; // Data = LOW
	    i2cee_delay();
	    GpioDataRegs.GPFSET.all      |= 0x4000; // CLK = HI
	    i2cee_delay();
		GpioDataRegs.GPESET.all      |= 0x0001; // Data = HIGH
	    i2cee_delay();
		// leaves Clk = HI, Data = Out, HI
	}

    return data_in;
}
#endif

#ifdef TB3CMB_GPIO
// We reassigned DSP IO Pins for I2C EEProm signals between TB3CMA and B
Uint16 i2cee_read1Byte(bool send_stop){
	// read 8 bits data
	// Assume Clk Hi
	// return ACK value -- 1 == true
	// send Stop Condition (option)
	// trying to change or read data only in the middle of a clock HI or LOW, not on edge

	// * * * * * * * * * * * * * * * * * * * * * * * * * * *
	//  AS OF TB3CM3: change I2C Data Line to GPIOA11 (was GPIOE0 in TB3CMA)
	//                change I2C Clk Line to GPIOA8 (was GPIOF14 in TB3CMA)
	// * * * * * * * * * * * * * * * * * * * * * * * * * * *

	Uint16 i;
	Uint16 data_in;

    EALLOW;
    GpioMuxRegs.GPADIR.all     &= 0xF7FF; // Data bit as input
    EDIS;

    // shift in our 8 bits
    data_in = 0;
	for (i=0;i<8;i++){
	    GpioDataRegs.GPACLEAR.all      |= 0x0100; // CLK = LOW
	    i2cee_delay();
	    i2cee_delay();
	    GpioDataRegs.GPASET.all        |= 0x0100; // CLK = HI
	    i2cee_delay();
		data_in <<=1;
		data_in &=0xFE;
		data_in |= ((GpioDataRegs.GPADAT.all & 0x0800)>>11);
	    i2cee_delay();
	}

	// Spec indicates that we don't send an ACK before a STOP condition
	// in READ operations
	if (send_stop != true){
		// Send (0) ACK bit to
		GpioDataRegs.GPACLEAR.all      |= 0x0100; // CLK = LOW
		i2cee_delay();
	    EALLOW;
		GpioMuxRegs.GPADIR.all         |= 0x0800; // Data, bit 0, as output
	    EDIS;
		GpioDataRegs.GPACLEAR.all      |= 0x0800; // Data = LOW
		i2cee_delay();
		GpioDataRegs.GPASET.all        |= 0x0100; // CLK = HI
		i2cee_delay();
		i2cee_delay();
		// Following line is very important . . .
		// We must set the clock line low before the next sequential read releases
		// the DATA line, otherwise, the EEProm my drive the data line HI, and think
		// it has seen a STOP (DATA:LO-->HI when CLK is HI.)
	    GpioDataRegs.GPACLEAR.all      |= 0x0100; // CLK = LOW

		// At this point: Clk = HI, Data = Out, LOW
	} else {
		// Stop: while Clk is HI, Data makes a LOW->HI transition
	    GpioDataRegs.GPACLEAR.all    |= 0x0100; // CLK = LOW
	    i2cee_delay();
	    EALLOW;
		GpioMuxRegs.GPADIR.all       |= 0x0800; // Data, bit 0, as output
	    EDIS;
		GpioDataRegs.GPACLEAR.all    |= 0x0800; // Data = LOW
	    i2cee_delay();
	    GpioDataRegs.GPASET.all      |= 0x0100; // CLK = HI
	    i2cee_delay();
		GpioDataRegs.GPASET.all      |= 0x0800; // Data = HIGH
	    i2cee_delay();
		// leaves Clk = HI, Data = Out, HI
	}

    return data_in;
}
#endif


bool i2cee_read(Uint16* dataBuf, Uint16 countBytesToRead, Uint16 eepromAddr){
// read data from the EEprom and pack it into 16-bit words in dataBuf
// called: success = i2cee_read( )
	Uint16 i;
	Uint16 data;

	LOG_I2CEE_ADDTOLOG3(LOG_EVENT_I2CEE_READ_TIMING,1,countBytesToRead,eepromAddr);

	i2cee_enableClkToSelectedEEProm(); // enable clk buff to selected 1 of 3 i2c eeproms

	//Send write command w/start
	if (i2cee_write1Byte(0xA0, SEND_START_CONDITION, DONT_SEND_STOP_CONDITION)){
		i2cee_disableClkToAllEEProms(); // disaable clk buff to all i2c eeproms
		return false;
	}
	//Send Address
	if (i2cee_write1Byte(eepromAddr, DONT_SEND_START_CONDITION, DONT_SEND_STOP_CONDITION)){
		i2cee_disableClkToAllEEProms(); // disaable clk buff to all i2c eeproms
		return false;
	}
	//Send read command w/ start
	if (i2cee_write1Byte(0xA1, SEND_START_CONDITION, DONT_SEND_STOP_CONDITION)){
		i2cee_disableClkToAllEEProms(); // disaable clk buff to all i2c eeproms
		return false;
	}
	//Send read-byte (N-1 times)
	for (i=0;i<(countBytesToRead-1);i++) {
		data = i2cee_read1Byte(DONT_SEND_STOP_CONDITION);
	    if ((i&1)== 0) {
	    	dataBuf[i>>1] = (data << 8)& 0xFF00;
	    } else {
	    	dataBuf[i>>1] |= (data & 0x00FF);
	    }
	}
	//Send read-byte w/stop
	data = i2cee_read1Byte(SEND_STOP_CONDITION);
	if ((i&1)== 0) {
		dataBuf[i>>1] = (data << 8)& 0xFF00;
	} else {
		dataBuf[i>>1] |= (data & 0x00FF);
	}

	i2cee_disableClkToAllEEProms(); // disaable clk buff to all i2c eeproms

	LOG_I2CEE_ADDTOLOG3(LOG_EVENT_I2CEE_READ_TIMING,2,countBytesToRead,eepromAddr);

	return true; // success
}

bool i2cee_write(Uint16* dataBuf, Uint16 countBytesToWrite, Uint16 eepromAddr){
// Write data to the EEprom from a packed 16-bit dataBuf.
// For ex: if you are writing only 1 byte of data, you supply it left shifted into MS nibble of dataBuf[0];
// called: success = i2cee_write( )
// Note that after we "write" the data to the eeprom, it takes time for the eeprom to program it
// into its memory, so a possible cause of failure here is if we attempt to "write" before a previous
// program operation has completed.
// Timing: writing 1 byte of data ino the EEProm, takes 879 uSec for the call to i2cee_write(),
// of course, additional time is required before the eeprom is ready again after the write operation.
// One test suggested need to wait 4.4 mSec for programming cycle to complete after end of
// i2cee_write().
	Uint16 i;
	Uint16 data;

	LOG_I2CEE_ADDTOLOG3(LOG_EVENT_I2CEE_WRITE_TIMING,3,countBytesToWrite,eepromAddr);

	i2cee_enableClkToSelectedEEProm(); // enable clk buff to selected 1 of 3 i2c eeproms

	//Send write command w/start
	if (i2cee_write1Byte(0xA0, SEND_START_CONDITION, DONT_SEND_STOP_CONDITION)){
		i2cee_disableClkToAllEEProms(); // disaable clk buff to all i2c eeproms
		LOG_I2CEE_ADDTOLOG3(LOG_EVENT_I2CEE_WRITE_TIMING,0x11,countBytesToWrite,eepromAddr);
		return false;
	}
	//Send Address
	if (i2cee_write1Byte(eepromAddr, DONT_SEND_START_CONDITION, DONT_SEND_STOP_CONDITION)){
		i2cee_disableClkToAllEEProms(); // disaable clk buff to all i2c eeproms
		LOG_I2CEE_ADDTOLOG3(LOG_EVENT_I2CEE_WRITE_TIMING,0x12,countBytesToWrite,eepromAddr);
		return false;
	}
	//Send write-byte (N-1 times)
	for (i=0;i<(countBytesToWrite-1);i++) {
		if ((i&1)== 0) {
	    	data = (dataBuf[i>>1] >> 8) & 0x00FF;
	    } else {
	    	data = (dataBuf[i>>1] & 0x00FF);
	    }
		if (i2cee_write1Byte(data, DONT_SEND_START_CONDITION, DONT_SEND_STOP_CONDITION)){
			i2cee_disableClkToAllEEProms(); // disaable clk buff to all i2c eeproms
			LOG_I2CEE_ADDTOLOG3(LOG_EVENT_I2CEE_WRITE_TIMING,0x13,countBytesToWrite,eepromAddr);
			return false;
		}
	}
	//Send write-byte w/stop
    if ((i&1)== 0) {
    	data = (dataBuf[i>>1] >> 8) & 0x00FF;
    } else {
    	data = (dataBuf[i>>1] & 0x00FF);
    }
	if (i2cee_write1Byte(data, DONT_SEND_START_CONDITION, SEND_STOP_CONDITION)){
		i2cee_disableClkToAllEEProms(); // disaable clk buff to all i2c eeproms
		return false;
	}

	i2cee_disableClkToAllEEProms(); // disaable clk buff to all i2c eeproms

	LOG_I2CEE_ADDTOLOG3(LOG_EVENT_I2CEE_WRITE_TIMING,4,countBytesToWrite,eepromAddr);

	return true; // success
}

void i2cee_writeProtect(bool on_not_off ){
// We have 3 i2c eeproms sharing clk and data lines, but they each have a separate
// write-protect mechanism.  Here we implement the write protect on or off
// mechanism for the selected eeprom, designated by value in i2ceeSelectedEeprom.
// In 24LC01B I2C EEProm,
//	Write protect is in force (ON) when the WP pin is HIGH (1)
//  The chip is write enabled - write protect is OFF - when the WP pin is LOW (0)
	Uint16 *extData;
	Uint16 tempData16;

	//First set them all on
	// TB3CM -- set a DSP GPIO bit
	GpioDataRegs.GPACLEAR.all      |= 0x8000; // Data = LOW, Write Protected

	// TB3IOM
	extData = CPLD_XINTF_ADDR(TBIOM_EE_I2C_WP);
    tempData16 = *extData; // Read from address to reset ~WP -> 0 (data not important)

	// TB3PM (not yet designed, probably CPLD)

    if (on_not_off == I2CEE_WP_ON) return;

	//Then set at most one of them to Write Enable
	if (i2ceeSelectedEeprom == I2CEE_SEL_TB3CM){
		// TB3CM -- set a DSP GPIO bit
		GpioDataRegs.GPASET.all      |= 0x8000; // Data = HI, Write Enabled
	} else if (i2ceeSelectedEeprom == I2CEE_SEL_TB3IOM) {
		// TB3IOM -- Tell CPLD to set line
		extData = CPLD_XINTF_ADDR(TBIOM_EE_I2C_WP);
		*extData = 0;	// write sets line HI, Write Enabled (data doesn't mater)
	} else if (i2ceeSelectedEeprom == I2CEE_SEL_TB3PM) {
		// TB3PM -- (not yet designed, probably a CPLD function)
	}

	extData = (Uint16 *)tempData16; // DOES NOTHING, PREVENTS warning "tempdata16 was set but never used"
}


//===========================================================================
// Misc Diagnostic routines
//
//===========================================================================

void diagRs232I2ceeRead(bool success,Uint16 address, Uint16* data, Uint16 countWords){
// Log to RS232/USB for diagnostic purposes only

	char msgOut[64];
    char *ptr;
    Uint16 i;

    ptr = strU_strcpy(msgOut,"  I2ceeRead - ");
    if (success == true){
        ptr = strU_strcpy(ptr,"success");
    } else {
        ptr = strU_strcpy(ptr,"ERROR");
    }
    ptr = strU_strcpy(ptr," @0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,address);
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

    while (countWords > 0) {
        // display words 8 at a time
    	ptr = msgOut;
        ptr = strU_strcpy(ptr,"  ");
        for (i=0;i<8;i++){
        	if (countWords > 0){
        		// display Words as hex
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,*(data++));
        		ptr = strU_strcpy(ptr," ");
                // display Chars as Ascii
        		countWords--;
        	}
        }
        ptr = strU_strcpy(ptr,"\n\r");
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    }

}

void diagRs232I2ceeWrite8(bool success){
// Log to RS232/USB for diagnostic purposes only

	char msgOut[64];
    char *ptr;

    ptr = strU_strcpy(msgOut,"  I2ceeWrite8 - ");
    if (success == true){
        ptr = strU_strcpy(ptr,"success");
    } else {
        ptr = strU_strcpy(ptr,"ERROR");
    }
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

}

//===========================================================================
// Diagnostic routine task
//
//===========================================================================

void i2cee_diag4203Task(){
	// PC wants us to read entire I2CEEProm
	// and display via RS232.


	Uint16 dataBuf[4];
	char msgOut[64];
    char *ptr;
    Uint16 i;

	// Read from I2cEEProm 8 bytes at a time
    /* success = */ i2cee_read(dataBuf, 8, i2ceeDiagAddr);

    ptr = strU_strcpy(msgOut,"  @0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,i2ceeDiagAddr);
    ptr = strU_strcpy(ptr,":   ");
    for (i=0;i<4;i++){
    	// display Words as hex
    	ptr = hexUtil_binTo4HexAsciiChars(ptr,dataBuf[i]);
    	ptr = strU_strcpy(ptr," ");
     }
    ptr = strU_strcpy(ptr,"\n\r");

    if (r232Out_outChars(msgOut, (Uint16)(ptr - msgOut))){
    	// then success writing out rs232
    	// Advance our read address and see if we have reached the end of the page
    	i2ceeDiagAddr += 8;
		if (i2ceeDiagAddr >= 256){
			// then we got to the end of the page and we are done
			return; // exit w/out re-starting the task
		}
    }
    // either RS232 Out Buffer full, we have to try again later
    // or we have more to display, next time around

	taskMgr_setTask(TASKNUM_i2cee_diag4203Task);
}
//===========================================================================
// Routines run from Command Interpreter in Comint.c
//
//===========================================================================


void i2cee_test4200(Uint16 dataWord){
// Read 8 bytes from the EEProm starting at address = dataword

	bool success;
	Uint16 dataBuf[4];

	success = i2cee_read(dataBuf, 8, dataWord);
	diagRs232I2ceeRead(success,dataWord, dataBuf, 4);

}

void i2cee_test4201(Uint16 dataWord){
// Write 8 bytes into the EEProm at consecutive address
// first byte is dataWord & 0xFF
// then add 0x11 to each successive byte
// first address is ((dataWord & 0xFF00)>>8)

	bool success;
	Uint16 dataBuf[4];
	Uint16 i;
	Uint16 address;

	for (i=0;i<8;i++){
		if ((i&1) == 0){
			dataBuf[i>>1] = ((dataWord << 8) & 0xFF00);
		} else {
			dataBuf[i>>1] |= (dataWord & 0x00FF);
		}
		dataWord += 0x0011;
	}

	address = ((dataWord >> 8)& 0xFF);

	success = i2cee_write(dataBuf, 8, address);
	diagRs232I2ceeWrite8(success);

}
void i2cee_test4203(Uint16 dataWord){
	// Dump entire I2CEEProm to RS232

	char msgOut[64];
    char *ptr;

	ptr = strU_strcpy(msgOut,"\n\ri2cee_test4203( )\n\r");
	r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

	i2ceeDiagAddr = 0;
	taskMgr_setTask(TASKNUM_i2cee_diag4203Task);

}
void i2cee_test4204(Uint16 dataWord){
// Write 1 byte in I2cEEProm
// dataword = 0xWXYZ
// write data 0xYZ at address 0xWX
	bool success;
	Uint16 dataBuf[4];
	Uint16 address;
	char msgOut[64];
    char *ptr;

	dataBuf[0] = ((dataWord << 8) & 0xFF00);
	address = (dataWord >> 8) & 0x00FF;

	success = i2cee_write(dataBuf, 1, address);

    ptr = strU_strcpy(msgOut,"  I2ceeWrite1 - ");
    if (success == true){
        ptr = strU_strcpy(ptr,"success");
    } else {
        ptr = strU_strcpy(ptr,"ERROR");
    }
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
}
void i2cee_test4205(Uint16 dataWord){
// Read 1 byte in I2cEEProm
// dataword = 0xWXzz
// Read byte address 0xWX, 0xzz - don't care
	bool success;
	Uint16 dataBuf[4];
	Uint16 address;
	char msgOut[64];
    char *ptr;

	dataBuf[0] = 0;
	address = (dataWord >> 8) & 0x00FF;

	success = i2cee_read(dataBuf, 1, address);

    ptr = strU_strcpy(msgOut,"  I2ceeRead1 - ");
    if (success == true){
        ptr = strU_strcpy(ptr,"success");
    } else {
        ptr = strU_strcpy(ptr,"ERROR");
    }
    ptr = strU_strcpy(ptr," - 0x");
	ptr = hexUtil_binTo4HexAsciiChars(ptr,dataBuf[0]);
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
}
void i2cee_test4206(Uint16 dataWord){
// Read 2 consecutive byte2 in I2cEEProm
// dataword = 0xWXzz
// Read byte address (of 1st byte) 0xWX, 0xzz - don't care
	bool success;
	Uint16 dataBuf[4];
	Uint16 address;
	char msgOut[64];
    char *ptr;

	dataBuf[0] = 0;
	address = (dataWord >> 8) & 0x00FF;

	success = i2cee_read(dataBuf, 2, address);

    ptr = strU_strcpy(msgOut,"  I2ceeRead2 - ");
    if (success == true){
        ptr = strU_strcpy(ptr,"success");
    } else {
        ptr = strU_strcpy(ptr,"ERROR");
    }
    ptr = strU_strcpy(ptr," - 0x");
	ptr = hexUtil_binTo4HexAsciiChars(ptr,dataBuf[0]);
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
}

//===========================================================================
// Routines run to service CAN commands
//
//===========================================================================


enum CANOPEN_STATUS i2cee_getTokenForEepromProg(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC wants us to program an I2C EEProm on the test station.
	// To avoid accident, this exchange sends 16-bit token to the PC
	// If the PC sends the token back to us in a programming request,
	// then we start the programming.

	// As a token we use a quasi random #, the lsw of 32-bit T0 timer counter
	i2cee_programmingToken = CpuTimer0Regs.TIM.half.LSW;
	*(data+2) = i2cee_programmingToken; //MboxC
	*(data+3) = 0;      //MboxD
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS i2cee_progEEPromFromCanFileData(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC wants us to write data into I2C EEProm.
	// To avoid accidental erasures, we give a random 16-bit token to the PC
	// If the PC sends the token back to us correctly then we initiate the EEProm programming.
	// We assume PC has used other CAN commands to send 256-byte EEProm image
	// into the CanFile buffer, and that's what we program into the EEProm.
	// We also assume the PC has used a CAN command to select one of the 3
	// EEProms (TB3CM=1, TB3IOM=2, TB3PM=3).

    if (*(data+2) != i2cee_programmingToken){
    	return CANOPEN_BAD_EEPROM_TOKEN;
    }

    //  kick off a round robin task to program I2cEEProm maybe 8 bytes at a time
	i2ceeCanFileIndex = 0;
	canF_zeroDummyFileBufOutCount();
	taskMgr_setTaskRoundRobin(TASKNUM_i2cee_progEEPromFromCanFileTask, 0);
	i2ceeCanFileStatus = I2CEE_CFS_WRITING_EEPROM;
	i2cee_writeProtect(I2CEE_WP_OFF);

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS i2cee_readEEPromStatus(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// Here we are going to return 4 bytes of status info to the host.
	// As yet, exact nature of status info is undefined, . . . But for starters
	//    Host wants to know if we have completed reading EEProm into CanFile buffer
	//    Host wants to know if we have completed programming EEProm from CanFile buffer

	i2cee_programmingToken = CpuTimer0Regs.TIM.half.LSW; // DH: what is this line doing here? 11/16/2016
	*(data+2) = (Uint16)i2ceeCanFileStatus; //MboxC
	*(data+3) = 0;      //MboxD

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS i2cee_readEEPromToCanFile(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC wants us to read I2CEEProm data.
	// Here we kick off a background task to read entire EEProm into CanFile Buffer.

	i2ceeCanFileIndex = 0;
	i2ceeCanFileStatus = I2CEE_CFS_READING_EEPROM;
	canF_zeroDummyFileBufCounts(); // initialize canFileBuf counts before appending into it
	taskMgr_setTaskRoundRobin(TASKNUM_i2cee_readEEPromToCanFileTask, 0);

	return CANOPEN_NO_ERR;
}

void i2cee_readEEPromToCanFileTask(void){
	// Read entire I2CEEProm 8 bytes at a time
	// into CanFile buffer for subsequent CAN transfer back to host.

	Uint16 dataBuf[4];
	char dataBufChar[8];
    Uint16 i;
    Uint16 j;
    bool success;

	// Read from I2cEEProm 8 bytes at a time
    success = i2cee_read(dataBuf, 8, i2ceeCanFileIndex);

    if (!success) {
    	i2ceeCanFileStatus = I2CEE_CFS_READING_EEFAIL;
    	// Exit without re-launching task
    	return;
    }

    // unpack 16-bit data into 8-bit char
    j = 0;
    for (i=0;i<4;i++){
    	dataBufChar[j++] = (dataBuf[i] >> 8) & 0x00FF;
    	dataBufChar[j++] = dataBuf[i] & 0x00FF;
    }
    canF_appendIntoDummyFileBuf(dataBufChar, 8) ;

    i2ceeCanFileIndex += 8;
	if (i2ceeCanFileIndex >= 256){
		// then we got to the end of the page and we are done
    	i2ceeCanFileStatus = I2CEE_CFS_READING_EEDONE;
		return; // exit w/out re-starting the task
	}

	// Re-Launch the task to continue
	taskMgr_setTaskRoundRobin(TASKNUM_i2cee_readEEPromToCanFileTask, 0);

}

void i2cee_progEEPromFromCanFileTask(void){
//  Background task to program I2cEEProm maybe 8 bytes at a time
//  with data from the CanFileBuf.
//  Manage the I2CEEProm Status value as programming proceeds.

	Uint16 charCountToProgram;
	char dataBufChar[8];
    #define MAXCHARSTOPROGRAM 8
	Uint16 dataBuf[4];
	Uint16 i;
	Uint16 eepromAddress;
    bool success;

	// Read characters out of CanFileBuf
	charCountToProgram =  canF_readOutOfDummyFileBuf(dataBufChar, MAXCHARSTOPROGRAM);

	// 1 to 8 characters is legit, otherwise error
	if ((charCountToProgram < 1) || (charCountToProgram > MAXCHARSTOPROGRAM)) {
		i2ceeCanFileStatus = I2CEE_CFS_WRITING_EEFAIL;
		i2cee_writeProtect(I2CEE_WP_ON);
		return; // without re-launching the task
	}
	// Also, we should be working with an even # of characters
	// And frankly it should always be 8, though we are writing this coverring
	// a more general case.
	if (charCountToProgram & 0x0001){
		i2ceeCanFileStatus = I2CEE_CFS_WRITING_EEFAIL;
		i2cee_writeProtect(I2CEE_WP_ON);
		return; // without re-launching the task
	}

	//Pack the characters from CanFileBuf into 16-bit words
	for (i=0;i<charCountToProgram;i++){
		if ((i&1) == 0){
			dataBuf[i>>1] = ((dataBufChar[i] << 8) & 0xFF00);
		} else {
			dataBuf[i>>1] |= (dataBufChar[i] & 0x00FF);
		}
	}

	// now write to the I2CEEProm
	eepromAddress = (i2ceeCanFileIndex& 0xFF);
	success = i2cee_write(dataBuf, charCountToProgram, eepromAddress);

	if (!success) {
		i2ceeCanFileStatus = I2CEE_CFS_WRITING_EEFAIL;
		i2cee_writeProtect(I2CEE_WP_ON);
		return; // without re-launching the task
	}

	i2ceeCanFileIndex += charCountToProgram;
	if (i2ceeCanFileIndex == 256){
		i2ceeCanFileStatus = I2CEE_CFS_WRITING_EEDONE; //Successful completion
		i2cee_writeProtect(I2CEE_WP_ON);
		return; // without re-launching the task
	}

    // Otherwise, we are not done yet, so set the task to run again
	// Note, it appears to program OK when we introduced a 0.1 sec deley before
	// re-firing the task -- that introduces app 3.2 sec of delay to write the whole thing.
	// And it definitely fails if we remove the 0.1 sec delay.
	// Looking at the Data Sheet for the 24LC02B, I'm guessing that after we send it
	// 8 bytes and tell it to program that it introduces an internal write cycle, and
	// is not available to accept more data until the conclusion of the write cycle.
	// The spec sheet tells how to do "Acknowledge Polling" to determine when the internal
	// write cycle is done, but we haven't implemented that yet.  For now, we'll use the
	// 0.1 sec delay.  If we need to speed it up, we can look into implementing "Acknowledge Polling."
	// DH -- 4/20/2016
	taskMgr_setTaskRoundRobin(TASKNUM_i2cee_progEEPromFromCanFileTask, 1); // 0.1 sec delay
	i2ceeCanFileStatus = I2CEE_CFS_WRITING_EEPROM; // continuing
}

Uint16 eeProm1ByteRWAddr;

enum CANOPEN_STATUS i2cee_readEEProm1Byte(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC wants us to read 1 byte of I2CEEProm data
	// from EEProm address previously set in eeProm1ByteRWAddr;
	// After the read we increment eeProm1ByteRWAddr for the next read.
	// Note: this assumes pc has previously selected one of our 3 I2CEEProms.
	//TIMING: takes 1.1mSec to read 1 byte of data from I2CEEprom.

	bool success;
	Uint16 dataByteFromEeProm;

    // in case of error, pre-load 0's into return data loacation
	*(data+2) = 0; 		//MboxC
	*(data+3) = 0;      //MboxD

    // Return error if PC has not previously selected an eeprom
	if ((i2ceeSelectedEeprom == 0) || (i2ceeSelectedEeprom > 3)) {
		return CANOPEN_I2CEEPROM_NO_SEL;
	}

    // Sanity check, EEProm addresses are 0 to 255
	if (eeProm1ByteRWAddr > 255) {
		eeProm1ByteRWAddr = 0;
	}

	// Increment eeProm1ByteRWAddr so next readEEProm1Byte request reads next byte
	success = i2cee_read(&dataByteFromEeProm, 1, eeProm1ByteRWAddr);

	if (success == false){
		return CANOPEN_I2CEEPROM_READ_ERR;
	}

	eeProm1ByteRWAddr++;

    // return 1 byte data from I2CEEProm
	// remember: i2cee_read() returns odd # of bytes left justified in 16-bit result.
	*(data+2) = (dataByteFromEeProm >> 8) & 0xFF; //MboxC
	*(data+3) = 0;      //MboxD

	return CANOPEN_NO_ERR;
}
enum CANOPEN_STATUS i2cee_writeEEProm1Byte(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// PC wants us to write 1 byte of I2CEEProm data
	// to EEProm address previously set in eeProm1ByteRWAddr;
	// After the write we increment eeProm1ByteRWAddr for the next write.
	// Note: this assumes pc has previously selected one of our 3 I2CEEProms.
	// Note: after a successful 1-byte-write, it may take an additional 4 mSec for the
	// eeprom to complete it's write cycle before it is ready for the next
	// i2cee_writeEEProm1Byte() command.
	Uint16 eepromWriteData;
	bool success;

	// data from PC:
	//  MS word i2cee_programmingToken -- PC fetched previously, here we validate it before writing
	//  LS word data byte to write to EEProm.
	eepromWriteData = (*(data+2)) & 0x00FF;

    if (*(data+3) != i2cee_programmingToken){
    	return CANOPEN_BAD_EEPROM_TOKEN;
    }

    // Write-enable the selected ee-prom
    i2cee_writeProtect(I2CEE_WP_OFF);

    // Sanity check, EEProm addresses are 0 to 255
	if (eeProm1ByteRWAddr > 255) {
		eeProm1ByteRWAddr = 0;
	}

	// Left shift data byte into MS nibble of what we pass to i2cee_write( ).
	eepromWriteData = (eepromWriteData << 8) & 0xFF00;
	success = i2cee_write(&eepromWriteData, 1, eeProm1ByteRWAddr);
	i2cee_writeProtect(I2CEE_WP_ON); //Disable writing to EEPROM (is this too soon?)
	if (success == false){
		return CANOPEN_I2CEE_WRITE_FAIL;
	}

	eeProm1ByteRWAddr++;
	return CANOPEN_NO_ERR;
}

Uint16 read32FromEEpromToBufTaskStatus;
Uint16 burn32ToEEpromToBufTaskStatus;
enum I2CEE_32BYTESTATUS i2cee32ByteStatus;
Uint16 eeProm32ByteRWAddr;
Uint16 packed32ByteBuf[16];
Uint16 burn32ToEEpromTaskDeadmanSwitch;

enum CANOPEN_STATUS i2cee_32BytesFromEEpromToBuf(const struct CAN_COMMAND* can_command, Uint16* data){
	//Read 32 bytes out of EEProm @eeProm32ByteRWAddr.
	//Leave it in a buffer so PC can read it -- maybe the CAN multi-packet buf.
	//May need to do this in a background task:
	//(Info from timing measurements):
	//   1 x i2cee_read() 32 bytes takes 9.5 mSec
	//   4 x i2cee_read() 8 bytes takes 4 x 3 mSec
	//   8 x i2cee_read() 4 bytes takes 8 x 1.7 mSec
	//Need to update status so PC can tell when we are done (via i2cee_readEEPromStatus)
	//Uint16 i;

	//PC reads i2cee32ByteStatus to see when read from EEProm is complete
	i2cee32ByteStatus = I2CEE_32BYTE_READING_EEPROM;

	//Start a background task to actually read 32 bytes from the EEProm
	read32FromEEpromToBufTaskStatus = 0; // starts task at byte 0
	taskMgr_setTaskRoundRobin(TASKNUM_i2cee_read32FromEEpromToBufTask, 0);

	// Debugging problem -- initialize packed32ByteBuf
	//for (i=0; i<16; i++){
	//	packed32ByteBuf[i] = 0x77;
	//}

	return CANOPEN_NO_ERR;
}

void i2cee_read32FromEEpromToBufTask(void){
	// background task reads 32 bytes from eeprom servicing
	// CAN request: i2cee_32BytesFromEEpromToBuf( ) above.
	// Breaks the task up into 8 x 4-byte reads so as not to occupy
	// the processor for a single 9.5 mS segment to read all 32 bytes at once.
	// Each 4-byte read takes 1.7 mSec.
	// When we are done, we set i2cee32ByteStatus to let PC know.
	// PC will then do a multi-packet read of the 32 bytes.
	bool success;
	Uint16 i;

	if (read32FromEEpromToBufTaskStatus < 8) {
		i = ((read32FromEEpromToBufTaskStatus << 1)& 0x0E); // word offset into packed32ByteBuf
		                                                    // 0->0 // 1->2 // ... // 8->16
		success = i2cee_read((packed32ByteBuf + i),4,eeProm32ByteRWAddr);
		if (success == false){ //on F6 CCS appears to step thru this loop even when success == true.
			i2cee32ByteStatus = I2CEE_32BYTE_READING_EEFAIL;
			return; // exit without re-launching the task
		}
		// eeprom read ok so far
		eeProm32ByteRWAddr = eeProm32ByteRWAddr + 4; // advance eeprom address by 4 bytes
		read32FromEEpromToBufTaskStatus++;
		taskMgr_setTaskRoundRobin(TASKNUM_i2cee_read32FromEEpromToBufTask, 0); // re-launch the task
		return;
	}
	// so we have completed reading 32 bytes successfully
	// set status to let PC know of success, don't re-launch the task
	i2cee32ByteStatus = I2CEE_32BYTE_READING_EEDONE;

}

enum CANOPEN_STATUS i2cee_read32ByteStatus(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// Here we are going to return 4 bytes of status info to the host.
	// As yet, exact nature of status info is undefined, . . . But for starters
	//    Host wants to know if we have completed reading EEProm into 32 Byte buffer
	//    Host wants to know if we have completed programming EEProm with 32 Bytes from buffer

	//i2cee_programmingToken = CpuTimer0Regs.TIM.half.LSW; // DH: what is this line doing here? 11/16/2016
	*(data+2) = (Uint16)i2cee32ByteStatus; //MboxC
	*(data+3) = 0;      //MboxD

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS i2cee_32BytesToPC(const struct CAN_COMMAND* can_command, Uint16* data){
	//Send 32 bytes from a buffer to PC via multi-packet non-expedited SDO
	// *data is char buff in MULTI_PACKET_BUF
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// We get here each time the CanOpen module receives a request to start a multi-packet
	// data set SEND back to the host for our index.subindex

	Uint16 countCopied;

	// copy 32 packed bytes from packed32ByteBuf into multi-packet bufer
	countCopied = copyPacked32BytesToMultiPacketBuf(packed32ByteBuf);
	if (countCopied == 0){
	   return CANOPEN_MULTI_SEG_005_ERR; //PC requests multi-segment transfer
	                                     // but multi-seg buffer is already busy.
	}

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS i2cee_32BytesFromPC(const struct CAN_COMMAND* can_command, Uint16* data){
	//We received 32 bytes from PC into the multi-packet buffer via multi-packet non-expedited SDO.
    // Here we copy it into our packed32ByteBuf,
	// from whence we will later program it to EEProm.
	Uint16 countCopied;

	countCopied = copy32BytesFromMultiPacketBufToPackedBuf(packed32ByteBuf);

	if (countCopied == 0){
	   return CANOPEN_MULTI_SEG_006_ERR; //Non specific problem with 32-byte
	                                     // multi-seg xfer from PC.
	}

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS i2cee_burn32BytesToEEProm(const struct CAN_COMMAND* can_command, Uint16* data){
	//Burn 32 bytes from a buffer -- packed32ByteBuf-- into EEProm @eeProm32ByteRWAddr.
	//Kick off a background task to do this.
	//Verify that PC has sent us a valid Programming Token,
	//as safety measure against unintended overwriting of I2CEEProm contents
	//Need to update status so PC can tell when we are done (via i2cee_readEEPromStatus)

    if (*(data+2) != i2cee_programmingToken){
    	return CANOPEN_BAD_EEPROM_TOKEN;
    }

	//PC reads i2cee32ByteStatus to see when write EEProm is complete
	i2cee32ByteStatus = I2CEE_32BYTE_WRITING_EEPROM;

	//Start a background task to actually write 32 bytes from the EEProm
	burn32ToEEpromToBufTaskStatus = 0; // starts task at byte 0
	burn32ToEEpromTaskDeadmanSwitch = 0; // part of task initialization
	taskMgr_setTaskRoundRobin(TASKNUM_i2cee_burn32ToEEpromTask, 0);

	i2cee_writeProtect(I2CEE_WP_OFF);

	return CANOPEN_NO_ERR;
}

void i2cee_burn32ToEEpromTask(void){
	// background task burns 32 bytes to eeprom, servicing
    // i2cee_burn32BytesToEEProm() CAN command
	// First pass burns 8 bytes at a time into eeprom.
	// Writing 8 bytes into eeprom takes 2.4 mSec to write to eeprom' internal buffer + write cycle time
	// Spec sheet quotes 5 mSec write cycle time.
    bool success;
    Uint16 i;

	if (burn32ToEEpromToBufTaskStatus < 4){
		i = ((burn32ToEEpromToBufTaskStatus << 2)& 0x0C); // word offset into packed32ByteBuf
		// now write to the I2CEEProm
		success = i2cee_write(&(packed32ByteBuf[i]), 8, eeProm32ByteRWAddr);
		if (success) {
			eeProm32ByteRWAddr += 8;
			burn32ToEEpromToBufTaskStatus++;
		} else { // !success -- There are a couple ways we can handle this
			// Assume !success is because previous EEProm write cycle is not done yet
			//  so we just repeat until it does work.   Probably should add some
			//  counter to make sure we don't do it forever.
			//  Also added a 0.1 sec delay before each successive re-launch from task mgr..
			LOG_I2CEE_ADDTOLOG3(LOG_EVENT_I2CEE_WRITE_TIMING,0x21,
					burn32ToEEpromToBufTaskStatus,burn32ToEEpromTaskDeadmanSwitch);
			if (burn32ToEEpromTaskDeadmanSwitch++ > 10) {
				i2cee32ByteStatus = I2CEE_32BYTE_WRITING_EEFAIL;
				i2cee_writeProtect(I2CEE_WP_ON);
				return; // error: exit without re-launching the task
			}
		}

	} else { // burn32ToEEpromToBufTaskStatus >= 4
		// Thru writing 32 bytes to eeprom, exit without re-launching task
		// turn write-protect back on
		i2cee_writeProtect(I2CEE_WP_ON);
		i2cee32ByteStatus = I2CEE_32BYTE_WRITING_EEDONE;
		return;
	}

	// Re-launch this task
	taskMgr_setTaskRoundRobin(TASKNUM_i2cee_burn32ToEEpromTask, 1); // 0.1 Sec delay before re-launcch

}

