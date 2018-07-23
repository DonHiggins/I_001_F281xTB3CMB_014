// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     SPI.C
//
//   Read, Write, or Erase portions of the FLASH memory
//       Accept commands and data from CAN communications.
//       Return data read from FLASH over CAN.
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "SPI.h"
#include "Cpld.h"
#include "GpioUtil.H"
#include "FlashRW.H"

//****************************************************************************************************************
//*   Initialize the DSP's SPI controller to be bus master (at EEPROM baudrate) without fifo enhencements.       *
//****************************************************************************************************************
void InitSpi( void )
// Initialize the peripheral hardware setup and control registers for the SPI device (SpiA).
// This is called from main.c after the processor powers up, and after GpioU_spi_Init( )
// has configured gpio pins used by SPI. Enable lines for all devices on the Spi bus should
// put the devices in a "disabled" state.  The Enable line for the EEProm on TB3CM was disabled
// when it was configured in GpioU_spi_Init( ).  The 3 enable lines on the IO module are
// controlled by the CPLD and it "disables" them on powerup.  Hence we do not explicitly
// disable any of them here.
{
   volatile Uint16 unused;                                  // this is here to keep this function NOT INLINE inline


   	   	   //DH Note: I picked this up from Bootloader_E and translated it to use peripheral structs in DSP281x_Spi.h.
   	   	   //         Originally it used structs from TI2800.H.
   	   	   //         Some original code is indented and commented, in case we need it for reference

   	   	   //   SetupSpiEE( );                                        // set default select --> EEPROM
   	   	   //   DH: this stored a bit-mask param to enable the desired spi device, and
   	   	   //       a numeric baud-rate parameter to be used to talk to the desired device
   	   	   //       Just stored params in memory to be used when we are ready to talk to the next device

   	   	   //   DeselectSpi( );                                       // & deselect all external chips
   	   	   //   DH: equivalent to our void spi_disableAllSpiDevices() below.
   	   	   //       not sure if we need to call it here

   	   	   //   SPI.Config.b.CharLength     = 15;                     // SPI CHAR length = 16-bits
   	   	   //   SPI.Config.b.Loopback       = 0;                      // loopback = off
   	   	   //   SPI.Config.b.rsvd_1         = 0;                      //
   	   	   //   SPI.Config.b.ClockPolarity  = 1;                      // Data is output on the falling edge
   	   	   //   SPI.Config.b.rsvd_2         = 0;                      //
   	   	   //   SPI.Config.b.ResetN         = 0;                      // hold SPI in reset during changes

   SpiaRegs.SPICCR.bit.rsvd2    = 0;
   SpiaRegs.SPICCR.bit.SPISWRESET = 0;    // SW RESET (0/1) Reset condition/ready to Rx/Tx
   SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;   // POLARITY (0/1) Data output on (rising/FALLING) edge
   SpiaRegs.SPICCR.bit.SPILBK   = 0;      // not loopback-enabled
   SpiaRegs.SPICCR.bit.rsvd1    = 0;
   SpiaRegs.SPICCR.bit.SPICHAR  = 0x0F;   // 16 bit character length

   	   	   //   SPI.Control.b.IntrEn        = 0;	                     // SPI      Interrupt disabled
   	   	   //   SPI.Control.b.TalkEn        = 1;	                     // TALK     Enable transmission
   	   	   //   SPI.Control.b.Master        = 1;	                     // MASTER   Master mode
   	   	   //   SPI.Control.b.ClockPhase    = 0;	                     // PHASE    Normal phasing
   	   	   //   SPI.Control.b.OverRunIntrEn = 0;	                     // OVERRUN  Overrun interrupt disabled
   	   	   //   SPI.Control.b.rsvd_1        = 0;

   SpiaRegs.SPICTL.bit.rsvd          = 0;
   SpiaRegs.SPICTL.bit.OVERRUNINTENA = 0;  // OVER RUN    0/1 = Overrun Intr disabled/enabled
   SpiaRegs.SPICTL.bit.CLK_PHASE     = 0;  // PHASE 0/1 = Normal phasing/Delayed phasing
   SpiaRegs.SPICTL.bit.MASTER_SLAVE  = 1;  // MASTER 0/1 = Slave mode/Master mode
   SpiaRegs.SPICTL.bit.TALK          = 1;  // TALK 1 = transmission enabled
   SpiaRegs.SPICTL.bit.SPIINTENA     = 0;  // SPI INTR ENA  1 = Interrupt enabled

   	   	   //   SPI.Status.w = SPISTATUSRESET;                        // Clear recv overrun, xfer complete, tx full

   SpiaRegs.SPISTS.bit.rsvd2        = 0;
   SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1;   // Tx FULL  R:0/1 buffer empty/full   W: 1--> Clears flag
   SpiaRegs.SPISTS.bit.INT_FLAG     = 1;   // SPI INTR R:0/1 xmit not done/done  W: 1--> Clears flag
   SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1;   // OVERRUN  R:0/1 No overrun/overrun  W: 1--> Clears flag
   SpiaRegs.SPISTS.bit.rsvd1        = 0;

   	   	   //   SPI.Priority.b.EmuControl   = 1;                        //
   	   	   //   SPI.Priority.b.Reserved_1   = 0;                        //
   	   	   //   SPI.Priority.b.Reserved_2   = 0;                        //

   SpiaRegs.SPIPRI.bit.rsvd2 = 0;
   SpiaRegs.SPIPRI.bit.PRIORITY = 0; // Interrupt priority select
   SpiaRegs.SPIPRI.bit.SOFT = 0;     // Soft &   0x03: Free run.
   SpiaRegs.SPIPRI.bit.FREE = 1;     // Free     0x02: Complete current Tx/Rx before stopping.
                                     //          0x01: Free run, so breakpoints don't disturb xmission
                                     //          0x00: Stop immediately on emulation suspend.
   SpiaRegs.SPIPRI.bit.rsvd1 = 0;


   // release SPI reset
   	   	   //   SPI.Config.b.ResetN         = 1;                      // release SPI reset
   SpiaRegs.SPICCR.bit.SPISWRESET = 1;    // SW RESET (0/1) Reset condition/ready to Rx/Tx

   	   	   //   SPI.TxFifo.Status           = DISABLESPIFIFOS;        // hold spi fifo enhancements in reset

   SpiaRegs.SPIFFTX.all = 0x8000; //  DISABLESPIFIFOS;


   	   	   //   SetupSpiEE( );                                        // set default select --> EEPROM
   	   	   //   DeselectSpi( );                                       // & deselect all external chips
   	   	   //   DH: odd, these 2 functions are repeated -- they were also called at the start of this routine
   	   	   //       Not sure we really need them.
}

// When we power up, explicitly set all "chip enables" for all SPI
// devices to disable them.  Remember that all these devices have their
// output lines (MISO) wire-or'ed together, so you don't want more
// than one of them enabled at the same time.  Presumably, the CPLD's
// internal initialization brings them up in this state already at
// powerup.  But it won't hurt us to do it again.
void spi_disableAllSpiDevices()
{
	Uint16 *extData;

	extData = CPLD_XINTF_ADDR(TBIOM_CS_IO_FLSH_1);
	*extData = 0;      // write sets CS_IO_FLSH line HI, data doesn't mater

#if TB3IOMA
	// EEprom present in TB3IOMA, not present in TB3IOMB
	extData = CPLD_XINTF_ADDR(TBIOM_CS_IO_EEPRM);
	*extData = 0;      // write sets CS_IO_EEPRM line HI, data doesn't mater
#else
	//  2nd FLASH present on TB3IOMB, not present on TB3IOMA
	extData = CPLD_XINTF_ADDR(TBIOM_CS_IO_FLSH_2);
	*extData = 0;      // write sets CS_IO_FLSH line HI, data doesn't mater
#endif

	extData = CPLD_XINTF_ADDR(TBIOM_MISO_ENA);
	*extData = 0;      // write sets MISO_ENA line HI, data doesn't mater

	GpioU_SpiDisableEEPromOnTB3CM(); // set ~ChipSelect HI for EEProm on TB3CM
}

//*****************************************************************************************************************
//*    SelectSpi() Execute a selection sequence (described below) on the Spi bus to the designated Spi chip.      *
//*    (1) Deselect all chips connected to the Spi bus; this may trigger execution of a previous Spi command.     *
//*    (2) Reload the DSP's SPI controller baudrate (different Spi chips may operate at different baudrates).     *
//*    (3) Reselect the designated Spi chip: prepares it to recieve a subsequent read/write or other command.     *
//*****************************************************************************************************************
//*****************************************************************************************************************
// following defines are from bootloader_E model.hpp file.  They are here solely to establish
// a value for SPIBRRFLASH.  In the future I need to understand them and decide if they belong
// somewhere else, or if I need a better way to establish the baud rate.
#define    SYSCLOCK             150000000L
#define    LSCLOCK              (SYSCLOCK/2)
#define    SPIBAUDRATEFLASH     10000000L
#define    SPIBRRFLASH          ( ( LSCLOCK - 1) / SPIBAUDRATEFLASH )
//*****************************************************************************************************************
void SelectSpiFlash( void )
{
   volatile Uint16 word;                                    // this is here to keep this function NOT INLINE inline
   Uint16 *extData;

   spi_disableAllSpiDevices();								// deselect all external chips
   SpiaRegs.SPIBRR     = SPIBRRFLASH;

   // reset HW status bits.
   SpiaRegs.SPISTS.bit.rsvd2        = 0;
   SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1;   // Tx FULL  R:0/1 buffer empty/full   W: 1--> Clears flag
   SpiaRegs.SPISTS.bit.INT_FLAG     = 1;   // SPI INTR R:0/1 xmit not done/done  W: 1--> Clears flag
   SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1;   // OVERRUN  R:0/1 No overrun/overrun  W: 1--> Clears flag
   SpiaRegs.SPISTS.bit.rsvd1        = 0;

    // drive designated select pin
    if ((frw_GetWhichFlash()) == 1) { // 1=FLASH_1, 2=FLASH_2
	   extData = CPLD_XINTF_ADDR(TBIOM_CS_IO_FLSH_1);
    } else {
       extData = CPLD_XINTF_ADDR(TBIOM_CS_IO_FLSH_2);
    }
	word = *extData;      // read sets CS_IO_FLSH line LOW, (data doesn't mater)
	extData = CPLD_XINTF_ADDR(TBIOM_MISO_ENA);
	word = *extData;      // read sets MISO_ENA line LOW, (data doesn't mater)
	                      // enables buffer to drive MISO line from TB3IOM to TB3CM
}

//**************************************************************************************************************
//    xfer_16( data ):  an internal functon to transfer one word of data to and from the selected Spi chip.    *
//**************************************************************************************************************

Uint16 xfer_16( volatile Uint16 data )                   // this is here to keep this function NOT INLINE
   {
	SpiaRegs.SPITXBUF=data;                                  // load data to start data transfer transaction
	while ( SpiaRegs.SPISTS.bit.INT_FLAG == 0 )       { ; }  // wait until hardware completes the transaction
                                                             //  note: .INT_FLG was labeled ".Done" in Bootloader_E TI2800.h
	                                                         //  "SPI INTR R:0/1 xmit not done/done  W: 1--> Clears flag"
	return( SpiaRegs.SPIRXBUF );
   }

//***********************************************************************************
//  Xfr8bitSpiCmd(): Change SPI config to write one 8-bit character to target chip. *
//                   Do no reset or disturb any SPI pin (esp clock pin) in any way. *
//                   Restore SPI config to 16-bit char size, do not disturb clocks. *
//***********************************************************************************

Uint16 Xfr8bitSpiCmd( Uint16 cmd )
   {
   	   //DH Note: I picked this up from Bootloader_E and translated it to use peripheral structs in DSP281x_Spi.h.
   	   //         Originally it used structs from TI2800.H.
   	   //         Some original code is indented and commented, in case we need it for reference

		//SPI.Config.b.CharLength = 7;                          // set 8-bit word size
   SpiaRegs.SPICCR.bit.SPICHAR  = 0x07;   // 8 bit character length
		//SPI.Status.w        = SPISTATUSRESET;
   SpiaRegs.SPISTS.bit.rsvd2        = 0;
   SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1;   // Tx FULL  R:0/1 buffer empty/full   W: 1--> Clears flag
   SpiaRegs.SPISTS.bit.INT_FLAG     = 1;   // SPI INTR R:0/1 xmit not done/done  W: 1--> Clears flag
   SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1;   // OVERRUN  R:0/1 No overrun/overrun  W: 1--> Clears flag
   SpiaRegs.SPISTS.bit.rsvd1        = 0;

		//SPI.TxBuffer            = cmd;                        // shift 8-bits in/out
   SpiaRegs.SPITXBUF=cmd;                                // shift 8-bits in/out

		//while(SPI.Status.b.Done == 0)          { ; }
   while (SpiaRegs.SPISTS.bit.INT_FLAG == 0)   { ; }
		//SPI.Config.b.CharLength = 15;                         // set 16-bit word size
   SpiaRegs.SPICCR.bit.SPICHAR  = 0x0F;   // 16 bit character length
		//return( SPI.RxBuffer );
   return( SpiaRegs.SPIRXBUF );
   }

//****************************************************************************************************************
//*   ReadSpiFlashStatus():  Read (return to user) the status register from the "currently designated" Spi chip.      *
//****************************************************************************************************************
// FLASHRDSTATUS: 8-bit instruction to "Read Status", see data sheet for M25P40 Flash chip, p19
#define    FLASHRDSTATUS             0x05

Uint16 spi_ReadSpiFlashStatus( void )
{
   Uint16 status;
   SelectSpiFlash();                                          // toggle select, start new cmd
   status = xfer_16( FLASHRDSTATUS << 8 );                    // send 8-bit cmd, recv 8-bit status
   spi_disableAllSpiDevices();
   return( status );
}

//****************************************************************************************************************
//*   ReadSpiFlashRDID():
//****************************************************************************************************************
// CHIPSIGNATURE: 8-bit instruction (<<8) to "Read RDID Identification", see data sheet for M25P40 Flash chip, p19
#define    CHIPSIGNATURE             0xAB00
// CHIP2Mb=0x11, CHIP4Mb=0x12 };     // read flash chip electronic signature

Uint16 spi_ReleasePowerdownRES( void )
{
   Uint16 status;
   SelectSpiFlash();                                          // toggle select, start new cmd
   xfer_16( CHIPSIGNATURE );
   xfer_16( 0x0000 );
   status = xfer_16(0x0000 );
   spi_disableAllSpiDevices();
   return( status );
}

//****************************************************************************************************************
//*   ReadSpiFlashRDID():
//****************************************************************************************************************
// CHIPSIGNATURE: 8-bit instruction (<<8) to "Read RDID Identification", see data sheet for M25P40 Flash chip, p19
#define    READ_RDID             0x9F00
// CHIP2Mb=0x11, CHIP4Mb=0x12 };     // read flash chip electronic signature

void spi_ReadSpiFlashRDID(Uint16* memoryType, Uint16* memoryCapacity)
{

   SelectSpiFlash();                                          // toggle select, start new cmd
   *memoryType = xfer_16( READ_RDID );
   *memoryCapacity = xfer_16( 0x0000 );
   spi_disableAllSpiDevices();
}

//****************************************************************************************************************
//*   SetFlashWriteEnable():
//****************************************************************************************************************
// FLASHWRTENABLE: 8-bit instruction (<<8) to "write enable", see data sheet for M25P40 Flash chip, p19
#define    FLASHWRTENABLE  0x0600

void spi_SetFlashWriteEnable( void )
{
   SelectSpiFlash();                                          // toggle select, start new cmd
   Xfr8bitSpiCmd( FLASHWRTENABLE );                           // send 8-bit command
   spi_disableAllSpiDevices();
}

//****************************************************************************************************************
//*   BulkEraseFlash():
//****************************************************************************************************************
// FLASHWRTENABLE: 8-bit instruction (<<8) to "Bulk Erase", see data sheet for M25P40 Flash chip, p19
#define    FLASHBULKERASE  0xC700

void spi_BulkEraseFlash( void )
{
   SelectSpiFlash();                                          // toggle select, start new cmd
   Xfr8bitSpiCmd( FLASHBULKERASE );                           // send 8-bit command
   spi_disableAllSpiDevices();
}
//****************************************************************************************************************
//*   ReadFlash():
//****************************************************************************************************************
// EEREAD: 8-bit instruction (<<8) to "READ", see data sheet for M25P40 Flash chip, p19
#define    EEREAD       0x0300

bool spi_ReadFlash(Uint16* address, Uint16 countWordsToRead,Uint16* destBuff){
// Intention is to preface read by check of status
// and return boolean "false" if there are problems
// For now we just return True

	SelectSpiFlash();                                          // toggle select, start new cmd
	xfer_16( EEREAD | (*(address+1)& 0x00FF) );
	xfer_16( *address );

	while (countWordsToRead-- > 0){
		*(destBuff++) = xfer_16( 0 );
	}
	spi_disableAllSpiDevices();
	return(true);
}

//****************************************************************************************************************
//*   ReadFlash(): Just to clock it into FPGA -- split into 2 pieces
//****************************************************************************************************************
void spi_ClockFlashToFpgaStart(Uint16* address){
// Clocking data out of the FLASH to load program into FPGA.
// To start we set the read-address into the FLASH, but we don't read any data here

	SelectSpiFlash();                                          // toggle select, start new cmd
	xfer_16( EEREAD | (*(address+1)& 0x00FF) );
	xfer_16( *address );
}

void spi_ClockFlashToFpga(Uint16 countWordsToRead){
// Clocking data out of the FLASH to load program into FPGA.
// Assume read-address is already set in FLASH

	while (countWordsToRead-- > 0){
		xfer_16( 0 );
	}
}

//****************************************************************************************************************
//*   WriteFlash():
//****************************************************************************************************************
// EEWRITE: 8-bit instruction (<<8) to "READ", see data sheet for M25P40 Flash chip, p19
#define    EEWRITE       0x0200

bool spi_WriteFlash(Uint16* address, Uint16 countWordsToWrite,Uint16* sourceBuff){
// Intention is to preface read by check of status and write-enable,
// and return boolean "false" if there are problems
// For now we just return True

	SelectSpiFlash();                                          // toggle select, start new cmd
	xfer_16( EEWRITE | (*(address+1)& 0x00FF) );
	xfer_16( *address );

	while (countWordsToWrite-- > 0){
		xfer_16( *(sourceBuff++) );
	}
	spi_disableAllSpiDevices();
	return(true);
}


//===========================================================================
// Misc Diagnostic routines
//
//===========================================================================
void diag_SelectSpiFlash( void ){
// for diag only, public entry point to SelectSpiFlash(  )
	SelectSpiFlash();
}
