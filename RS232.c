// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     RS232.C
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File

#include "stdbool.h"            // needed for bool data types
#include <string.h>             // needed for strcpy() function

#include "RS232.h"
#include "TaskMgr.h"
#include "HexUtil.h"
#include "Timer0.h"
#include "Comint.h"
#include "Rs232Out.h"
#include "StrUtil.H"

char msg_notLegalBaudRate[] = {"Not a legal Baud Rate value\n\r"};
char msg_changeBaudRateIn3Sec[] = {"Changing Baud Rate in 3 Sec\n\r"};
char msg_notLegalParity[] = {"Not legal Parity value\n\r"};
char msg_notLegalCharSize[] = {"Not legal Char size\n\r"};
char msg_notLegalStopBits[] = {"Not legal Stop Bits\n\r"};
char msg_detectedBreakOrError[] = {"\n\r\n\r"};
char msg_AckAutobaud[] = {"\n\rAutobaud detected\n\r\n\r"};
char msg_Ts3Banner001[] = {"\n\r - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n\r"};
char msg_Ts3Banner002[] = {"\n\r     T S 3   D I A G N O S T I C   P O R T A L \n\r"};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//   RS232 BAUD RATE
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#define CPU_FREQ 	    150E6
#define LSPCLK_FREQ     CPU_FREQ/4
#define SCI_BAUD_RATE   9600
//#define SCI_BRR 	    (LSPCLK_FREQ/(SCI_BAUD_RATE*8))-1
#define SCI_BRR          0x1E7

// --  Baud Rate explanation --
// CPU Frequency is 150MHz, "150E6"
// Low Speed Peripheral Clock runs at 1/4 CPU Freq, 37.5Mhz
// Page 2-8 in TMS320x281x, 280x DSP Serial Communication Interface (SCI) Reference Guide
// gives formula:
//   Baud Rate Register = ((LSPCLK / (<async_baud_rate>*8)) - 1
//   for ex: 0x1E7 = 487 = ((37500000/(9600*8))-1
//   so set SCILBAUD = 0x0E7, SCIHBAUD = 0x0001
//   Note (virtual) Baud Rate Register (BRR) combines
//       ls 8 bits of SCILBAUD + (0x100 * ls 8 bits of SCIHBAUD)
//
//    BAUD  BRR
//    9600: 0x1E7 = 487
//   19200: 0x0F3 = 243
//  115200: 0x028 =  40

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//   Definitions for useful ASCII Character Values
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#define CR 0x0D
#define LF 0x0A
#define SP 0x20

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Prototype statements for functions found within this file.
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
interrupt void sciaTxFifoIsr(void);
interrupt void sciaRxFifoIsr(void);
void scia_fifo_init(void);
void error(void);

void TurnOnTx(void);

//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
// RS232 private variables
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
Uint16 sci_brr; // value to split into 2 bytes and write into SCIHBAUD and SCILBAUD
Uint16 sci_ccr; // value to put parity, chars & stop bits into SCICCR
Uint16 rs232BaudChangeCounter;
Uint16 afterBreakCounter;
Uint16 rs232AckAutobaudCounter;

char *sdata;
Uint16 sdataIndex;
Uint16 txBuf[80];
Uint16 sdataCount;

#define RXDATA_MAX_LENGTH 80
Uint16 rdata[RXDATA_MAX_LENGTH];
Uint16 rdataIndex;
// char ooad[] = "\r\n\nOutside of a dog, a book is man's best friend! \r\n\0";
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// RS232 non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
struct RS232_STATUS Rs232Status; // status of firmware operations, not H/W see RS232.h

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessor for RS232 non-public variable : Rs232Status
bool rs232_transmit_status_busy(void){
	// 'true' indicates rs232 module's local RAM buffer not empty
	if (Rs232Status.RS232_STAT.bit.TX_BUSY == 1) {
		return true;
	} else {
		return false;
	}
}
bool rs232_txFifo_Busy(void){
	// 'true' indicates hardware tx FIFO not empty
	if ((((SciaRegs.SCIFFTX.all & 0x1F00) >> 8) & 0x1F) != 0) {
		return true;
	} else {
		return false;
	}
}
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o
// Accessor for RS232 non-public variable :kicking off a transmit operation
void rs232_transmit_characters(char* chars_to_transmit,int count){
    sdata = chars_to_transmit;
    sdataCount = count;
    TurnOnTx();
}
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//    . . . called from Main module during initialization
// PIE Interrupts vectors are mapped to ISR functions found within this file.
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_store_int_vectors_in_PIE(void) {
   EALLOW;	// This is needed to write to EALLOW protected registers
   PieVectTable.RXAINT = &sciaRxFifoIsr;
   PieVectTable.TXAINT = &sciaTxFifoIsr;
   EDIS;   // This is needed to disable write to EALLOW protected registers
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//    . . . called from Main module during initialization
// Enable PIE interrupts required for RS232 (sciA)
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_enable_PIE_int(){
   PieCtrlRegs.PIECRTL.bit.ENPIE = 1;   // Enable the PIE block
   PieCtrlRegs.PIEIER9.bit.INTx1=1;     // PIE Group 9, INT1
   PieCtrlRegs.PIEIER9.bit.INTx2=1;     // PIE Group 9, INT2
   IER = 0x100;	// Enable CPU INT
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// sciA / RS232 Tx interrupt.  Since we use FIFOs, Tx interrupt
// occurs when Tx FIFO is empty, and Tx interrupt is enabled.
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
interrupt void sciaTxFifoIsr(void)
{
	Uint16 TxFifoFill;

    // NOTE: F2812 has a 16-level FIFO

    // Assume before we got here, someone wants to Transmit something,
    // and they enabled TxFF interrupts.  Hence we are here, and
    // hence we can assume our sdata buffer has at least 1 byte to transmit.

    // But that's not always true -- something in the process of resetting
	// and reinitializing SciA after receiving a BREAK is throwing spurious
	// TX interrupts.  Consequently we have allowed a path to gracefully
	// exit when no data needs transmitting.

	TxFifoFill = ((SciaRegs.SCIFFTX.all & 0x1F00) >> 8) & 0x1F;
	while((TxFifoFill < 16) && (sdataIndex < sdataCount))
	{
    	SciaRegs.SCITXBUF = sdata[sdataIndex++];
    	TxFifoFill = ((SciaRegs.SCIFFTX.all & 0x1F00) >> 8) & 0x1F;
	}

    if(sdataIndex >= sdataCount){
    	// Breaks us out of the loop of continuing TxFF interrupts
    	//   by disabling the interrupt.
    	// Next time somebody wants to transmit something,
    	//   they re-enable the interrupt.
    	SciaRegs.SCIFFTX.bit.TXFFIENA = 0; //disableTxFifoInt
    	// Turn off Tx_Busy status to announce we are thru transmitting.
    	Rs232Status.RS232_STAT.bit.TX_BUSY = 0;
    	taskMgr_setTaskRoundRobin(TASKNUM_BgTask_TxDone, 0);	// set a flag to run a task in the background
    	                                        				// in case there is more to transmit
    }

	SciaRegs.SCIFFTX.bit.TXINTCLR=1;	// Clear SCI Interrupt flag
	PieCtrlRegs.PIEACK.all|=0x100;      // Issue PIE ACK

    return;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// sciA / RS232 Rx interrupt.  Since we use FIFOs, Tx interrupt
// occurs when Rx FIFO >= fill level, which we set at 1 character received.
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
interrupt void sciaRxFifoIsr(void)
{

	Uint16 RxFifoFill;
	Uint16 incoming_char;
	bool launchBreakOrErrorTask;

	launchBreakOrErrorTask = false; // until we decide otherwise

    // We got here, maybe there is at least 1 received character
	// or break or error

	if ((SciaRegs.SCIRXST.all & 0xBC) == 0) { // if NOT responding to a BREAK or error

		do {  // empty Rx FIFO into a buffer in memory
			incoming_char = SciaRegs.SCIRXBUF.all;
			rdata[rdataIndex++] = incoming_char;
			RxFifoFill = ((SciaRegs.SCIFFRX.all & 0x1F00) >> 8) & 0x1F;
		}while((RxFifoFill > 0) && (rdataIndex < RXDATA_MAX_LENGTH) && (incoming_char != CR));
		// NOTE: F2812 has a 16-level FIFO

		if (SciaRegs.SCIFFCT.bit.CDC == 1) { // if we are doing autobaud detect
			rdataIndex = 0;                  // discard autobaud characters
		} else if((rdataIndex >= RXDATA_MAX_LENGTH) || (rdata[rdataIndex-1]== CR)){
			// and don't run the commandDecode task durring autobaud detect
			taskMgr_setTaskRoundRobin(TASKNUM_RS232_commandDecode,0);	// set a flag to run a task in the background
															// copy received chars to transmit buffer
															// and initiate a transmit
		}

		// see if we just completed an auto-baud detect action as indicated by:
		//   CDC = 1 means we requested an autobaud
		//   SCIBAUDH & L != 0000, 0001
		if (SciaRegs.SCIFFCT.bit.CDC == 1) {
			if ((SciaRegs.SCIHBAUD != 0) || (SciaRegs.SCILBAUD !=1)) {
				SciaRegs.SCIFFCT.bit.CDC = 0;   // Auto baud mode disenable
				SciaRegs.SCIFFCT.bit.ABDCLR = 1; // 0x4000 Auto baud clear
				sci_brr = ((SciaRegs.SCIHBAUD << 8) & 0xFF00) | SciaRegs.SCILBAUD; // capture new baud rate
				rs232AckAutobaudCounter = 0;
				taskMgr_setTaskRoundRobin(TASKNUM_RS232_ackAutobaud,0);	// launch background task to print out acknowledgement
			} else {
				// got here because autobaud is in progress, but we didn't capture a
				// valid baud rate -- SCIBAUDH & L == 0000, 0001 -- and we got a
				// receiver interrupt.  I think I can cause this by typing a character
				// other than the 'A' or 'a'.

				launchBreakOrErrorTask = true;

				// Hopefully, resetting the SCI and re-launching TASKNUM_RS232_breakOrError
				// will allow us a 2nd chance for the operatore to type the correct
				// autobaud character.
			}
		}

	} else {
		// WE GET HERE ON BREAK AND ERROR !!!

		// empty receive fifo
		do {
			incoming_char = SciaRegs.SCIRXBUF.all;
			RxFifoFill = ((SciaRegs.SCIFFRX.all & 0x1F00) >> 8) & 0x1F;
		} while (RxFifoFill > 0);

		launchBreakOrErrorTask = true;
    }

	if (launchBreakOrErrorTask == true) {
		SciaRegs.SCICTL1.all =0x0003;  // Reset SciA, leave TXENA & RXENA
		                               // this resets error flags in SCIRXST
		afterBreakCounter = 0;
		taskMgr_setTaskWithDelay(TASKNUM_RS232_breakOrError,5); // run bkgnd task after 0.5 sec

	}

    // Clear Overflow flag
	SciaRegs.SCIFFRX.bit.RXFFOVF=1;	    // Clear Receive FIFO overflow flag
	SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;	// Clear SCI Interrupt flag
	PieCtrlRegs.PIEACK.all|=0x100;      // Issue PIE ACK

    return;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Write to sciA peripheral registers to configure RS232
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_scia_fifo_init()
// this is the primary initialization routine for RS232, called from main( ) at
// power-up.  This sets default baud rate and parity, as well as FIFO use.
{
   SciaRegs.SCICCR.all = sci_ccr; // stop bit,  No loopback, parity, char-bits,
                                  // async mode, idle-line protocol
   SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                  // Disable RX ERR, SLEEP, TXWAKE

   //SciaRegs.SCICTL2.bit.TXINTENA =1; // Don't enable the Tx interrupt here.
                                       // Enable it only when we have something to transmit.
   SciaRegs.SCICTL2.bit.RXBKINTENA =1; // Enable interrupt for receipt of char or BREAK

   SciaRegs.SCIHBAUD = (sci_brr >> 8) & 0xFF;
   SciaRegs.SCILBAUD = sci_brr & 0xFF;

   SciaRegs.SCIFFTX.all=0xC060;   // interrupt at tx buff fill = 0
   SciaRegs.SCIFFRX.all=0x0021;   // interrupt at rx buf fill = 1
   SciaRegs.SCIFFCT.all=0x00;

   SciaRegs.SCICTL1.all =0x0063;     // Relinquish SCI from Reset
                                     //  and enable interrupt on Rx Error
   SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
   SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
}

void rs232_start_autobaud_detect(void)
// As described in SCI Reference guide, page 1-22, 23
// with this set up, an incoming 'a' or 'A' causes baud to be automatically calculated,
// and causes a Rx interrupt, where we read the 'A' or 'a' from the receive buffer.
{
	SciaRegs.SCIFFCT.bit.CDC = 1;    // Auto baud mode enable
	SciaRegs.SCIFFCT.bit.ABDCLR = 1; // clear auto baud detect flag
	SciaRegs.SCIHBAUD = 0;           // set baud rate register to 1
	SciaRegs.SCILBAUD = 1;
}

void rs232_setBaudRateDefault(void){
	// called from main( )at power-up,
	// called before main( ) calls rs232_scia_fifo_init( ), above.
	// sets default baud rate to 9600 baud, parity, chars & stop-bits
	sci_brr = (Uint16)SCI_BRR;
	sci_ccr = 0x0007;               // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Start a transmission
// Assumes: sdata points to a character string to transmit, and
//          sdataCount is the count of characters to transmit.
//          For example
//            sdata = ooad;
//            sdataCount = 50;
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void TurnOnTx(void)
    {
    //Start our SCI transmitting operation,
    //Should quickly generate an SCI Tx interrupt
    sdataIndex = 0;
//    rdataIndex = 0;

    Rs232Status.RS232_STAT.bit.TX_BUSY = 1;

	SciaRegs.SCIFFTX.bit.TXINTCLR=1;	// Clear SCI Interrupt flag
                                        // write 1 to TXFFINT_CLR  bit
                                        // in SCIFFTX to clear Tx interrupt flag

	SciaRegs.SCIFFTX.bit.TXFFIENA =1;   // write 1 to TXFFIENA bit in SCIFFTX to
                                        // enable Tx interrupt
}


//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  "Halt and Catch Fire" -- lifted from TI RS232 Example
//  Not called from anywhere, as of 7/24/2013
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void error(void)
{
    asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Called from MAIN.C at startup
//  Perform any additional initialization needed for RS232 Tx & Recv
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_Init(void)
{
    rdataIndex = 0; // index into firmware receive buffer
    sdataIndex = 0;
    sdataCount = 0;
}


// %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-
//
//       |      COMMAND DECODE & BACKGROUND TASKS      |
//       V                                             V
//
// %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Called from MAIN.C at startup
//  Perform any initialization needed for any of RS232's background tasks
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_BgTaskInit(void)
{
	//empty
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RS232 Command Decode, Invoked whenever we receive a CR or get a full Rx Buffer
//
// Legit. commands formatted like so:
//  C A3E4 Cr -- 4 hex/ascii digits enclosed between 'C' and Cr
//  C A3E4 : 1234 Cr -- 'C' 4 hex digits command ":" 4 hex digits data Cr
//
// Also support "speed-dial" commands of form
//  S (or s) followed by digits
//
// Also support "HELP" command
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_commandDecode(void) {
	Uint16 i;
	Uint16 cmd = 0;
	Uint16 dat = 0;
	bool validChars;
	bool dataReceived = false; // until proven below
	Uint16 rDataLen;

	rDataLen = rdataIndex; // store a local copy so we can zero rdataIndex
	rdataIndex = 0;

	// - - - - - - - - F I R S T   L O O K   F O R   " H E L P "  - - - - - - - -
    // cheap shot: look for leading 'H'/ 'h' followed by 4 chars + Cr
	//  Or discard 1st char, look for 'H'/ 'h' followed by 4 chars + Cr
	if ((((rdata[0] == 'H') || (rdata[0] == 'h'))
			&& (rDataLen == 5))
	 || (((rdata[1] == 'H') || (rdata[1] == 'h'))
					&& (rDataLen == 6)))
	{

	   	// Call Command Interpreter to act on the command
	   	comint_comintHelp();
		return;
	}


	// - - - - - - - - "S"  S P E E D   D I A L   C O M M A N D S - - - - - - - -
	// check to see if this is a "speed-dial" command starting with 'S' or 's', terminated with Cr
	if (((rdata[0] == 0x0053) || (rdata[0] == 0x0073))
	   && (rdata[rDataLen-1] == CR)){
		i  = 1;
		while (i <(rDataLen-1)) {
		  cmd = (cmd << 4) + hexUtil_hexCharToBin(rdata[i]);
		  i++;
		}
		// Need to Ack the command
		r232Out_Command_Ack(cmd,dat,dataReceived,rdata[0]);

	   	// Call Command Interpreter to act on the command
	   	comint_comintSpeedDial(cmd);
		return;
	}

	// - - - - - - - - "T"  T E X T   C O M M A N D S - - - - - - - -
	// check to see if this is a "text" command starting with 'T' or 't',
	// followed by 4 hex digits and a colon (:),
	// followed by a text string, terminated with Cr
	if (((rdata[0] == 0x0054) || (rdata[0] == 0x0074))
	   && (rdata[rDataLen-1] == CR)
	   && (rdata[5] == 0x003A)
	   && (hexUtil_isValidAsciiHexDigits((char*)(rdata+1),4)) ){
		// convert 4 hex asci characters to cmd value
		hexUtil_hexCharsToBin((char*)(&rdata[1]), 4, &cmd);
		i  = 1;

		// Need to Ack the command
		r232Out_Command_Ack(cmd,dat,dataReceived,rdata[0]);

	   	// Call Command Interpreter to act on the command
		comint_comintTextCommand(cmd,(char*)(&rdata[6]), rDataLen-6);
		return;
	}

	// - - - - - - - - "C"  N U M E R I C   C O M M A N D S - - - - - - - -
    // Check for command of form CxxxxCr or Cxxxx:xxxxCr
    // error check: 'C', Cr, Len = 6 or 11 ?
	if (((rdata[0] != 0x0043) && (rdata[0] != 0x0063))
		|| (rdata[rDataLen-1] != CR)
		|| (!((rDataLen == 6) || (rDataLen == 11))) ) {
		// data received doesn't conform to command structure
		// for now, just echo it
		r232Out_Command_Nak((char*)rdata, rDataLen);
		return;
	}

	// error check: 4 valid ascii hex digits?
	validChars = hexUtil_isValidAsciiHexDigits((char*)(rdata+1),4);
	if (!validChars){
		r232Out_Command_Nak((char*)rdata, rDataLen);
	    return;
	}

    // go ahead and convert Ascii hex command digits to binary
	// set task to acknowledge valid command
	for (i=0;i<4;i++) {
       cmd <<= 4;
       cmd += hexUtil_hexCharToBin(rdata[i+1]);
	}

    // extra checks for longer command -- ":" & valid hex chars for data
	if (rDataLen == 11){
		if (rdata[5] != 0x003A) {     //":"
			r232Out_Command_Nak((char*)rdata, rDataLen);
		    return;
		}

		// error check: 4 valid ascii hex digits?
		validChars = hexUtil_isValidAsciiHexDigits((char*)(rdata+6),4);
		if (!validChars){
			r232Out_Command_Nak((char*)rdata, rDataLen);
		    return;
		}

	    // go ahead and convert Ascii hex data digits to binary
		// set task to acknowledge valid command
		for (i=0;i<4;i++) {
	       dat <<= 4;
	       dat += hexUtil_hexCharToBin(rdata[i+6]);
		}
		dataReceived = true;
	}

	rs232_setCommandAck(cmd, dat, dataReceived); // store values for use in rs232_Command_Ack()
	                                             // message back out RS232
	r232Out_Command_Ack(cmd,dat,dataReceived,rdata[0]);

   	// Call Command Interpreter to act on the command
   	comint_comint(cmd,dat,dataReceived);
}

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// rs232_setCommandAck non-public variables
//
// I suspect these variables and their accessor are not required any more.
// I think they were used when we had a task-mgr task to perform the Ack.
// Now I think they are only called/accessed from rs232_commandDecode( ) above,
// and it probably isn't necessary for that. -- DH
//
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
Uint16 commandAckCommandCode;
Uint16 commandAckDataWord;
bool   commandAckDataWordReceived;
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessor for rs232_setCommandAck non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
void rs232_setCommandAck(Uint16 commandCode,Uint16 dataWord, bool dataWordReceived){
	commandAckCommandCode = commandCode;
	commandAckDataWord = dataWord;
	commandAckDataWordReceived = dataWordReceived;
}
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Transmit a long text string out the RS232
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_BgTask_ooad(void)
{
	char msgOut[64];
	char* ptr;

	// newer method, using circular-buffer output manager
    ptr = strU_strcpy(msgOut,"\r\n\nOutside of a dog, a book is man's best friend! \r\n");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

	// old method, pointing transmitter ptr directly to the message
    //sdata = ooad;
    //sdataCount = 52;
    //TurnOnTx();
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Transmit a long text string out the RS232
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_BgTask_ts3StartUp(void)
{
	char msgOut[64];
	char* ptr;

	// newer method, using circular-buffer output manager
    ptr = strU_strcpy(msgOut,"\r\n\nTS3 Starting Up \r\n");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

}


//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Run this background task every time we complete a transmission.
//  If we have more to transmit, he can queue up the next line.
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_BgTask_TxDone(void) {
	// set a flag to run a task in the background,
	// see if there is more to transmit from the r232Out Circular Buffer

	taskMgr_setTaskRoundRobin(TASKNUM_r232Out_circBufOutput,0);

}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Called from Command Interpreter
//  Reports Baudrate, Parity, Char-length & stop Bits
//  as read from SCICCR, SCIHBAUD & SCILBAUD
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_reportBaudParityEtc(void){
	char msgOut[48];
    char *ptr;
    Uint16 brr;
    Uint16 charBits;
    Uint16 stopBits;
    Uint32 calcBaud;
    Uint32 tableBaud;
    bool tableLookUpSuccess;

    // extract baud rate values from hardware registers
    brr = (SciaRegs.SCIHBAUD << 8) & 0xFF00;
    brr |= SciaRegs.SCILBAUD;
    // compute actual effective baud rate (non-ideal)
    calcBaud = 4687500L / (brr + 1); // LSPCLK / ((brr + 1) * 8)
                                     // LSPCLK = 37.5MHz, = 37500000L
                                     // LSPCLK / 8 = 4687500L
    // use a table look up to determine ideal baud rate
    tableLookUpSuccess = rs232_getBaudRateFromBrrValue(brr, &tableBaud);

    charBits = (SciaRegs.SCICCR.all & 0x0007) + 1;
    if (SciaRegs.SCICCR.all & 0x0080) {
    	stopBits = 2;
    } else {
    	stopBits = 1;
    }

    // now format it into display buffer
    // first line like: "9600-N-8-1"
    ptr = strU_strcpy(msgOut,"  ");
    if (tableLookUpSuccess == true) {
    	ptr = hexUtil_bin32ToDecAsciiCharsZeroSuppress(ptr,tableBaud);
    } else {
    	ptr = strU_strcpy(msgOut,"?????");
    }
    if ((SciaRegs.SCICCR.all & 0x0020) == 0) {
       ptr = strU_strcpy(ptr,"-N-");
    } else if ((SciaRegs.SCICCR.all & 0x0040) == 1) {
        ptr = strU_strcpy(ptr,"-E-");
    } else {
        ptr = strU_strcpy(ptr,"-O-");
    }
    *(ptr++) = (char)(charBits + 0x0030);
    ptr = strU_strcpy(ptr,"-");
    *(ptr++) = (char)(stopBits + 0x0030);
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

    // second line
    ptr = strU_strcpy(msgOut,"  brr=0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,brr);
    ptr = strU_strcpy(ptr,", Actual Bd=");
    ptr = hexUtil_bin32ToDecAsciiCharsZeroSuppress(ptr,calcBaud);
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

}
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Called from Command Interpreter
//  Command asks us to set new Baudrate, Parity, Char-length, stop Bits
//  for example 9600-N-8-1
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_changeBaudParityEtc(char* params,int length)
{
	char *c;
	Uint32 baud32;
	Uint16 baudRateRegisterValue;
	Uint16 sciccrValue;
	bool success;


	// - - - Baud Rate - - - - - - - - - - - - - - - - - - -
	c = hexUtil_decCharsToBin32(params, &baud32);
	success = rs232_getBrrValueFromBaudRate(baud32,&baudRateRegisterValue);
	if (success != true)
	{
		r232Out_outCharsNT(msg_notLegalBaudRate);
		return;
	}

	// - - - Parity  - - - - - - - - - - - - - - - - - - - -

	// check length to make sure we haven't run out of input string
	success = true; // assume success, until we find otherwise
	if (length > 1 + (Uint16)(c - params)) {
		// yes we have additional characters in input param string
		c++; // skip over separator character
		if ((*c == 'N') | (*c == 'n')) {
			// remember: parity control uses 2 bits 0x40 & 0x20 in SCICCR
			// note: these sciccr values include: No loopback, async mode, idle-line protocol
			sciccrValue = 0x0000; // no parity
		}else if ((*c == 'O') | (*c == 'o')) {
			sciccrValue = 0x0020; // even parity
		}else if ((*c == 'E') | (*c == 'e')) {
			sciccrValue = 0x0060; // even parity
		} else {
			success = false;
		}
	}else {
		success = false;
	}
	if (success == false) {
		r232Out_outCharsNT(msg_notLegalParity);
		return;
	}

	// - - - Character Size  - - - - - - - - - - - - - - - -

	// check length to make sure we haven't run out of input string
	success = true; // assume success, until we find otherwise
	c++; // skip over separator character
	c++;
	if (length > (Uint16)(c - params)) {
		// yes we have additional characters in input param string
		if ((*c > '0') | (*c <= '8')) {
			// remember: parity control uses ls 3 bits in SCICCR
			// and the 3-bit value = char_bits - 1
			sciccrValue |= (*c & 0x000F) - 1; // no parity
		} else {
			success = false;
		}
	}else {
		success = false;
	}
	if (success == false) {
		r232Out_outCharsNT(msg_notLegalCharSize);
		return;
	}

	// - - - Stop Bits  - - - - - - - - - - - - - - - -

	// check length to make sure we haven't run out of input string
	success = true; // assume success, until we find otherwise
	c++; // skip over separator character
	c++;
	if (length > (Uint16)(c - params)) {
		// yes we have additional characters in input param string
		if (*c == '1') {
			// remember: stop bits uses ms bit in SCICCR
			// and the 3-bit value = char_bits - 1
			sciccrValue &= 0x007F; // 1 stop bits
		} else if (*c == '2') {
			sciccrValue |= 0x0080; // 2 stop bits
		} else {
			success = false;
		}
	}else {
		success = false;
	}
	if (success == false) {
		r232Out_outCharsNT(msg_notLegalStopBits);
		return;
	}

	// put our results into globals that will be used in subsequent
	// call to rs232_scia_fifo_init()
	sci_brr = baudRateRegisterValue;
	sci_ccr = sciccrValue;

	// launch a task to perform the change after a time interval
	rs232BaudChangeCounter = 0;
	taskMgr_setTaskRoundRobin(TASKNUM_Rs232BaudChange,0);

}

bool rs232_getBrrValueFromBaudRate(Uint32 baud32,Uint16 *baudRateRegisterValue)
// return true/false depending on whether or not input param baud32
// represents one of the set of discrete baud rates that we are prepared to
// use.  Also return "BRR", the corresponding value to be used in SCIHBAUD & SCILBAUD
// hardware registers.  Note BRR ("baud rate register") is a virtual 16-bit
// value, and you take the MS byte of BRR and write it to SCIHBAUD, and take
// the LS byute of BRR and write it to SCILBAUD.
//
// MAY BE ABLE TO DO THIS MORE EFFICIENTLY USING brrToBaudTable
{
	if (baud32 ==	110	)
			*baudRateRegisterValue = 42613;
	else if (baud32 ==	300	)
			*baudRateRegisterValue = 15624;
	else if (baud32 ==	1200	)
			*baudRateRegisterValue = 3905;
	else if (baud32 ==	2400	)
			*baudRateRegisterValue = 1952;
	else if (baud32 ==	4800	)
			*baudRateRegisterValue = 976;
	else if (baud32 ==	9600	)
			*baudRateRegisterValue = 487;
	else if (baud32 ==	19200	)
			*baudRateRegisterValue = 243;
	else if (baud32 ==	38400	)
			*baudRateRegisterValue = 121;
	else if (baud32 ==	57600	)
			*baudRateRegisterValue = 80;
	else if (baud32 ==	115200	)
			*baudRateRegisterValue = 40;
	else if (baud32 ==	230400	)
			*baudRateRegisterValue = 19;
	else if (baud32 ==	460800	)
			*baudRateRegisterValue = 9;
	else if (baud32 ==	921600	)
			*baudRateRegisterValue = 4;
	else
		return false;

	return true;
}

struct BRR_TO_BAUD {
	Uint16 brr;
	Uint16 fill;
	Uint32 baud;
} brrToBaudTable[] = {
		// BRR  fill  Baud
		{42613,  0,   110L},
		{15624,  0,   300L},
		{3905,   0,   1200L},
		{1952,   0,   2400L},
		{976,    0,   4800L},
		{487,    0,   9600L},
		{243,    0,   19200L},
		{121,    0,   38400L},
		{80,     0,   57600L},
		{40,     0,   115200L},
		{19,     0,   230400L},
		{9,      0,   460800L},
		{4,      0,   921600L}
        };
#define NUM_OF_ENTRIES_IN_BRR_TO_BAUD_TABLE  13

bool rs232_getBaudRateFromBrrValue(Uint16 baudRateRegisterValue, Uint32 *baud32){
// Given baudRateRegisterValue, attempt to look up corresponding baud rate
// in table, and return it.
// Called success = getBaudRateFromBrrValue( . . . )
//
// Need to look at BRR value +/- 1, because Autobaud detection does not adhere to
// these discrete BRR values.

	Uint16 i;
	for(i=0;i<NUM_OF_ENTRIES_IN_BRR_TO_BAUD_TABLE;i++) {
		if ((brrToBaudTable[i].brr >= (baudRateRegisterValue - 1))
		  &&(brrToBaudTable[i].brr <= (baudRateRegisterValue + 1))){
			*baud32 = brrToBaudTable[i].baud;
			return true;
		}
	}

	return false;
}


//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Background task started when we get rs232 request to change baud, parity, etc.
//  Gives a heads up -- rs232 output.
//  Changes baud rate after approximately 3 seconds.
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_BgTask_Rs232BaudChange(void)
{

	if (rs232BaudChangeCounter == 0) {
		r232Out_outCharsNT(msg_changeBaudRateIn3Sec);
		taskMgr_setTaskWithDelay(TASKNUM_Rs232BaudChange,5); // ==> 0.5 Sec
	} else if (rs232BaudChangeCounter == 1) {
		rs232_scia_fifo_init(); // routine performs baud change, etc
		return; // exit here w/out re-launching task
	}
	rs232BaudChangeCounter++;

}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Background task started when we receive a BREAK or ERROR
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_BgTask_Rs232BreakOrError(void)
// Wait, then reinitialize SCI A, then wait, then transmit message.
//
// If I don't include the waits, then the DSP winds up transmitting a bunch
// of garbage characters.  My guess is that TX interrupts are getting generated,
// and I haven't been able to pin it down.  So heuristically, it doesn't
// cause a problem if I put in the waits.  So I can live with that until I have
// more time or insight to apply to the problem.
//
{

	if (afterBreakCounter == 0) {  // got here initially after 0.5 second delay after BREAK
		// sci_brr = ((SciaRegs.SCIHBAUD << 8) & 0xFF00) | SciaRegs.SCILBAUD; // capture current baud rate
		rs232_scia_fifo_init(); // initialize SCIA / RS232
		/* success = */ r232Out_outCharsNT(msg_detectedBreakOrError);
		taskMgr_setTaskWithDelay(TASKNUM_RS232_breakOrError,5); // ==> 0.5 Sec

	} else if (afterBreakCounter == 1) {  // got here after 0.5 second delay after rs232_scia_fifo_init()
 		rs232_start_autobaud_detect();
 		// launch auto-baud detection
		return; // exit here w/out re-launching task
	}

	afterBreakCounter++;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Background task started when we detect receipt of autobaud character
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rs232_BgTask_AckAutobaud(void)
//
{
	if (rs232AckAutobaudCounter == 0) {             // 0.0 sec delay
		/* success = */ r232Out_outCharsNT(msg_AckAutobaud);
 	} else if (rs232AckAutobaudCounter == 1) {      // 0.2 sec delay
 		rs232_reportBaudParityEtc(); // C1005Cr command usually invoked from comint
 	} else if (rs232AckAutobaudCounter == 2) {      // 0.4 sec delay
		/* success = */ r232Out_outCharsNT(msg_Ts3Banner001);
 	} else if (rs232AckAutobaudCounter == 3) {      // 0.6 sec delay
		/* success = */ r232Out_outCharsNT(msg_Ts3Banner002);
 	} else if (rs232AckAutobaudCounter == 4) {      // 0.8 sec delay
		/* success = */ r232Out_outCharsNT(msg_Ts3Banner001);
		return; // exit here w/out re-launching task
 	}

	rs232AckAutobaudCounter++;
	taskMgr_setTaskWithDelay(TASKNUM_RS232_ackAutobaud,2); // keep relaunching the task w/ 0.5 sec delay

}
void rs232_DiagPortalMsg(void){
	/* success = */ r232Out_outCharsNT(msg_Ts3Banner001);
	/* success = */ r232Out_outCharsNT(msg_Ts3Banner002);
	/* success = */ r232Out_outCharsNT(msg_Ts3Banner001);
}

