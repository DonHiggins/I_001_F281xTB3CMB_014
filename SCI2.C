// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     SCI2.C
//
//   RS232 / RS485 via DSP SCI#2 (Serial Channel) and DB9 connector on TB3IOMC
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "CanOpen.h"
#include "StrUtil.H"
#include "CPLD.H"
#include "SCI2.h"

extern struct MULTI_PACKET_BUF multi_packet_buf; // defined in CanOpen.C
struct SCI2_PARAMS sci2 =
	{/*SCICCR*/   SCICCR_ONE_STOP_BIT | SCICCR_NO_PARITY | SCICCR_PARITY_DISABLE | SCICCR_8_BIT_CHAR,
	 /*SCIHBAUD*/ SCIBAUD_9600,
	  0,0,0,0,0};

struct SCI2_BUF sci2_Tx_Buf = {768,0,0}; // max 768 characters in buffer
struct SCI2_BUF sci2_Rx_Buf = {768,0,0}; //  count of chars = 0, 1st char = 0

// --  Baud Rate explanation -- from RS232.C
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


enum CANOPEN_STATUS sci2_init(const struct CAN_COMMAND* can_command, Uint16* data){
// Called when PC sends CAN Index 0x2031.5 requesting
//  initialization of RS232/485 hardware.  This mainly
//  takes values previously stored in the rs232 struct
//  by previous CAN commands and applies them to hardware
//  registers.
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 temp;

	sci2.sciccr &= ~(0x08); // insure Addr/Idle Mode bit is 0

	ScibRegs.SCICTL2.all = SCICTL2_RX_INT_DISABLE | SCICTL2_TX_INT_DISABLE;
	ScibRegs.SCICTL1.all = SCICTL1_SW_RESET | SCICTL1_TX_ENABLE | SCICTL1_RX_ENABLE;
	ScibRegs.SCICCR.all  = sci2.sciccr;
	ScibRegs.SCIHBAUD = (sci2.scibaud >> 8);
	ScibRegs.SCILBAUD = (sci2.scibaud & 0xFF);
	ScibRegs.SCIPRI.all =  SCIPRI_SCITX_LOW_PRI | SCIPRI_EMULATOR_NO_SUSPEND;
	ScibRegs.SCICTL1.all = SCICTL1_SW_NOT_RESET | SCICTL1_TX_ENABLE | SCICTL1_RX_ENABLE;

 	// initialize driver software
	sci2.tx_status = 0;
	sci2.tx_previous_status = 0;
	sci2.tx_index_txing_from_buf = 0;
	sci2.rx_err_status = 0;
	sci2_Rx_Buf.count_of_bytes_in_buf = 0;
	sci2_Tx_Buf.count_of_bytes_in_buf = 0;

	// Access CPLD to set ~TX2_BUF_ENA low
	temp = *CPLD_XINTF_ADDR(TBIOM_TX2_BUF_ENA); // read CPLD to set ~TX2_BUF_ENA low
	                                            // enables buffer from TB3CM.TXD2 to LTC1387I on TB3IOM

	//Access to CPLD to set ~RS_232 according to sci2.rs232_not_rs485
	if (sci2.rs232_not_rs485) {
		temp = *CPLD_XINTF_ADDR(TBIOM_RS_232); // read CPLD to set ~TX2_BUF_ENA low -> RS232
	} else {
		*CPLD_XINTF_ADDR(TBIOM_RS_232) = temp; // write CPLD to set ~TX2_BUF_ENA hi -> RS485
	}
	return CANOPEN_NO_ERR;
}

void sci2_rx_tx(void)
{
// This is called periodically from the Main Loop
// Read in any bytes received via the RS232/485 port
// Transmit out any bytes ready to transmit.

	char c;

	if(sci2.tx_status)
	{
		if (sci2.tx_previous_status == 0)
		{
			// status just went from 0 to 1
			// start transmitting from beginning of buffer
			sci2.tx_index_txing_from_buf = 0;
			if (sci2.idle_line) {
				// set TXWAKE high to tell hardware to transmit 11-bit idle signal
				ScibRegs.SCICTL1.all = SCICTL1_TXWAKE | SCICTL1_SW_NOT_RESET | SCICTL1_TX_ENABLE | SCICTL1_RX_ENABLE;
				ScibRegs.SCITXBUF = 0;	 // write a dummy character to Tx Buffer
			}
		}

		if (sci2.tx_index_txing_from_buf >= sci2_Tx_Buf.count_of_bytes_in_buf) {
			sci2.tx_status = 0; // done transmitting

		} else if (ScibRegs.SCICTL2.all & 0x80) { // if TXRDY, means: if Tx Buffer Register is empty
    	    ScibRegs.SCITXBUF = sci2_Tx_Buf.buff[sci2.tx_index_txing_from_buf++];	 // write a character to Tx Buffer
		}
	}
	sci2.tx_previous_status = sci2.tx_status;


	// sci2 Receive
	if (sci2_Rx_Buf.count_of_bytes_in_buf >= 768) {
		sci2.rx_err_status = 0x08; // "Overrun error"
		return; // buffer full
	}

	// Clear any reported Receiver Errors
	if (ScibRegs.SCIRXST.all & SCIRXST_RX_ERROR) {
		sci2.rx_err_status = ScibRegs.SCIRXST.all;
		sci2_Rx_Buf.buff[sci2_Rx_Buf.count_of_bytes_in_buf++] = (char)ScibRegs.SCIRXBUF.all;
		return;
	}

	// If Idle-line protocol is in effect,
	//  and we are receiving the first character of a message
	//  then discard the character if it was not preceeded by an
	//  idle-line condition

	if((sci2_Rx_Buf.count_of_bytes_in_buf == 0) &&
        (sci2.idle_line == 1) &&
           (ScibRegs.SCIRXST.all & SCIRXST_RX_RDY) &&
              (!(ScibRegs.SCIRXST.all & SCIRXST_RX_WAKE)) )
	{
		sci2_Rx_Buf.buff[0] = (char)ScibRegs.SCIRXBUF.all;  // Clear char from the port & discard

		//asm("_Brk_Pt_W:	NOP");         // public symbol useful for breakpoint, NOT destination for goto or call
                // Testing Note:  I was able to get to this breakpoint to prove to myself that
				//  the code is properly implementing the "idle-line" protocol when it is receiving.
				//  Here's what I did:  I used Hyper-Terminal on the PC and I used the Transfer/"Send-Text-File"
				//  function to transmit a stream of more than 1000 characters. I used my "Test_RS232.xls" program
				//  to communicate with the NTB and sent it the "Initialize RS232" command durring the
				//  transmission from Hyper-Terminal.  One additional detail -- I had to slow the transmission
				//  speed to 1200 baud -- otherwise Hyper-term leaves too much space between characters
				//  so that RX_WAKE gets set between each character.  Note, you may have to try this a couple
				//  times to get to the breakpoint, since even at 1200 Bd, hyper-term still leaves considerable
				//  spaces between some characters. -- DH 1/2/'03
		return;
	}

	// If a character is received, save it in memory
	// But not if our receive buffer is already full
	   if (ScibRegs.SCIRXST.all & SCIRXST_RX_RDY) {  // check the Receiver Ready Flag
		   c = (char)ScibRegs.SCIRXBUF.all;  // read the received char from the port
			if (sci2_Rx_Buf.count_of_bytes_in_buf < sci2_Rx_Buf.max_char_in_buf ) {
		       sci2_Rx_Buf.buff[sci2_Rx_Buf.count_of_bytes_in_buf++] = c;
	   }
	}
}

enum CANOPEN_STATUS sci2_recvTxBuf(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is our destination char buff sci2_Tx_Buf.buff
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// We get here each time the CanOpen module receives a complete multi-packet data set
	// for our index.subindex, data is in
	// extern struct MULTI_PACKET_BUF multi_packet_buf; // defined in CanOpen.C
	// Place received characters (bytes) into sci2_Tx_Buf.buff starting at offset
	// sci2_Tx_Buf.count_of_bytes_in_buf, and incrementing it as
	// we go, taking care not to overrun the end of the buffer as indicated by
	// sci2_Tx_Buf.max_char_in_buf

	struct MULTI_PACKET_BUF *mpb;
	Uint16 countCharIn;
	char *src;
	Uint16 i;

	sci2_Tx_Buf.count_of_bytes_in_buf = 0; // This CAN command, starts with an empty sci_Tx_Buf

	// for right now, lets display the char string on our diagnostic output
	//mpb = (struct MULTI_PACKET_BUF *)(data-2);
	//diagRs232recvMultiPacket(mpb);

	mpb = &multi_packet_buf;

	//append the received characters into sci2_Tx_Buf.buff, don't overrun it
    countCharIn = mpb->count_of_bytes_in_buf;
    src = mpb->buff;
	for (i=0;i<countCharIn;i++){
		if (sci2_Tx_Buf.count_of_bytes_in_buf >= sci2_Tx_Buf.max_char_in_buf) break; // exit "for" loop
		sci2_Tx_Buf.buff[(sci2_Tx_Buf.count_of_bytes_in_buf++)] = *(src++);
	}

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS sci2_recvTxBufAppend(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is our destination char buff sci2_Tx_Buf.buff
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// We get here each time the CanOpen module receives a complete multi-packet data set
	// for our index.subindex, data is in
	// extern struct MULTI_PACKET_BUF multi_packet_buf; // defined in CanOpen.C
	// APPEND received characters (bytes) into sci2_Tx_Buf.buff starting at offset
	// sci2_Tx_Buf.count_of_bytes_in_buf, and incrementing it as
	// we go, taking care not to overrun the end of the buffer as indicated by
	// sci2_Tx_Buf.max_char_in_buf

	struct MULTI_PACKET_BUF *mpb;
	Uint16 countCharIn;
	char *src;
	Uint16 i;

	// This CAN command, APPENDS to whatever is already in the sci_Tx_Buf
	//sci2_Tx_Buf.count_of_bytes_in_buf = 0;

	// for right now, lets display the char string on our diagnostic output
	//mpb = (struct MULTI_PACKET_BUF *)(data-2);
	//diagRs232recvMultiPacket(mpb);

	mpb = &multi_packet_buf;

	//append the received characters into sci2_Tx_Buf.buff, don't overrun it
    countCharIn = mpb->count_of_bytes_in_buf;
    src = mpb->buff;
	for (i=0;i<countCharIn;i++){
		if (sci2_Tx_Buf.count_of_bytes_in_buf >= sci2_Tx_Buf.max_char_in_buf) break; // exit "for" loop
		sci2_Tx_Buf.buff[(sci2_Tx_Buf.count_of_bytes_in_buf++)] = *(src++);
	}

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS sci2_sendRxBuf(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is char buff in MULTI_PACKET_BUF
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// We get here each time the CanOpen module receives a request to start a multi-packet
	// data set SEND back to the host for our index.subindex data destination is in
	// extern struct MULTI_PACKET_BUF multi_packet_buf; // defined in CanOpen.C
	// Extract characters (bytes) from sci2_Rx_Buf.buff starting at index 0
	// copy to destination taking care not to overrun the end of the destination buffer
	// as indicated by multi_packet_buf.max_char_in_buf

    Uint16 countCopied;
    Uint16 countRemaining;
    Uint16 i;
    char *sptr;
    char *dptr;

	// if they are asking for characters beyond what we have in the buffer, return an error
	// unless it's just that we have an empty buffer to start with
	if (sci2_Rx_Buf.count_of_bytes_in_buf < 1) {
    	return CANOPEN_SCI2_RX_BUF_EMPTY_ERR; // they are asking for Rx data, but Rx buff is empty
	}

	countCopied = copyDataToMultiPacketBuf(sci2_Rx_Buf.buff, sci2_Rx_Buf.count_of_bytes_in_buf);

	if ((countCopied == 0) && (sci2_Rx_Buf.count_of_bytes_in_buf != 0)) {
    	return CANOPEN_SCI2_RX_002_ERR; // we can't copy to MultiPacketBuf, maybe it is is in use
    }

	// flush sci2_Rx_Buf
	// If we didn't send the whole Rx buffer, then slide unsent chats to the front of the buffer
	if (countCopied == sci2_Rx_Buf.count_of_bytes_in_buf) {
	   sci2_Rx_Buf.count_of_bytes_in_buf = 0;
	} else {
		countRemaining = sci2_Rx_Buf.count_of_bytes_in_buf - countCopied;
		sptr = &(sci2_Rx_Buf.buff[countCopied]);
		dptr = &(sci2_Rx_Buf.buff[0]);
		for (i=0;i<countRemaining;i++) {
			*(dptr++) = *(sptr++);
		}
		sci2_Rx_Buf.count_of_bytes_in_buf = countRemaining;
	}

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS sci2_xmit_test(const struct CAN_COMMAND* can_command, Uint16* data){
	// Drop some text in the transmit buffer to test Sci2 Rs232 / Rs485
//	char *ptr;

	sci2_Tx_Buf.count_of_bytes_in_buf = 26;
//    ptr = strU_strcpy(sci2_Tx_Buf.buff,"Testing SCI2 serial xmit\n\r");
    strU_strcpy(sci2_Tx_Buf.buff,"Testing SCI2 serial xmit\n\r");

	return CANOPEN_NO_ERR;
}

