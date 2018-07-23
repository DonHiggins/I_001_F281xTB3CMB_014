// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     F2Int.H
//         support for FPGA2 to interrupt DSP
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File

#include "Rs232Out.H"
#include "stdbool.h"            // needed for bool data types
#include "HexUtil.H"
#include "StrUtil.H"

#include "TaskMgr.h"
#include "GpioUtil.H"
#include "CanOpen.h"
#include "F2Int.H"
#include "CPLD.H"

enum F2I_STATE f2i_state = F2I_ST_IDLE;
Uint16 f2i_BgTask_count;


void f2i_enable_interrupt (void){
    // Int13 is does NOT go through the PIE interrupt multiplexor, hence there is no
	// need to enable a PIE group for Int13
	// PieCtrlRegs.PIEIER5.all = M_INT1; // code retained as an example

    // Enable CPU INT13
    IER |= M_INT13;
}

// Interrupts that are used in this function are re-mapped to
// ISR functions found within this file.
void f2i_store_int_vectors_in_PIE(void){

	   EALLOW;  // This is needed to write to EALLOW protected registers
	   PieVectTable.XINT13 = &f2i_isr;
	   EDIS;   // This is needed to disable write to EALLOW protected registers
}

void f2i_initialize_interrupt (void){

	XIntruptRegs.XNMICR.bit.SELECT = 1;		// XNMI_XINT13 pin  connected to INT13, not Timer1
	XIntruptRegs.XNMICR.bit.ENABLE = 0;		// 0-> disable XNMI interrupt (but do use INT13)
	XIntruptRegs.XNMICR.bit.POLARITY = 1;	// interrupt generated on rising edge

	GpioU_f2iInit(); 						// Setup the GPIO to use IO pin E2 As XNMI_XINT13 interrupt
	f2i_state = F2I_ST_IDLE;

	f2i_BgTask_count = 0;
}

interrupt void f2i_isr(void){

	   // Since Int13 is not multiplexed thru the PIE interrupt controller, we don't
	   // need to "acknowledge" interrupt to receive more interrupts from the PIE group.
 	   //  PieCtrlRegs.PIEACK.all = PIEACK_GROUP5; // code retained as an example

		//Queue a task to run in bacground -- following task just outputs to RS232
		//for testing, this gives us some indication of interrupt activation w/out
		//having to stop the code in debug.
		taskMgr_setTask(TASKNUM_F2Int_SSEnc); // re-queue this task


	   	switch(f2i_state){
	    case F2I_ST_IDLE: // Shouldn't get here
	        break;

	    case F2I_ST_DUMMY1: // Dummy Task 1, Toggle LED's until we are turned off
	    	   //Need something here to toggle an I/O so we can see the effect of the interrupt
//	    	   GpioDataRegs.GPBTOGGLE.all = 0x000F; // toggle all 4 LS bit of GPIOB
	    	   break;

	    default:
	   	break;
	   }

	   // Re-Enable CPU INT13
	   IER |= M_INT13;
}

// ==========================================================================
//    T A S K S for running features in background
// ==========================================================================

// Parameters from Host via CAN
// Pos_1 & 2 are encoder absolute position data
// MS bit (0x80) of Pos_1 is Alarm Bit,
// Next bit of Pos_1 (0x40) is LS bit of Position Data
// Position data bits continue LS to MS
// Zero fill, on the right, any unused bits of Pos_1 & 2
// This is the format we hand it to the FPGA for EnDat transmission
Uint16 f2i_SSEnc_Pos_1;
Uint16 f2i_SSEnc_Pos_2;
Uint16 f2i_SSEnc_Num_Pos_Bits;
Uint16 f2i_SSEnc_Alarm_Bit;  // LS Bit is Alarm Bit,
Uint32 f2i_SSEnc_Pos_32; // Absolute position, right justified, not bit-reversed
Uint16 f2i_SSEnc_CRC_5;
enum ENDAT_MODE previousMode;

void f2i_SSEnc_init(void){
// Called from main.c at startup, make sure we have default values that make sense for our task.
// If we get a phase-detect request from a drive, we can respond to the three EnDat queries,
// communicating (1) resolution / number of bits in position data, and (2) absolute position.
// Of course host can use CAN commands to override these default values.
	f2i_SSEnc_Num_Pos_Bits = 13;
	f2i_SSEnc_Alarm_Bit = 0;
	f2i_SSEnc_Pos_32 = 0x00000123;
	f2i_SSEnc_reverse_position(); // reformat 32-bit position for FPGA
	// Next we calculate a CRC_5 on this
	f2i_SSEnc_CRC_5 = f2i_CRC5(f2i_SSEnc_Pos_1, f2i_SSEnc_Pos_2,(f2i_SSEnc_Num_Pos_Bits + 1));
}

void f2i_BgTask_SSEnc(void){
// Got here after receiving an interrupt, interrupt handler fired off this background task
// Interrupt signals that FPGA2's Sin Encoder simulator received a data packet (from a drive)
// Here we examine the data packet, formulate a response, and send that back to FPGA2 to xmit.
	char msgOut[96];
    char *ptr;
    Uint16 status;
    Uint16 data_hi;
    Uint16 data_low;
    volatile Uint16 * write_1st_16 = CPLD_F2_XA(FPGA2_WRITE_SSE_DATA_1ST_16);
    volatile Uint16 * write_2nd_16 = CPLD_F2_XA(FPGA2_WRITE_SSE_DATA_2ND_16);
    volatile Uint16 * write_crc5 = CPLD_F2_XA(FPGA2_WRITE_SSE_CRC5);
    enum ENDAT_MODE mode;
    enum ENDAT_MRS mrs;
    Uint16 parsingStatus;
    Uint16 param_1;
    Uint16 param_2;
    Uint16 param_3;

    // read this info from FPGA
    status = *CPLD_F2_XA(FPGA2_READ_SSE_STATUS);
    data_hi = *CPLD_F2_XA(FPGA2_READ_SSE_1ST_16);
    data_low = *CPLD_F2_XA(FPGA2_READ_SSE_2ND_16);

    // Decode the incoming data, according to EnDat
    mode = (enum ENDAT_MODE)((data_hi >> 8) & 0x6F);
    mrs = (enum ENDAT_MRS)(data_hi & 0xFF);

    parsingStatus = 0;

	switch(mode){
	case EM_SELECTION_OF_MEM_AREA: // "Select memory area to address in next packet"
		if (mrs == EMRS_MEM_ALLOC_OEM_PARAMS) {
			// for right now, we hardcode the proper response to the first packet
			*write_1st_16	= 0xA100;
			*write_2nd_16	= 0x0000;
			*write_crc5		= 0x181C; //0x18 = 24 bits, 0x1C = 5'b11100 == crc5(0xA10000)
		    parsingStatus = 1; // value displayed as diagnostic
		}
        break;

	case EM_ENCODER_TRANSMIT_PARAM: // "Encoder Transmit Parameter"
		// technically, we should check first to insure previousMode == EM_SELECTION_OF_MEM_AREA
		if (mrs == EMRS_BITS_OF_POSITION) {

//		- - -   H I S T O R I C A L   - - -
//			// for right now, we hardcode a proper response for 13-bit encoder
//			// later we want PC to be able to specify number of bits
//			*write_1st_16	= 0x0D80;	// echo_mode (0D) + ms_8_bits_data (80)
//			*write_2nd_16	= 0x0D00;	// ls_8_bits_data (0D) + dont_care (00)
//			*write_crc5		= 0x1815; //0x18 = 24 bits, 0x15 = 5'b10101 == crc5(0x0D800D)

			// construct data values to respond to drives query about how many bits do we use
			// to communicate position, and send data to FPGA2 / SSEnc subsystem
			param_1 = 0x0D00;								 // echo the mode command (0D)
			param_1 |= 0x0080;								 // + always set 0x0080 HI
			param_1 |= ((f2i_SSEnc_Num_Pos_Bits>>8)&0x007F); // + MS 7 of 15 bits of param: f2i_SSEnc_Num_Pos_Bits
			*write_1st_16	= param_1;

			param_2 = ((f2i_SSEnc_Num_Pos_Bits<<8)&0xFF00);	// LS 8 of 15 bits of the param: f2i_SSEnc_Num_Pos_Bits
															// dont_care (00)
			*write_2nd_16 = param_2;

			param_3 = f2i_CRC5(param_1, param_2, 24);		// ls 5 bits is CRC_5
			param_3 |= 0x1800;								// upper 8 bits, 0x18 = 24d, is # of bits in this transmission
			*write_crc5	= param_3; 																	   // ls 5 bits is CRC_5

			parsingStatus = 2; // value displayed as diagnostic
		}
        break;

	case EM_ENC_TRANSMIT_POS_VALUE: // "Encoder Transmit Parameter"

//		- - -   H I S T O R I C A L   - - -
//		// for right now, we hardcode a proper response for 13-bit position request
//		// later we want to do this dynamically, where position and # of bits are parameters
//		*write_1st_16	= 0x47D0; // status=0 + 13_bit_pos_lsb_first
//		*write_2nd_16	= 0x0000; // (don't care)
//		*write_crc5		= 0x0E01; //0x0E = 14 bits, 0x1C = 5'b00001 == crc5(0xA10000)

		// When host sent new position value via CAN, we formatted it ready for transmission
		// and we calculated proper CRC for it, based on # of bits and alarm-flag.
		// Ao at this point we are ready to send it to the FPGA2 / SSEnc
		*write_1st_16 = f2i_SSEnc_Pos_1;
		*write_2nd_16 = f2i_SSEnc_Pos_2;
		param_3 = ((f2i_SSEnc_Num_Pos_Bits + 1)<<8)&0xFF00; // upper 8 bits, is # of bits in this transmission
		param_3 |= f2i_SSEnc_CRC_5;							// ls 5 bits is CRC_5
		*write_crc5	= param_3;

		parsingStatus = 3; // value displayed as diagnostic
        break;

     default:
    	break;
    }

	previousMode = mode; // store this for next time we come through here

    // Just for diagnostics, echo the info to the RS232 output
    f2i_BgTask_count +=1;
    ptr = strU_strcpy(msgOut,"F2I_Bg mode: 0x");
    ptr = hexUtil_binTo2HexAsciiChars(ptr,(Uint16)mode);
    ptr = strU_strcpy(ptr," data 0x");
    ptr = hexUtil_binTo2HexAsciiChars(ptr,data_hi); // just ls 8 bits (ms 8 is mode)
    ptr = strU_strcpy(ptr,",");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,data_low);
    ptr = strU_strcpy(ptr,", stat:");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,status);
    ptr = strU_strcpy(ptr,", parse:");
    ptr = hexUtil_binTo2HexAsciiChars(ptr,parsingStatus);
    ptr = strU_strcpy(ptr,"\n\r");
    /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

}

// ==========================================================================
//    U T I L I T I E S
// ==========================================================================

Uint16 f2i_CRC5(Uint16 data_1, Uint16 data_2,Uint16 num_bits) {
// 5-bit CRC for EnDat communications.
// Data bit-stream to run CRC on is max 32 bits, in data_1 and data_2,
// with first transmitted bit MSB (0x8000) of data_1
// Num_bits is 1 to 32
// For ex, Echoing 8-bit MRS code 0x0D & 16-bit data 0x801F
//   ==> num_bits = 24, data_1 = 0x0D80, data_2 = 0x1F00
//   by convention, but not required, we 0-fill unused bits on right of data_2
// Another example, writing a "13-bit" position value
//   position = 0x00000123 w/ Alarm_bit = 0 . . .
//   ( left shift position 1 bit, and insert alarm_bit 0, on right ==> 0x0000246)
//   ( now bit reverse that value because alarm bit is first bit to xmit, followed by LS bit of position)
//   ( ==> 0x62400000 ==> data_1 = 0x6240, data_2 = 0x0000, num_bits = 14 (== 13-position + 1-alarm))

	Uint16 EnDat_CRC_Reg;
	Uint16 i;
	Uint16 j;
	Uint16 data_byte[5];
	Uint16 count;

	EnDat_CRC_Reg = 0x1F;
	count = num_bits;

	//Lets peel the data off, into bytes
	data_byte[0] = (data_1 >> 8) & 0x00FF;
	data_byte[1] = data_1 & 0x00FF;
	data_byte[2] = (data_2 >> 8) & 0x00FF;
	data_byte[3] = data_2 & 0x00FF;
	data_byte[4] = 0;

	for (j=0;j<5;j++){
		for (i=0;(i<8)&(count>0);i++){
			count--;
			EnDat_CRC_Reg = (EnDat_CRC_Reg << 1) & 0x3E;
			if (((EnDat_CRC_Reg >> 5) ^ (data_byte[j] >> 7))& 0x0001) {
				EnDat_CRC_Reg = EnDat_CRC_Reg ^ 0x000B;
			}
			data_byte[j] = (data_byte[j] << 1) & 0x00FE;
		}
	}
	return ((EnDat_CRC_Reg & 0x001F) ^ 0x001F);
}

void f2i_SSEnc_reverse_position(void){
// Take the 32-bit absolute encoder position value in f2i_SSEnc_Pos_32, and reformat it.
// Start with the alarm bit, followed by position bits -- LS bit first
// Then shift it to left-justify the whole thing so Alarm bit is 0x80000000.
// Zero fill bits past the actual number of bits in our position data, f2i_SSEnc_Num_Pos_Bits,
// which should be from 1 to 31.
// Result is left in 2 16-bit variables, f2i_SSEnc_Pos_1 & 2, ready to hand off
// to FPGA2 SSEnc code.

	Uint16 i;
	Uint32 reversedPos;
	Uint32 Pos_32;

	reversedPos = f2i_SSEnc_Alarm_Bit & 0x0001;

	Pos_32 = f2i_SSEnc_Pos_32;
	for (i=f2i_SSEnc_Num_Pos_Bits;i>0;i--){
		reversedPos = (reversedPos << 1) & 0xFFFFFFFE;
		reversedPos = reversedPos | (Pos_32 & 0x00000001);
		Pos_32 = Pos_32 >> 1;
	}
	for (i=(31 - f2i_SSEnc_Num_Pos_Bits);i>0;i--){
	reversedPos = (reversedPos << 1) & 0xFFFFFFFE;
	}
	// put results back in persistant static locations
	f2i_SSEnc_Pos_1 = (Uint16)(reversedPos >> 16);
	f2i_SSEnc_Pos_2 = (Uint16)(reversedPos & 0x0000FFFF);
}

// ==========================================================================
//    C A N   Interface
// ==========================================================================

enum CANOPEN_STATUS f2i_recv_Pos_32_from_Host(const struct CAN_COMMAND* can_command, Uint16* data){
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// and for this command 0x2050.E, datapointer is : &f2i_SSEnc_Pos_32
	//
	// The host has sent us 32 bits representing Absolute Encoder position.
	// We save it and also convert it into the format sent to FPGA2, starting w/ alarm bit
	// followed by position bits, LS first.
	// We also calculate the proper CRC_5

	Uint16 *dest;
	dest = (Uint16*)can_command->datapointer;
	*dest++ = *(data+2); // MboxC
	*dest = *(data+3);   // MboxD

	// After storing new value, we need to reformat it
	// Take the 32-bit absolute encoder position value in f2i_SSEnc_Pos_32, and reformat it.
	// Start with the alarm bit, followed by position bits -- LS bit first
	// Then shift it to left-justify the whole thing so Alarm bit is 0x80000000.
	// Zero fill bits past the actual number of bits in our position data, f2i_SSEnc_Num_Pos_Bits,
	// which should be from 1 to 31.
	// Result is left in 2 16-bit variables, f2i_SSEnc_Pos_1 & 2, ready to hand off
	// to FPGA2 SSEnc code.

	f2i_SSEnc_reverse_position(); // from f2i_SSEnc_Pos_32 into f2i_SSEnc_Pos_1, f2i_SSEnc_Pos_2

	// Next we calculate a CRC_5 on this
	f2i_SSEnc_CRC_5 = f2i_CRC5(f2i_SSEnc_Pos_1, f2i_SSEnc_Pos_2,(f2i_SSEnc_Num_Pos_Bits + 1));

	return CANOPEN_NO_ERR;
}
