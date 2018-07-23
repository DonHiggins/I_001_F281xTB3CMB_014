// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     CanFile.C
//
//   CAN Given that we just received a multi-packet data set
//       Here are applications that interpret that data set as all or part
//       of a file transfer, such as used to reload code into FPGA or DSP.
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//
// - - - - - CanFile Operations Made Simple - - - - - - - - - - - - - - - - - - - -
//
// This was originally a "Dummy" demonstration app for testing CAN Non-Expedited SDO
// function, AKA byte-stream multi-packet CAN messaging.
// Coincidentally it happens to work just fine as written, to also handle
// transmission of the I2C EEProm images for reading and writing to the EEProms
// on each of our TB3 circuit modules.
// Here's how you operate it:
//
// There are 3 Uint16 values you set or read from the host via expedited SDO.
//	They manage transfer of data in and out of CanFile's intewrnal dummyFileBuf[].
//
// FileBufInCharCount 0x204A.01:
//	This # is an index into the dummyFileBuf in the CanFile application in the  Test Station.
//	This # tells how many characters are held in that buffer, if you want to read from
//	the buffer.
//	This # also tells where in the buffer to store the next character that the the CanFile
//	application receives from us when we do the Send_Append_Str function below.
//	If we send 0 to the InCharCt, effectively we are telling the CanFile application to
//	discard the previous contents of the dummyFileBuf , and then anything we send with
//	the Send_Append_Str function (below) will be stored at the start of the buffer.
//
// FileBufOutCharCount 0x204A.03:
//	This # is also an index into the dummyFileBuf in the CanFile application in the  Test Station.
//	This # tells the CanFile app where to start grabbing characters out of the buffer to
//	send back to us when we do the Read_String function (below).
//	And after we do the Read_String function, the CanFile app advances this # by the number
//	of characters it has sent so that successive Read_String function calls will read
//	successive blocks of text.
//	If we send 0 to the OutCharCt, effectively we are telling the CanFile application
//	to start reading at the beginning of the buffer the next time we do a Read_String function.
//
// FileBufOutPacketSize 0x204A.04:
//	Max # of characters test station will send to host in Read_String operation, below
//
// There is a single set of read and write non-expedited SDO operations that perform
// the multi-packet data transfer configured in the 3 values above:
//
// Send_Append_Str(host function): canF_recvDummyFile: 0x204A.02
// 	We get here each time the CanOpen module receives a complete multi-packet data set
//	from the host for our index.subindex.
//	Append received characters (bytes) into dummyFileBuf[] starting at
//	offset dummyFileBufInCharCount, and incrementing dummyFileBufInCharCount as
//	we go, taking care not to overrun the end of dummyFileBuf[]
//
// Read_String(host function): canF_sendDummyFile: 0x204A.02
//	We get here each time the CanOpen module receives a request to start a multi-packet
//	data set SEND back to the host for our index.subindex
//	Start at offset "dummyFileBufOutCharCount" into the dummyFileBuf
//	Copy "dummyFileBufOutPacketSize" characters to the MULTI_PACKET_BUF
//	increment "dummyFileBufOutCharCount" by "dummyFileBufOutPacketSize"



// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
//#include "CanComm.h"
#include "Rs232Out.H"
//#include "stdbool.h"            // needed for bool data types
#include "HexUtil.H"
#include "StrUtil.H"
#include "CanOpen.h"
#include "CanFile.h"

extern struct MULTI_PACKET_BUF multi_packet_buf;

Uint16 canF_diagOnOff;

Uint16 dummyFileBufInCharCount;
Uint16 dummyFileBufOutCharCount;
Uint16 dummyFileBufOutPacketSize;

#define DUMMY_FILE_BUF_SIZE 256
char dummyFileBuf[DUMMY_FILE_BUF_SIZE];

void canF_zeroDummyFileBufCounts(void) {
// Used, for ex, by I2cee CAN routines before filling CanFile buffer from EEProm contents.
	dummyFileBufInCharCount = 0;
	dummyFileBufOutCharCount = 0;
}

void canF_zeroDummyFileBufOutCount(void) {
// Used, for ex, by I2cee CAN routines before filling CanFile buffer from EEProm contents.
	dummyFileBufOutCharCount = 0;
}

void canF_appendIntoDummyFileBuf(char *src, Uint16 charCount) {
//append the received characters into dummyFileBuf, don't overrun it
//This routine is used by other applications that use CanFile
//to transfer a block of data between PC and Test Station.
	Uint16 i;
	for (i=0;i<charCount;i++){
		if (dummyFileBufInCharCount >= DUMMY_FILE_BUF_SIZE) break; // exit "for" loop
		dummyFileBuf[dummyFileBufInCharCount++] = *(src++);
	}
}

Uint16 canF_readOutOfDummyFileBuf(char *dest, Uint16 reqCharCount) {
// Read characters out of CanFile's DummyFileBuf
//This routine is used by other applications that use CanFile
//to transfer a block of data between PC and Test Station.
// reqCharCount is maximum # of characters to write into dest.
// returns actual character count of characters transferred to dest.
	char *dfbPtr;
	Uint16 countRemaining;
	Uint16 countToCopy;
	Uint16 i;

	// if they are asking for characters beyond what we have in the buffer, return an error
	// unless it's just that we have an empty buffer to start with
	if ((dummyFileBufInCharCount > 0) && (dummyFileBufOutCharCount >= dummyFileBufInCharCount)) {
    	return (Uint16)(-1); // they are asking for characters beyond what is in our buffer
	}
	if (reqCharCount < 1){
		return (Uint16)(-1); // they are asking for 0 characters
	}

	// <ptr to 1st char to send> = <ptr to start of buffer> + <# of chars already sent>
	countRemaining = dummyFileBufInCharCount - dummyFileBufOutCharCount; // # chars left to send
	// figure countToCopy as MAX(caller's reqCharCount, or # chars left to send)
	if (countRemaining > reqCharCount){
		countToCopy = reqCharCount;
	} else {
		countToCopy = countRemaining;
	}

	// now copy the characters and bump up the dummyFileBufOutCharCount
	dfbPtr = dummyFileBuf + dummyFileBufOutCharCount;
	for (i=0;i<countToCopy;i++){
		*(dest++) = *(dfbPtr++);
	}
	dummyFileBufOutCharCount += countToCopy;
	return countToCopy;
}

enum CANOPEN_STATUS canF_recvDummyFile(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is char buff in MULTI_PACKET_BUF
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// This is an "Example Application" using multi-packet Receives
	// Consider it a template for other applications such as loading code files for FPGA or DSP
	// We get here each time the CanOpen module receives a complete multi-packet data set
	// for our index.subindex
	// Append received characters (bytes) into dummyFileBuf[] starting at
	// offset dummyFileBufInCharCount, and incrementing dummyFileBufInCharCount as
	// we go, taking care not to overrun the end of dummyFileBuf[]

	struct MULTI_PACKET_BUF *mpb;
	Uint16 countCharIn;
	char *src;
	Uint16 i;

	// for right now, lets display the char string on our diagnostic output
	mpb = (struct MULTI_PACKET_BUF *)(data-2);
	diagRs232recvMultiPacket(mpb);

	//append the received characters into dummyFileBuf, don't overrun it
    countCharIn = mpb->count_of_bytes_in_buf;
    src = mpb->buff;
	for (i=0;i<countCharIn;i++){
		if (dummyFileBufInCharCount >= DUMMY_FILE_BUF_SIZE) break; // exit "for" loop
		dummyFileBuf[dummyFileBufInCharCount++] = *(src++);
	}

	return CANOPEN_NO_ERR;
}
enum CANOPEN_STATUS canF_sendDummyFile(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is char buff in MULTI_PACKET_BUF
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	// This is an "Example Application" using multi-packet Receives
	// Consider it a template for other applications such as loading code files for FPGA or DSP
	// We get here each time the CanOpen module receives a request to start a multi-packet
	// data set SEND back to the host for our index.subindex
	//
	// Start at offset "dummyFileBufOutCharCount" into the dummyFileBuf
	// Copy "dummyFileBufOutPacketSize" characters to the MULTI_PACKET_BUF
	// increment "dummyFileBufOutCharCount" by "dummyFileBufOutPacketSize"

	char *dfbPtr;
	Uint16 packetCount;
	Uint16 countRemaining;
	Uint16 countCopied;

// - - - - Called at the start of each multi-packet SEND operation - - - -
//
	diagRs232sendDummyFile(); // diagnstic to RS232

	// Just in case FileBufOutPacketSize is <= 0, eg. improper value, we fix it
	// Note: "dummyFileBufOutPacketSize" is a value supplied by the PC specifying
	//    the maximum # of data bytes it wants to receive in each multi-packet send.
	if (dummyFileBufOutPacketSize <= 0) {
		dummyFileBufOutPacketSize = 7;
	}

	// if they are asking for characters beyond what we have in the buffer, return an error
	// unless it's just that we have an empty buffer to start with
	if ((dummyFileBufInCharCount > 0) && (dummyFileBufOutCharCount >= dummyFileBufInCharCount)) {
    	return CANOPEN_DFX_SEND_002_ERR; // they are asking for characters beyond what is in our buffer
	}

	// <ptr to 1st char to send> = <ptr to start of buffer> + <# of chars already sent>
	dfbPtr = dummyFileBuf + dummyFileBufOutCharCount;
	countRemaining = dummyFileBufInCharCount - dummyFileBufOutCharCount; // # chars left to send
	// figure packet size as MAX(user's requested packet size, or # chars left to send)
	if (countRemaining > dummyFileBufOutPacketSize){
		packetCount = dummyFileBufOutPacketSize;
	} else {
		packetCount = countRemaining;
	}
	countCopied = copyDataToMultiPacketBuf(dfbPtr, packetCount);

    if ((countCopied == 0) && (packetCount != 0)) {
    	return CANOPEN_DFX_SEND_001_ERR; // we can't copy to MultiPacketBuf, maybe it is is in use
    }

    dummyFileBufOutCharCount = dummyFileBufOutCharCount + countCopied;
	return CANOPEN_NO_ERR;
}

//===========================================================================
// Misc Diagnostic routines
//
//===========================================================================

void diagRs232sendDummyFile(){
// Log to RS232/USB for diagnostic purposes only
// Simple text display
	char msgOut[64];
    char *ptr;

    ptr = strU_strcpy(msgOut,"diagRs232sendDummyFile( )");
    ptr = strU_strcpy(ptr,"\n\r");
    r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
}
void diagRs232recvMultiPacket(struct MULTI_PACKET_BUF* mpb){
// Log to RS232/USB for diagnostic purposes only
// Display a character string received in a multi-packet upload
	char msgOut[64];
	char msgOut_b[64];
    char *ptr;
    char *ptr_b;
    char *receivedMsg;
    Uint16 i;
    Uint16 bytesToDisplay;

    receivedMsg = (char *)&(mpb->buff);
    bytesToDisplay = mpb->count_of_bytes_in_buf;

    ptr = strU_strcpy(msgOut,"Recv Multi-Packet Msg, Count: 0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,bytesToDisplay);
    ptr = strU_strcpy(ptr,"\n\r");
	if ((canF_diagOnOff & DIAG_ON_OFF_RECV_MULTI_PACK_MSG) != 0){ //diagnostic can be turned on or off
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
	}

    while (bytesToDisplay > 0) {
        // display characters 16 at a time
    	ptr = msgOut;
    	ptr_b = msgOut_b;
        for (i=0;i<16;i++){
        	if (bytesToDisplay > 0){
        		// display Chars as hex bytes
        		ptr_b = hexUtil_binTo2HexAsciiChars(ptr_b,(Uint16)*(receivedMsg));
        		ptr_b = strU_strcpy(ptr_b," ");
                // display Chars as Ascii
        		*(ptr++) = *(receivedMsg++);
        		bytesToDisplay--;
        	}
        }
        ptr = strU_strcpy(ptr,"\n\r");
        ptr_b = strU_strcpy(ptr_b,"\n\r");
		if ((canF_diagOnOff & DIAG_ON_OFF_MULTI_PACKET_ASCII) != 0){ //diagnostic can be turned on or off
			r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
		}
		if ((canF_diagOnOff & DIAG_ON_OFF_MULTI_PACKET_HEX) != 0){  //diagnostic can be turned on or off
			r232Out_outChars(msgOut_b, (Uint16)(ptr_b - msgOut_b));
		}
    }
}

void diagRs232CanRecvMsg(Uint16 mbxNumber, Uint16 *msg){
// Log to RS232/USB for diagnostic purposes only
// Given mailbox # 0 to 31, ptr to 4-word message received
	char msgOut[64];
    char *ptr;

    ptr = strU_strcpy(msgOut,"Recv CAN Mbox # 0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,mbxNumber);
    ptr = strU_strcpy(ptr," Msg: 0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,*(msg++));
    ptr = strU_strcpy(ptr," 0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,*(msg++));
    ptr = strU_strcpy(ptr," 0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,*(msg++));
    ptr = strU_strcpy(ptr," 0x");
    ptr = hexUtil_binTo4HexAsciiChars(ptr,*(msg++));
    ptr = strU_strcpy(ptr,"\n\r");
	if ((canF_diagOnOff & DIAG_ON_OFF_RECV_CAN_MBOX) != 0){  //diagnostic can be turned on or off
		r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
	}
}
