// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     Rs232Out.C
//
//   Provide RS232 Output service to tasks
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "stdbool.h"            // needed for bool data types
#include "TaskMgr.h"
#include "RS232.H"
#include "StrUtil.H"
#include "HexUtil.H"

#define CIRC_BUF_LENGTH 256
char r232Out_circular_buf[CIRC_BUF_LENGTH];
int circ_buf_next_in = 0;
int circ_buf_next_out = 0;
#define R232OUT_TX_BUF_LENGTH 32
char r232Out_Tx_buf[R232OUT_TX_BUF_LENGTH];
#define R232OUT_UTIL_BUF_LENGTH 32
char r232Out_util_buf[R232OUT_UTIL_BUF_LENGTH];

//
// Observations about our circular buffer . . .
// if circ_buf_next_in = circ_buf_next_out then buffer is EMPTY
// if circ_buf_next_in = circ_buf_next_out - 1 then buffer is FULL
// We never actually use the last space in our buffer,
// so maximum capacity is CIRC_BUF_LENGTH - 1
// we use null characters to separate strings in the circular buf
// and we will add a null, if the caller hasn't provided one.
//
bool r232Out_outChars(char* outChars, int charLength){
	// called: success = r232Out_outChars( )
    // called to store a character string into the circular buffer
    // for subsequent transmission out of the RS232 port
	// returns True if successful, false if it doesn't
	// all fit in available buffer
	int space_avail_in_circ_buf;
	int i;
	int add_space_for_null = 0;

	space_avail_in_circ_buf = circ_buf_next_out - circ_buf_next_in - 1;
	if (space_avail_in_circ_buf < 0){
		space_avail_in_circ_buf = space_avail_in_circ_buf + CIRC_BUF_LENGTH;
	}

	// is outChars already null terminated or do we
	// have to add a null?
	if (outChars[charLength-1] != 0) {
		add_space_for_null = 1;
	}

	// Is there enough space in the buffer for the whole message?
   if (space_avail_in_circ_buf < charLength + add_space_for_null) {
	  return false; // not successful
   }

   // OK, lets copy message (and null) into buffer
   for (i=0; i< charLength; i++) {
	   r232Out_circular_buf[circ_buf_next_in++]= outChars[i];
	   // don't go past the end of the circular buffer
	   // theoretically, earlier calculations on length should
	   // keep us from ever over running the circ_buf_next_out
	   if (circ_buf_next_in == CIRC_BUF_LENGTH) {
		   circ_buf_next_in = 0;
	   }
   }
   if (add_space_for_null != 0) {
	   r232Out_circular_buf[circ_buf_next_in++]= 0;
	   // don't go past the end of the circular buffer
	   if (circ_buf_next_in == CIRC_BUF_LENGTH) {
		   circ_buf_next_in = 0;
	   }
   }
   // invoke r232Out_circBufOutput( ) via the task manager to get data transmitted
   taskMgr_setTaskRoundRobin(TASKNUM_r232Out_circBufOutput,0);
   return true;
}

bool r232Out_outCharsNT(char* outChars){
	// This alternative method for calling r232Out_outChars( ), above,
	// works when *outChars points to a NULL TERMINATED string.
	// Here you don't need to supply the character length, we compute it,
	// then execute r232Out_outChars( ) and return the result to you.
	// Again, our purpose is to leave characters in a circular out-buff
	// to be transmitted by RS232 later.
	int i = 0;
	while (outChars[i++] != 0){};
	return r232Out_outChars(outChars,i);
}

void r232Out_circBufOutput(void){
	// Background task.  Activated whenever RS232 module completes
	// a transmit operation, or whenever someone stores some new
	// output data into the r232Out circular buffer.
	// If there is data in the circular buffer, then we hand some
	// of it over to the rs232 module to transmit.
	int chars_in_circ_buf;
	int chars_to_tx_buf;
	int chars_to_end_of_circ_buf;

	chars_in_circ_buf = circ_buf_next_in - circ_buf_next_out;
	if (chars_in_circ_buf < 0){
		chars_in_circ_buf = chars_in_circ_buf + CIRC_BUF_LENGTH;
	}

	if (chars_in_circ_buf == 0){
		return; // nothing to transmit
	}

    // if the rs232 transmission is busy, then exit
	if (rs232_transmit_status_busy()) {
		return;
	}


    // We take characters out of our circular buffer and place them
	// in a buffer that will be accessed by the RS232 Transmit routine.
	// # of chars will be the minimum of either
	//     chars_in_circ_buf or else
	//     size of our Tx buffer, or
	//     # chars to the physical end of our circular buffer (before we wrap.)
	chars_to_end_of_circ_buf =  CIRC_BUF_LENGTH - circ_buf_next_out;
	chars_to_tx_buf = R232OUT_TX_BUF_LENGTH;
	if (chars_to_tx_buf > chars_to_end_of_circ_buf){
		chars_to_tx_buf = chars_to_end_of_circ_buf;
	}
	if (chars_to_tx_buf > chars_in_circ_buf){
		chars_to_tx_buf = chars_in_circ_buf;
	}
  	memcpy(r232Out_Tx_buf,r232Out_circular_buf+circ_buf_next_out,chars_to_tx_buf);

  	// adjust circular buffer pointers, accounting for characters we "transmitted"
  	circ_buf_next_out = circ_buf_next_out + chars_to_tx_buf;
  	if (circ_buf_next_out >= CIRC_BUF_LENGTH){
  		circ_buf_next_out = 0;
  	}

  	// now call RS232 module to start the transmission
  	rs232_transmit_characters(r232Out_Tx_buf,chars_to_tx_buf);
}

bool r232Out_transmit_status_busy(void){
	// anyone can call this to see if we are in the process of transmitting

	if (circ_buf_next_in != circ_buf_next_out) {
		return true; // Circular buffer is not empty
	}

	if (rs232_transmit_status_busy()) {
		return true; // RS232 transmitter is still working on his local xmit buffer
	}
	// above may not take into account characters remaining in the xmit FIFO
	// but we can add something if that is an issue.
	return false; // Not busy
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Tx a line to RS232 acknowledging receipt of valid command
// for ex: "Ack Cmd  3005:2002"
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void r232Out_Command_Ack(Uint16 comintCode,Uint16 Data, bool dataPresent,char commandChar){
	// bool success;
	char *ptr;

	ptr = strU_strcpy(r232Out_util_buf,"\n\rAck Cmd  ");
	*(ptr++) = commandChar; // first character of command string
	ptr = hexUtil_binTo4HexAsciiChars(ptr, comintCode);
	if (dataPresent) {
		*(ptr++) = ':';
		ptr = hexUtil_binTo4HexAsciiChars(ptr,Data);
	}
	ptr = strU_strcpy(ptr,"\n\r");

	/* success = */ r232Out_outChars(r232Out_util_buf, (Uint16)(ptr - r232Out_util_buf));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Echo receive buffer contents to RS232 TX, preceded by CR LF SP SP
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void r232Out_Command_Nak(char* rxdata,Uint16 rxdataLength)
{
	// bool success;
	char* ptr;

	ptr = strU_strcpy(r232Out_util_buf,"\n\rNot a Cmd: ");
  	memcpy(ptr,(char*)(rxdata),rxdataLength);
  	ptr += rxdataLength;
	ptr = strU_strcpy(ptr,"\n\r");

	/* success = */ r232Out_outChars(r232Out_util_buf, (Uint16)(ptr - r232Out_util_buf));
}
