// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     SCI2.H
//
//   RS232 / RS485 via DSP SCI#2 (Serial Channel) and DB9 connector on TB3IOMC
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef SCI2x_H
#define SCI2x_H

#include "stddef.h"             // defnes NULL
#include "stdbool.h"            // needed for bool data types

// SCI Communication Control Register
#define SCICCR_ONE_STOP_BIT 0x00
#define SCICCR_NO_PARITY 0x00
#define SCICCR_ODD_PARITY 0x00
#define SCICCR_EVEN_PARITY 0x40
#define SCICCR_PARITY_DISABLE 0x00
#define SCICCR_PARITY_ENABLE 0x20
#define SCICCR_LOOPBACK_ENABLE 0x10
#define SCICCR_8_BIT_CHAR 0x07
#define SCICCR_7_BIT_CHAR 0x06

//SCI Control Register 1
#define SCICTL1_RX_ERR_INT_ENABLE 0x40
#define SCICTL1_RX_ERR_INT_DISABLE 0x00
#define SCICTL1_SW_RESET 0x00
#define SCICTL1_SW_NOT_RESET 0x20
#define SCICTL1_TXWAKE 0x40
#define SCICTL1_NO_TXWAKE 0x00
#define SCICTL1_SLEEP_ENABLED 0x04
#define SCICTL1_SLEEP_DISABLED 0x00
#define SCICTL1_TX_ENABLE 0x02
#define SCICTL1_TX_DISABLE 0x00
#define SCICTL1_RX_ENABLE 0x01
#define SCICTL1_RX_DISABLE 0x00

// SCI Baud-Select Registers
#define	 SCIBAUD_9600 487
//    BAUD  BRR
//    9600: 0x1E7 = 487
//   19200: 0x0F3 = 243
//  115200: 0x028 =  40

// SCI Control Register 2
#define SCICTL2_RX_INT_ENABLE 0x02
#define SCICTL2_RX_INT_DISABLE 0x00
#define SCICTL2_TX_INT_ENABLE 0x01
#define SCICTL2_TX_INT_DISABLE 0x00
#define SCICTL2_TX_RDY 0x80
#define SCICTL2_TX_EMPTY 0x40

//SCI Receiver Status Register
#define SCIRXST_RX_ERROR 0x80
#define SCIRXST_RX_RDY 0x40
#define SCIRXST_BRK_DT 0x20
#define SCIRXST_FE_ERR 0x10
#define SCIRXST_OE_ERR 0x08
#define SCIRXST_PE_ERR 0x04
#define SCIRXST_RX_WAKE 0x02

//SCI Priority Control Register
#define SCIPRI_SCITX_LOW_PRI 0x40
#define SCIPRI_SCIRX_LOW_PRI 0x20
#define SCIPRI_EMULATOR_SUSPEND 0x10
#define SCIPRI_EMULATOR_NO_SUSPEND 0x08

enum CANOPEN_STATUS sci2_init(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS sci2_xmit_test(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS sci2_recvTxBuf(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS sci2_sendRxBuf(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS sci2_recvTxBufAppend(const struct CAN_COMMAND* can_command, Uint16* data);

void sci2_rx_tx(void);


struct SCI2_PARAMS
    {
    Uint16 sciccr; //	number of bits, parity, and stop-bits value for SCICCR register
	Uint16 scibaud; // baud rate value for SCIBAUD register
    Uint16 idle_line; // 1= use Idle-Line protocol regarding start of new messages
    Uint16 tx_status; // 1= transmitting, 0 = not transmitting, 0=>1 start transmitting
	Uint16 tx_previous_status; // value of tx_status previous time through rs232_rx_tx( )
    Uint16 tx_index_txing_from_buf;
    Uint16 rx_err_status; // most recent error indication from SCIRXST, automatically 0'd at start of transmission
    Uint16 rs232_not_rs485; // 1-bit flag directs us to configure hardware
    };

struct SCI2_BUF
	{
		Uint16 max_char_in_buf; // initialize this to 512 max allowable # of chars in buff
	    Uint16 count_of_bytes_in_buf;
		char buff[(768+7)];	// spec at 768 max, + 7 extra words so overfilling won't crash anything else
	};


extern struct SCI2_PARAMS sci2;
extern struct SCI2_BUF sci2_Tx_Buf;
extern struct SCI2_BUF sci2_Rx_Buf;


#endif
