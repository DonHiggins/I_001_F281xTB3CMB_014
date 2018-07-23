// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     F2Int.H
//         support for FPGA2 to interrupt DSP
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef F2INTx_H
#define F2INTx_H

void f2i_enable_interrupt (void);
void f2i_initialize_interrupt (void);
interrupt void f2i_isr(void);
void f2i_store_int_vectors_in_PIE(void);
void f2i_BgTask_SSEnc(void);
Uint16 f2i_CRC5(Uint16 data_1, Uint16 data_2,Uint16 num_bits);
void f2i_SSEnc_init(void);
void f2i_SSEnc_reverse_position(void);

enum CANOPEN_STATUS f2i_recv_Pos_32_from_Host(const struct CAN_COMMAND* can_command, Uint16* data);

extern Uint16 f2i_SSEnc_Pos_1;
extern Uint16 f2i_SSEnc_Pos_2;
extern Uint16 f2i_SSEnc_Num_Pos_Bits;
extern Uint16 f2i_SSEnc_Alarm_Bit;  // LS Bit is Alarm Bit,
extern Uint32 f2i_SSEnc_Pos_32; // Absolute position, right justified, not bit-reversed
extern Uint16 f2i_SSEnc_CRC_5;

enum F2I_STATE {
	F2I_ST_IDLE     = 0,
	F2I_ST_DUMMY1 	= 1,
	F2I_ST_DUMMY2   = 2
};

enum ENDAT_MODE {
	EM_ENCODER_TRANSMIT_PARAM    = 0x23,
	EM_SELECTION_OF_MEM_AREA     = 0x0E,
	EM_ENC_TRANSMIT_POS_VALUE    = 0x07

};

enum ENDAT_MRS {
	EMRS_BITS_OF_POSITION   	  = 0x0D,
	EMRS_MEM_ALLOC_OEM_PARAMS     = 0xA1

};


#endif
