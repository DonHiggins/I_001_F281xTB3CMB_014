// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     AnlgIn.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef ANLGINx_H
#define ANLGINx_H

#include "stddef.h"             // defnes NULL
#include "stdbool.h"            // needed for bool data types

void ain_offsetCalcTask(void);
void ain_offsetCalcInit(void);
void ain_ad7175_setup_task_init();
void ain_ad7175_setup_task();
void ain_setSwitchDefaultsForB1(void);


extern Uint16 ain_offsets[8];
extern union CANOPEN16_32 ain_ad7175_single_write_data;

enum CANOPEN_STATUS ain_startOffsetCalc(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS ain_calibratedValue(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS ain_calibratedClassicValue(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS ain_calibratedClassicValRaw(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS ain_ad7175_request(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS ain_ad7175_fetch_status(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS ain_ad7175_fetch_data_read(const struct CAN_COMMAND* can_command, Uint16* data);

enum ANLGIN_CAL_STATE {
	ANLGIN_CS_DISCONNECT_IO        =  0,
	ANLGIN_CS_SET_ANLG_OUT         =  1,
	ANLGIN_CS_SET_LOOPBACK         =  2,
	ANLGIN_CS_READ_AI_VALUES       =  3,
	ANLGIN_CS_CALC_OFFSETS         =  4,
	ANLGIN_CS_RESTORE_SWITCHES     =  5,
	ANLGIN_CS_CALIBRATION_DONE     =  6
};

enum ANLGIN_CAL_STATE ain_getAnlginCalState(void);

enum ANLGIN_AD7175_SETUP {
	ANLGIN_AD7175_SETUP_START						= 0,
	ANLGIN_AD7175_SETUP_WAIT_001					= 1,
	ANLGIN_AD7175_SETUP_CONFIG_REG					= 2,
	ANLGIN_AD7175_SETUP_FPGA_FOR_CONTINUOUS_READ	= 3,
	ANLGIN_AD7175_SETUP_DONE						= 4
};

enum ANLGIN_AD7175_SETUP ain_get_ad7175SetupTaskState(void);

enum ANLGIN_AD7175_ERR {
	ANLGIN_AD7175_NO_ERROR       			 =  0,
	ANLGIN_AD7175_ERR_NO_ZERO_STATUS         =  1
};

struct ANLGIN_AD7175_REGISTER_CONFIG
   {
   Uint16  registerAddr;
   Uint16  configValue;
   };

union ANLGIN_SIGNED_16_32 {
	long 		all; //I'm guessing long is a signed 32
	struct TWO_UINT16 {
		Uint16 lsw;
		Uint16 msw;
	} words;
};


#endif
