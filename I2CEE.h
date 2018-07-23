// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     I2CEE.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef I2CEEx_H
#define I2CEEx_H

#include "stddef.h"             // defnes NULL
#include "stdbool.h"            // needed for bool data types

#define SEND_START_CONDITION true
#define DONT_SEND_START_CONDITION false
#define SEND_STOP_CONDITION true
#define DONT_SEND_STOP_CONDITION false

#define I2CEE_WP_ON  true
#define I2CEE_WP_OFF false

extern Uint16 i2ceeSelectedEeprom;
extern Uint16 eeProm1ByteRWAddr;
extern Uint16 eeProm32ByteRWAddr;

bool i2cee_write(Uint16* dataBuf, Uint16 countBytesToWrite, Uint16 eepromAddr);
bool i2cee_write1Byte(Uint16 data, bool send_start, bool send_stop);
Uint16 i2cee_read1Byte(bool send_stop);
void i2cee_Init(void);
void i2cee_writeProtect(bool on_not_off );
void i2cee_disableClkToSelectedEEProm(void);
void i2cee_enableClkToSelectedEEProm(void);

void i2cee_test4200(Uint16 dataWord);
void i2cee_test4201(Uint16 dataWord);
void i2cee_diag4203Task(void);
void i2cee_test4203(Uint16 dataWord);
void i2cee_test4204(Uint16 dataWord);
void i2cee_test4205(Uint16 dataWord);
void i2cee_test4206(Uint16 dataWord);
void i2cee_selectEEProm(Uint16 selection);
void i2cee_readEEPromToCanFileTask(void);
void i2cee_progEEPromFromCanFileTask(void);
void i2cee_read32FromEEpromToBufTask(void);
void i2cee_burn32ToEEpromTask(void);

enum CANOPEN_STATUS i2cee_getTokenForEepromProg(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS i2cee_progEEPromFromCanFileData(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS i2cee_readEEPromStatus(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS i2cee_readEEPromToCanFile(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS i2cee_readEEProm1Byte(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS i2cee_writeEEProm1Byte(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS i2cee_32BytesFromEEpromToBuf(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS i2cee_32BytesToPC(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS i2cee_32BytesFromPC(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS i2cee_burn32BytesToEEProm(const struct CAN_COMMAND* can_command, Uint16* data);
enum CANOPEN_STATUS i2cee_read32ByteStatus(const struct CAN_COMMAND* can_command, Uint16* data);


enum I2CEE_SELECTEDEEPROM {
	I2CEE_SEL_TB3CM			  =	0x0001,
	I2CEE_SEL_TB3IOM		  =	0x0002,
	I2CEE_SEL_TB3PM			  =	0x0003

};

enum I2CEE_CANFILESTATUS {
	I2CEE_CFS_IDLE			 	=	0x0000,
	I2CEE_CFS_READING_EEPROM	=	0x0001,
	I2CEE_CFS_WRITING_EEPROM  	=	0x0002,
	I2CEE_CFS_READING_EEFAIL	=	0x0003,
	I2CEE_CFS_WRITING_EEFAIL  	=	0x0004,
	I2CEE_CFS_READING_EEDONE	=	0x0005,
	I2CEE_CFS_WRITING_EEDONE  	=	0x0006
};

enum I2CEE_32BYTESTATUS {
	I2CEE_32BYTE_IDLE		 	=	0x0000,
	I2CEE_32BYTE_READING_EEPROM	=	0x0001,
	I2CEE_32BYTE_WRITING_EEPROM =	0x0002,
	I2CEE_32BYTE_READING_EEFAIL	=	0x0003,
	I2CEE_32BYTE_WRITING_EEFAIL	=	0x0004,
	I2CEE_32BYTE_READING_EEDONE	=	0x0005,
	I2CEE_32BYTE_WRITING_EEDONE =	0x0006
};


#endif /* I2CEEx_H */
