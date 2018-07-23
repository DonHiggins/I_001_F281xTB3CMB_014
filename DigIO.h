// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     DigIO.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef DIGIOx_H
#define DIGIOx_H

extern Uint16 digio_DacComparatorValuesClassic[4];
extern Uint16 digio_DacAnlgOutValuesClassic[8];
extern Uint16 digio_nativeDacVal[16];

extern Uint32 digio_PwmOutputFreq32;
extern Uint32 digio_PwmOutputDutyCycl32;
extern Uint16 digio_PwmOutputFreqClassic16;
extern Uint16 digio_PwmOutputDutyCyclClassic16;

extern Uint32 digio_Enc1OutFreq32;
extern Uint32 digio_Enc2OutFreq32;
extern Uint32 digio_Enc1StopAfterN;
extern Uint32 digio_Enc2StopAfterN;
extern Uint32 digio_Enc1OutIndex32;
extern Uint32 digio_Enc2OutIndex32;
extern Uint16 digio_Enc1OutDir;
extern Uint16 digio_Enc2OutDir;
extern Uint16 digio_Enc1ManualStop;
extern Uint16 digio_Enc2ManualStop;
extern Uint16 digio_Enc1ClassicFreq;
extern Uint16 digio_Enc1ClassicIndex;
extern Uint16 digio_Enc1ClassicDir;
extern Uint16 digio_Enc2ClassicFreq;
extern Uint16 digio_Enc2ClassicIndex;
extern Uint16 digio_Enc2ClassicDir;

extern Uint32 digio_HallOutputFreq32;
extern Uint16 digio_HallOutputFreqClassic16;
extern Uint16 digio_HallOutputPhaseClassic16;  // Bit 0 - 0=60 degree, 1=120 degree phasing
extern Uint16 digio_HallOutputDirClassic16;    // Bit 0 - 0=CW, 1=CCW

enum CANOPEN_STATUS digio_recvComparitorClassic(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio_send16DigitalInputs(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio_recvAnlgOutClassic(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio_recvPwmOutputFreq32(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio_recvPwmOutputDutyCyc32(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio_recvPwmOutputFreqClassic16(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio_recvPwmOutputDutyCycClassic16(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio_recvEncOutParam(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio_recvClassicEncOutParam(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio_receiveNativeDacVal(const struct CAN_COMMAND *can_command,Uint16 *data) ;
enum CANOPEN_STATUS digio_recvHallOutputParam32(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS digio_recvHallOutputParamClassic16(const struct CAN_COMMAND *can_command,Uint16 *data);

void digio_initDacComparatorValuesClassic();
void digio_writeDacOutputValue(Uint16 dac_index, Uint16 dac_output_value);
void digio_PwmOutputFreq16Task(void);
void digio_EncInit(void);
void digio_HallOutInit(void);


#endif
