// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     CanOpen.C
//
//   CAN Given that we just received an 8-byte CAN packet
//       interpret it according to CAN Open, and perform some sort of Test Station
//       action as directed by the packet.
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "Rs232Out.H"
//#include "stdbool.h"            // needed for bool data types
#include "HexUtil.H"
#include "StrUtil.H"
#include "CanOpen.h"
#include "CanComm.h"
#include "CanFile.h"
#include "FlashRW.H"
#include "CPLD.H"
#include "TimeStamp.h"
#include "F2Int.h"
#include "AnlgIn.H"
#include "I2CEE.H"
#include "Log.H"
#include "DigIO.H"
#include "DigIO2.H"
#include "Resolver.H"
#include "SSEnc.H"
#include "Sci2.H"
#include "LimitChk.H"
#include "Led.H"
#include "FpgaTest.H"

extern struct MULTI_PACKET_BUF multi_packet_buf;

Uint16 canTestData16[2];
Uint16 canTestData0x98765432[] = {0x5432,0x9876}; // {LS word, MS word}
Uint16 co_reset = 100;
Uint16 co_jump_to_bootloader = 100;


//===========================================================================
// Tables with parameters for each CanOpen request index.subindex.
//
//===========================================================================
//Tables to help parse incoming CAN requests.
//
// CAN_INDEX (array of structs) aka: can_index[]
//
//    When we receive a CAN message (SDO) from the PC, canC_pollRecv() calls
//    the canO_HandleCanOpenMessage() routine which uses the CANOpen INDEX value
//    from the message to index into the CAN_INDEX array to get 2 pieces of information:
//        1. the address of a CAN_COMMAND structure (below) which holds more info
//           that we will use to service the CAN message, and
//        2. value of the maximum legal SUBINDEX for this INDEX.  We use this to
//           validate the CANOpen SUBINDEX from the incoming CAN message.
//
//     Program Maintenance on the CAN_INDEX (array of structs):
//        1. We have a #define for MAX_CAN_INDEX, so if you need to add new elements
//           to the CAN_INDEX array, you should increase MAX_CAN_INDEX accordingly.
//        2. To add service for a message with a new CANOpen Index value, you have to
//           create a new CAN_COMMAND value (below) and add a new element to the
//           CAN_INDEX array.
//        3. If there is a CANOpen INDEX already serviced by this program and you want
//           to add service for a new CANOpen SUBINDEX value for that INDEX, you have
//           to add an element to the CAN_COMMAND array for that CANOpen INDEX, and
//           you have to adjust the CAN_INDEX element so it gives the new value for the
//           maximum legal SUBINDEX for this INDEX.
//
//
// CAN_COMMAND -- for each CAN index and each sub-index we have several pieces of
//    information useful in responding to the command, such as function pointers to
//    call, depending on whether it is a read or a write operation, an indicator of
//    what type/size of data is expected in the response, and a pointer to a data field
//    that may be important to the response.
//
// CAN_COMMAND (for each CANOpen INDEX an array of structs) aka: index_2000[], index_2001[], etc.
//    When we receive a CAN message (SDO) the canO_HandleCanOpenMessage() routine uses the
//    CANOpen INDEX value from the message and the CAN_INDEX table (above) to get a pointer to
//    the approporiate CAN_COMMAND array for that message.  Then it uses the CANOpen SUBINDEX
//    from the message to index into the CAN_COMMAND array, and locate parameters that may (or
//    may not) be used to service the CAN message.  The element in the CAN_COMMAND array (a struct)
//    contains function pointers.  The functions are called to perform any service required for
//    the CAN message.  The functions receive all the information in the CAN_COMMAND element, and
//    it is up to each function to use whatever data it needs and ignore the rest.
//
//    *datapointer --
//       In the simplest case, we have CAN messages that cause us to read or write a 16 or 32-bit
//       data value into memory and we read or write at this address.  In some cases the read
//       or write directly address the CPLD or FPGA's on the external bus -- to the DSP it looks
//       just like a RAM access, but the CPLD and FPGA's may be performing complicated functions
//       for us.
//       In some cases datapointer points to an element in an array, and we subtract it from the
//       base of the array to get the array index.
//       In some cases datapointer may not be used at all by the functions servicing the CAN message,
//       but in our haste we put in a dummy value like "canTestData16", rather than a more elegant
//       indicator for an unused value, so there may be opportunities to clean this up a little.
//    replyDataType --
//       As far A I know, we ignore this value unless it is TYP_OCT_STRING, in which case it
//       signifies a "non-expedited SDO" multi-packet data transfer.  This is pretty much
//       dictated by compatibility with the Classic Test Station and the PC software that
//       communicates with it.
//    *sendProcess --
//       The canO_HandleCanOpenMessage() calls this function for messages where the test station
//       "sends" data to satisfy a "read" request from the PC.
//       To facilitate being called from pointers in CAN_COMMAND, all send and recv processes
//       are defined with identical calling parameters and return type.  For example check out
//          enum CANOPEN_STATUS canO_recv16Bits(const struct CAN_COMMAND *can_command,Uint16 *data) {};
//    *recvProcess --
//       The canO_HandleCanOpenMessage() calls this function for messages where the test station
//       "receives" data from the PC which sent a "write" request.


const struct CAN_COMMAND index_2000[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};

// Digital Output Bank Functionality (Index 2001)
const struct CAN_COMMAND index_2001[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
	// (void  *)data    	                        Uint16     send_funct   recv_funct
	//-----------------------------------------    ----------- ----------  -----------------
	{(digio2_DigOutSignalAssignment+0),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutSignalAssignment},     //2001.01
	{(digio2_DigOutSignalAssignment+1),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutSignalAssignment},     //2001.02
	{(digio2_DigOutSignalAssignment+2),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutSignalAssignment},     //2001.03
	{(digio2_DigOutSignalAssignment+3),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutSignalAssignment},     //2001.04
	{(digio2_DigOutSignalAssignment+4),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutSignalAssignment},     //2001.05
	{(digio2_DigOutSignalAssignment+5),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutSignalAssignment}};    //2001.06

// Differential Output Functionality
const struct CAN_COMMAND index_2002[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
	{(digio2_DiffOutSignalAssignment+0),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDiffOutSignalAssignment},     //2002.01
	{(digio2_DiffOutSignalAssignment+1),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDiffOutSignalAssignment}};    //2002.02

// Digital Output Bank Configuration (Index 2003) Rail Voltages
const struct CAN_COMMAND index_2003[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},                                    //2003.00
	// (void  *)data    	                        Uint16     send_funct   recv_funct
	//-----------------------------------------    ----------- ----------  -----------------
	{(digio2_DigOutRails+0),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutRails},     //2003.01
	{(digio2_DigOutRails+1),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutRails},     //2003.02
	{(digio2_DigOutRails+2),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutRails},     //2003.03
	{(digio2_DigOutRails+3),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutRails},     //2003.04
	{(digio2_DigOutRails+4),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutRails},     //2003.05
	{(digio2_DigOutRails+5),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutRails}};    //2003.06

// Digital Output Bank Mode (Index 2004) eg Push-Pull vs Open Collector, etc
const struct CAN_COMMAND index_2004[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// (void  *)data    	                        Uint16     send_funct   recv_funct
//-----------------------------------------    ----------- ----------  -----------------
	{(digio2_DigOutMode+0),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutMode},     //2004.01
	{(digio2_DigOutMode+1),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutMode},     //2004.02
	{(digio2_DigOutMode+2),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutMode},     //2004.03
	{(digio2_DigOutMode+3),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutMode},     //2004.04
	{(digio2_DigOutMode+4),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutMode},     //2004.05
	{(digio2_DigOutMode+5),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutMode}};    //2004.06


// Digital Output Bank State (Index 2005)
const struct CAN_COMMAND index_2005[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// (void  *)data    	                        Uint16     send_funct   recv_funct
//-----------------------------------------    ----------- ----------  -----------------
	{(digio2_DigOutLevel+0),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutLevel},     //2005.01
	{(digio2_DigOutLevel+1),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutLevel},     //2005.02
	{(digio2_DigOutLevel+2),  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDigOutLevel}};    //2005.03

// Differential Outputs
const struct CAN_COMMAND index_2006[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
{&digio2_DiffOutEnable,  TYP_UINT32,   &canO_send16Bits,    &digio2_recvDiffOutEnable},     //2006.01
{&digio2_DiffOutLevel,   TYP_UINT32,   &canO_send16Bits,    &digio2_recvDiffOutLevel}};     //2006.02

// Hall Signal characteristics for Digital Outputs
const struct CAN_COMMAND index_2007[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
{&digio_HallOutputFreqClassic16,TYP_UINT32, &canO_send16Bits, &digio_recvHallOutputParamClassic16},     //2007.01
{&digio_HallOutputPhaseClassic16,TYP_UINT32, &canO_send16Bits, &digio_recvHallOutputParamClassic16},	//2007.02
{&digio_HallOutputDirClassic16,	TYP_UINT32, &canO_send16Bits, &digio_recvHallOutputParamClassic16},     //2007.03
{&digio_HallOutputFreq32,		TYP_UINT32, &canO_send32Bits, &digio_recvHallOutputParam32		}};		//2007.04

//Encoder Output # 1, 16-bit commands compatible w/ classic test station
// See 0x2055 for 32-bit native mode commands to control encoder simulator output.
const struct CAN_COMMAND index_2008[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
{&digio_Enc1ClassicFreq,	TYP_UINT32, &canO_send16Bits, &digio_recvClassicEncOutParam },     //2008.01
{&digio_Enc1ClassicIndex,	TYP_UINT32, &canO_send16Bits, &digio_recvClassicEncOutParam },     //2008.02
{&digio_Enc1ClassicDir,		TYP_UINT32, &canO_send16Bits, &digio_recvClassicEncOutParam }};    //2008.03
//Encoder Output # 2, 16-bit commands compatible w/ classic test station
const struct CAN_COMMAND index_2009[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
{&digio_Enc2ClassicFreq,	TYP_UINT32, &canO_send16Bits, &digio_recvClassicEncOutParam },     //2009.01
{&digio_Enc2ClassicIndex,	TYP_UINT32, &canO_send16Bits, &digio_recvClassicEncOutParam },     //2009.02
{&digio_Enc2ClassicDir,		TYP_UINT32, &canO_send16Bits, &digio_recvClassicEncOutParam }};    //2009.03

// PWM Frequency Output, compatible w/ classic test station (see index 0x2054 for 32-bit version)
const struct CAN_COMMAND index_200A[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
	{&digio_PwmOutputFreqClassic16,		TYP_UINT32, &canO_send32Bits, &digio_recvPwmOutputFreqClassic16},     //200A.01
	{&digio_PwmOutputDutyCyclClassic16,	TYP_UINT32, &canO_send32Bits, &digio_recvPwmOutputDutyCycClassic16}}; //200A.02
	//	PWM / ENC DUTY CYCLE CONFUSION
	//
	//	Reading PWM Duty Cycle or "On Time", the Classic test station reports the duration of the OFF portion of the PWM wave.
	// 	For compatibility, the TS3 test station also reports the duration of the OFF portion (rather than the ON portion) of
	//	the measured PWM wave.
	//  This is non-intuitive and can be a source of confusion, though all of our legacy PC client software works with it this way.
	//   Here's what you need to keep in mind:  In commands that set the "Duty Cycle" for digital machine output, you must
	//	specify the duration of the ON time, whereas when you read the measured "Duty Cycle" from the digital machine input,
	//	it gives you the duration of the OFF time of the PWM waveform.

//  Outputs -- Scaled to be compatible with Classic NTSTSYS
const struct CAN_COMMAND index_200B[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
	{(digio_DacAnlgOutValuesClassic+0),	TYP_UINT32,   &canO_send16Bits,    &digio_recvAnlgOutClassic },     //200B.01
	{(digio_DacAnlgOutValuesClassic+1),	TYP_UINT32,   &canO_send16Bits,    &digio_recvAnlgOutClassic },     //200B.02
	{(digio_DacAnlgOutValuesClassic+2),	TYP_UINT32,   &canO_send16Bits,    &digio_recvAnlgOutClassic },     //200B.03
	{(digio_DacAnlgOutValuesClassic+3),	TYP_UINT32,   &canO_send16Bits,    &digio_recvAnlgOutClassic },     //200B.04
	{(digio_DacAnlgOutValuesClassic+4),	TYP_UINT32,   &canO_send16Bits,    &digio_recvAnlgOutClassic },     //200B.05
	{(digio_DacAnlgOutValuesClassic+5),	TYP_UINT32,   &canO_send16Bits,    &digio_recvAnlgOutClassic },     //200B.06
	{(digio_DacAnlgOutValuesClassic+6),	TYP_UINT32,   &canO_send16Bits,    &digio_recvAnlgOutClassic },     //200B.07
	{(digio_DacAnlgOutValuesClassic+7),	TYP_UINT32,   &canO_send16Bits,    &digio_recvAnlgOutClassic }};    //200B.08

// Resolver Simulator Settings 200C compatible with Classic NTSTSYS
//const struct CAN_COMMAND index_200C[] ={ {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
const struct CAN_COMMAND index_200C[] ={ {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
	{&res_RefDacValue            ,TYP_UINT16,  &canO_send16Bits  ,&res_recvRefDacValue},		//200C.01
	{&res_RefFreq                ,TYP_UINT16,  &canO_send16Bits  ,&res_recvRefFreq   },		    //200C.02
	{&res_AmpRefInOut            ,TYP_UINT16,  &canO_send16Bits  ,&res_recvAmpRef       },		//200C.03
	{&res_FixedShaftAngle        ,TYP_UINT16,  &canO_send16Bits  ,&res_recvShaftAngle },		//200C.04
	{&res_HiPrecisShaftAngle     ,TYP_UINT16,  &canO_send16Bits  ,&res_recvShaftAngleIincreasedPrecision }, //200C.05
	{&res_attenuation            ,TYP_UINT16,  &canO_send16Bits  ,&canO_recv16Bits },	      	//200C.06
	{&res_velocity               ,TYP_UINT16,  &canO_send16Bits  ,&canO_recv16Bits }};		    //200C.07

const struct CAN_COMMAND index_200D[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};

// Set Digital In Comparator Voltage Thresholds -- compatible with Classic Test Station, NTSTSYS
const struct CAN_COMMAND index_200E[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
{(digio_DacComparatorValuesClassic+0),	TYP_UINT32,   &canO_send16Bits,    &digio_recvComparitorClassic },     //200E.01
{(digio_DacComparatorValuesClassic+1),	TYP_UINT32,   &canO_send16Bits,    &digio_recvComparitorClassic },     //200E.02
{(digio_DacComparatorValuesClassic+2),	TYP_UINT32,   &canO_send16Bits,    &digio_recvComparitorClassic },     //200E.03
{(digio_DacComparatorValuesClassic+3),	TYP_UINT32,   &canO_send16Bits,    &digio_recvComparitorClassic }};    //200E.04


// Filter Clock Freq for MAX297 Filters in ANLG_IN 5,6,7,8
const struct CAN_COMMAND index_200F[] =
	{CPLD_F2_XA(FPGA2_WRITE_ANLG_IN_FLTR_CLK),	TYP_UINT32,   NULL,    &canO_recv16Bits };  //200F.00

// Analog_In Scaled as Classic Test Station.  AKA compatible w/ legacy PC software
// See 0x204E for new ADC commands
const struct CAN_COMMAND index_2010[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
		{(ain_offsets+0),	TYP_UINT32,   &ain_calibratedClassicValue,    NULL },     //2010.01
		{(ain_offsets+1),	TYP_UINT32,   &ain_calibratedClassicValue,    NULL },     //2010.02
		{(ain_offsets+2),	TYP_UINT32,   &ain_calibratedClassicValue,    NULL },     //2010.03
		{(ain_offsets+3),	TYP_UINT32,   &ain_calibratedClassicValue,    NULL },     //2010.04
		{(ain_offsets+4),	TYP_UINT32,   &ain_calibratedClassicValue,    NULL },     //2010.05
		{(ain_offsets+5),	TYP_UINT32,   &ain_calibratedClassicValue,    NULL },     //2010.06
		{(ain_offsets+6),	TYP_UINT32,   &ain_calibratedClassicValue,    NULL },     //2010.07
		{(ain_offsets+7),	TYP_UINT32,   &ain_calibratedClassicValue,    NULL },     //2010.08
		{canTestData16,		TYP_UINT32,   &ain_calibratedClassicValRaw,   NULL }};    //2010.09

// Write to 0x2011.1,2,3 to configure which digital input pins are used as input
// to PWM, Encoder and Hall input measurement machines.
// Compatible w/ legacy PC software interface
const struct CAN_COMMAND index_2011[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
		{&digio2_EncInMap,	 TYP_UINT32, &canO_send16Bits, &digio2_recvEncInMap},		//2011.01
		{&digio2_HallInMap,	 TYP_UINT32, &canO_send16Bits, &digio2_recvHallInMap},		//2011.02
		{&digio2_PwmInMap,	 TYP_UINT32, &canO_send16Bits, &digio2_recvPwmInMap}};  	//2011.03

// Digital Inputs
const struct CAN_COMMAND index_2012[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// (void  *)data    	               Uint16     send_funct              recv_funct
//--------------------------------   ----------- --------------------  -----------------
 {CPLD_F2_XA(FPGA2_READ_DIG_IN),	TYP_UINT32,   &digio_send16DigitalInputs,	NULL},     //2012.01
 {CPLD_F2_XA(FPGA2_READ_DIFF_IN),	TYP_UINT32,   &canO_send16Bits,				NULL}};    //2012.02

// Read from 0x2013 to get measurements (freq, etc) from Encoder digital input machines.
// Compatible w/ legacy PC software interface
const struct CAN_COMMAND index_2013[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
{NULL,	TYP_UINT32,   &digio2_sendEncReadingClassic,	NULL},     //2013.01 read Encoder Period
{NULL,	TYP_UINT32,   &digio2_sendEncReadingClassic,	NULL},     //2013.02 read Encoder Index Period
{NULL,	TYP_UINT32,   &digio2_sendEncReadingClassic,	NULL},     //2013.03 read Encoder Direction
{NULL,	TYP_UINT32,   &digio2_sendEncReadingClassic,	NULL},     //2013.04 read Encoder Direction
{NULL,	TYP_UINT32,   &digio2_sendEncReadingClassic,	NULL},     //2013.05 read Encoder Counts
{&digio2_EncIndexDivisor,	TYP_UINT32,   &canO_send16Bits,	&digio2_recvEncIndexDivisor}};    //2013.06 Write Encoder Index Divisor

// Read from 0x2014 to get measurements (freq, etc) from Hall digital input machines.
// Compatible w/ legacy PC software interface
// Per R.C, "we don't need Hall digital input machines in TS3"
// Just need to make sure our responses don't cause problem with legacy systems.
const struct CAN_COMMAND index_2014[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
{NULL,	TYP_UINT32,   &digio2_sendHallReadingClassic,	NULL},     //2014.01 read Hall Period
{NULL,	TYP_UINT32,   &digio2_sendHallReadingClassic,	NULL},     //2014.02 read Hall Phase
{NULL,	TYP_UINT32,   &digio2_sendHallReadingClassic,	NULL},     //2014.03 read Hall Dir
{NULL,	TYP_UINT32,   &digio2_sendHallReadingClassic,	NULL}};    //2014.04 read Hall OnTime

// Read from 0x2015 to get measurements (freq, etc) from PWM digital input machines.
// Compatible w/ legacy PC software interface
const struct CAN_COMMAND index_2015[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// (void  *)data    	               Uint16     send_funct              recv_funct
//--------------------------------   ----------- --------------------  -----------------
{NULL,	TYP_UINT32,   &digio2_sendPwmReadingClassic,	NULL},     //2015.01 read PWM_1 Period
{NULL,	TYP_UINT32,   &digio2_sendPwmReadingClassic,	NULL},     //2015.02 read PWM_1 OnTime
{NULL,	TYP_UINT32,   &digio2_sendPwmReadingClassic,	NULL},     //2015.03 read PWM_2 Period
{NULL,	TYP_UINT32,   &digio2_sendPwmReadingClassic,	NULL}};    //2015.04 read PWM_2 OnTime
//	PWM / ENC DUTY CYCLE CONFUSION
//
//	Reading PWM Duty Cycle or "On Time", the Classic test station reports the duration of the OFF portion of the PWM wave.
// 	For compatibility, the TS3 test station also reports the duration of the OFF portion (rather than the ON portion) of
//	the measured PWM wave.
//  This is non-intuitive and can be a source of confusion, though all of our legacy PC client software works with it this way.
//   Here's what you need to keep in mind:  In commands that set the "Duty Cycle" for digital machine output, you must
//	specify the duration of the ON time, whereas when you read the measured "Duty Cycle" from the digital machine input,
//	it gives you the duration of the OFF time of the PWM waveform.



const struct CAN_COMMAND index_2016[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2017[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2018[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2019[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_201A[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_201B[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_201C[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_201D[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_201E[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_201F[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2020[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2021[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};

// Limit Check: synch command
const struct CAN_COMMAND index_2022[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,&limchkRecvSync}; //2022.00

// Start of Classic Compatible Limit Check 1 - 8
// Limit Check Channel 1
const struct CAN_COMMAND index_2023[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,limchkResetOneChannel},
// (void  *)data    	               Uint16     send_funct              recv_funct
//--------------------------------   ----------- --------------------  -----------------
{&limitCheckParams[0].enableTest,	TYP_UINT32,   &canO_send16Bits,	&limchkRecvEnable},     //202x.01 1=TEST_ENABLED, 0=TEST_DISABLED
{NULL,	TYP_UINT32,   &limchkSendWhichInput, &limchkRecvWhichInput},			//202x.02  Which_Input  & No_Halt
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},					//202x.03 Inner Limits
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},					//202x.04 Outer Limits
{NULL,	TYP_UINT32,   &limchkSendTimeStartEnd,	&limchkRecvTimeStartEnd},		//202x.05 Time_start, Time_End
{&limitCheckParams[0].allowedFails, TYP_UINT32,  &limchkSendFails, &canO_recv16Bits},	//202x.06
{NULL,	TYP_UINT32,   &limchkSendMinMaxValue,	NULL},			//202x.07
{NULL,	TYP_UINT32,   &limchkSendAvgValue,		NULL},			//202x.08
{NULL,									TYP_UINT32,   &limchkSendTestStatus,	NULL},	//202x.09 testStatus + (measValue * 0x10000)
{NULL,	TYP_UINT32,   &limchkSendMeasValueClassic,		NULL},							//202x.0A measValue Classic
// -- new native commands for limit check
{&limitCheckParams[0].comparisonResult,	TYP_UINT32,   &canO_send16Bits,	NULL},			//202x.0B comparisonResult
{&limitCheckParams[0].measValue			,TYP_UINT32,   &canO_send32Bits,	NULL},			//202x.0C Meas Value Native
{&limitCheckParams[0].limitCheckState,	TYP_UINT32,   &canO_send16Bits,	NULL}};			//202x.0D limitCheckState Native

// Limit Check Channel 2
const struct CAN_COMMAND index_2024[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,limchkResetOneChannel},
// (void  *)data    	               Uint16     send_funct              recv_funct
//--------------------------------   ----------- --------------------  -----------------
{&limitCheckParams[1].enableTest,	TYP_UINT32,   &canO_send16Bits,	&limchkRecvEnable}, //202x.01 1=TEST_ENABLED, 0=TEST_DISABLED
{NULL,	TYP_UINT32,   &limchkSendWhichInput, &limchkRecvWhichInput},					//202x.02 Which_Input  & No_Halt
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.03 Inner Limits
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.04 Outer Limits
{NULL,	TYP_UINT32,   &limchkSendTimeStartEnd,	&limchkRecvTimeStartEnd},				//202x.05 Time_start, Time_End
{&limitCheckParams[1].allowedFails,	TYP_UINT32,   &limchkSendFails,	&canO_recv16Bits},	//202x.06 allowedFails
{NULL,	TYP_UINT32,   &limchkSendMinMaxValue,	NULL},									//202x.07 Min / Max Values
{NULL,	TYP_UINT32,   &limchkSendAvgValue,		NULL},									//202x.08 Average Value
{NULL,									TYP_UINT32,   &limchkSendTestStatus,	NULL},	//202x.09 testStatus + (measValue * 0x10000)
{NULL,	TYP_UINT32,   &limchkSendMeasValueClassic,		NULL},							//202x.0A measValue Classic
// -- new native commands for limit check
{&limitCheckParams[1].comparisonResult,	TYP_UINT32,   &canO_send16Bits,	NULL},			//202x.0B comparisonResult Native
{&limitCheckParams[1].measValue			,TYP_UINT32,   &canO_send32Bits,	NULL},		//202x.0C Meas Value Native
{&limitCheckParams[1].limitCheckState,	TYP_UINT32,   &canO_send16Bits,	NULL}};			//202x.0D limitCheckState Native

// Limit Check Channel 3
const struct CAN_COMMAND index_2025[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,limchkResetOneChannel},
// (void  *)data    	               Uint16     send_funct              recv_funct
//--------------------------------   ----------- --------------------  -----------------
{&limitCheckParams[2].enableTest,	TYP_UINT32,   &canO_send16Bits,	&limchkRecvEnable}, //202x.01 1=TEST_ENABLED, 0=TEST_DISABLED
{NULL,	TYP_UINT32,   &limchkSendWhichInput, &limchkRecvWhichInput},					//202x.02 Which_Input  & No_Halt
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.03 Inner Limits
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.04 Outer Limits
{NULL,	TYP_UINT32,   &limchkSendTimeStartEnd,	&limchkRecvTimeStartEnd},				//202x.05 Time_start, Time_End
{&limitCheckParams[2].allowedFails,	TYP_UINT32,   &limchkSendFails,	&canO_recv16Bits},	//202x.06 allowedFails
{NULL,	TYP_UINT32,   &limchkSendMinMaxValue,	NULL},									//202x.07 Min / Max Values
{NULL,	TYP_UINT32,   &limchkSendAvgValue,		NULL},									//202x.08 Average Value
{NULL,									TYP_UINT32,   &limchkSendTestStatus,	NULL},	//202x.09 testStatus + (measValue * 0x10000)
{NULL,	TYP_UINT32,   &limchkSendMeasValueClassic,		NULL},							//202x.0A measValue Classic
// -- new native commands for limit check
{&limitCheckParams[2].comparisonResult,	TYP_UINT32,   &canO_send16Bits,	NULL},			//202x.0B comparisonResult Native
{&limitCheckParams[2].measValue			,TYP_UINT32,   &canO_send32Bits,	NULL},		//202x.0C Meas Value Native
{&limitCheckParams[2].limitCheckState,	TYP_UINT32,   &canO_send16Bits,	NULL}};			//202x.0D limitCheckState Native

// Limit Check Channel 4
const struct CAN_COMMAND index_2026[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,limchkResetOneChannel},
// (void  *)data    	               Uint16     send_funct              recv_funct
//--------------------------------   ----------- --------------------  -----------------
{&limitCheckParams[3].enableTest,	TYP_UINT32,   &canO_send16Bits,	&limchkRecvEnable}, //202x.01 1=TEST_ENABLED, 0=TEST_DISABLED
{NULL,	TYP_UINT32,   &limchkSendWhichInput, &limchkRecvWhichInput},					//202x.02 Which_Input  & No_Halt
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.03 Inner Limits
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.04 Outer Limits
{NULL,	TYP_UINT32,   &limchkSendTimeStartEnd,	&limchkRecvTimeStartEnd},				//202x.05 Time_start, Time_End
{&limitCheckParams[3].allowedFails,	TYP_UINT32,   &limchkSendFails,	&canO_recv16Bits},	//202x.06 allowedFails
{NULL,	TYP_UINT32,   &limchkSendMinMaxValue,	NULL},									//202x.07 Min / Max Values
{NULL,	TYP_UINT32,   &limchkSendAvgValue,		NULL},									//202x.08 Average Value
{NULL,									TYP_UINT32,   &limchkSendTestStatus,	NULL},	//202x.09 testStatus + (measValue * 0x10000)
{NULL,	TYP_UINT32,   &limchkSendMeasValueClassic,		NULL},							//202x.0A measValue Classic
// -- new native commands for limit check
{&limitCheckParams[3].comparisonResult,	TYP_UINT32,   &canO_send16Bits,	NULL},			//202x.0B comparisonResult Native
{&limitCheckParams[3].measValue			,TYP_UINT32,   &canO_send32Bits,	NULL},		//202x.0C Meas Value Native
{&limitCheckParams[3].limitCheckState,	TYP_UINT32,   &canO_send16Bits,	NULL}};			//202x.0D limitCheckState Native

const struct CAN_COMMAND index_2027[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,limchkResetOneChannel},
// (void  *)data    	               Uint16     send_funct              recv_funct
//--------------------------------   ----------- --------------------  -----------------
{&limitCheckParams[4].enableTest,	TYP_UINT32,   &canO_send16Bits,	&limchkRecvEnable}, //202x.01 1=TEST_ENABLED, 0=TEST_DISABLED
{NULL,	TYP_UINT32,   &limchkSendWhichInput, &limchkRecvWhichInput},					//202x.02 Which_Input  & No_Halt
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.03 Inner Limits
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.04 Outer Limits
{NULL,	TYP_UINT32,   &limchkSendTimeStartEnd,	&limchkRecvTimeStartEnd},				//202x.05 Time_start, Time_End
{&limitCheckParams[4].allowedFails,	TYP_UINT32,   &limchkSendFails,	&canO_recv16Bits},	//202x.06 allowedFails
{NULL,	TYP_UINT32,   &limchkSendMinMaxValue,	NULL},									//202x.07 Min / Max Values
{NULL,	TYP_UINT32,   &limchkSendAvgValue,		NULL},									//202x.08 Average Value
{NULL,									TYP_UINT32,   &limchkSendTestStatus,	NULL},	//202x.09 testStatus + (measValue * 0x10000)
{NULL,	TYP_UINT32,   &limchkSendMeasValueClassic,		NULL},							//202x.0A measValue Classic
// -- new native commands for limit check
{&limitCheckParams[4].comparisonResult,	TYP_UINT32,   &canO_send16Bits,	NULL},			//202x.0B comparisonResult Native
{&limitCheckParams[4].measValue			,TYP_UINT32,   &canO_send32Bits,	NULL},		//202x.0C Meas Value Native
{&limitCheckParams[4].limitCheckState,	TYP_UINT32,   &canO_send16Bits,	NULL}};			//202x.0D limitCheckState Native

// Limit Check Channel 6
const struct CAN_COMMAND index_2028[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,limchkResetOneChannel},
// (void  *)data    	               Uint16     send_funct              recv_funct
//--------------------------------   ----------- --------------------  -----------------
{&limitCheckParams[5].enableTest,	TYP_UINT32,   &canO_send16Bits,	&limchkRecvEnable}, //202x.01 1=TEST_ENABLED, 0=TEST_DISABLED
{NULL,	TYP_UINT32,   &limchkSendWhichInput, &limchkRecvWhichInput},					//202x.02 Which_Input  & No_Halt
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.03 Inner Limits
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.04 Outer Limits
{NULL,	TYP_UINT32,   &limchkSendTimeStartEnd,	&limchkRecvTimeStartEnd},				//202x.05 Time_start, Time_End
{&limitCheckParams[5].allowedFails,	TYP_UINT32,   &limchkSendFails,	&canO_recv16Bits},	//202x.06 allowedFails
{NULL,	TYP_UINT32,   &limchkSendMinMaxValue,	NULL},									//202x.07 Min / Max Values
{NULL,	TYP_UINT32,   &limchkSendAvgValue,		NULL},									//202x.08 Average Value
{NULL,									TYP_UINT32,   &limchkSendTestStatus,	NULL},	//202x.09 testStatus + (measValue * 0x10000)
{NULL,	TYP_UINT32,   &limchkSendMeasValueClassic,		NULL},							//202x.0A measValue Classic
// -- new native commands for limit check
{&limitCheckParams[5].comparisonResult,	TYP_UINT32,   &canO_send16Bits,	NULL},			//202x.0B comparisonResult Native
{&limitCheckParams[5].measValue			,TYP_UINT32,   &canO_send32Bits,	NULL},		//202x.0C Meas Value Native
{&limitCheckParams[5].limitCheckState,	TYP_UINT32,   &canO_send16Bits,	NULL}};			//202x.0D limitCheckState Native

// Limit Check Channel 7
const struct CAN_COMMAND index_2029[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,limchkResetOneChannel},
// (void  *)data    	               Uint16     send_funct              recv_funct
//--------------------------------   ----------- --------------------  -----------------
{&limitCheckParams[6].enableTest,	TYP_UINT32,   &canO_send16Bits,	&limchkRecvEnable}, //202x.01 1=TEST_ENABLED, 0=TEST_DISABLED
{NULL,	TYP_UINT32,   &limchkSendWhichInput, &limchkRecvWhichInput},					//202x.02 Which_Input  & No_Halt
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.03 Inner Limits
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.04 Outer Limits
{NULL,	TYP_UINT32,   &limchkSendTimeStartEnd,	&limchkRecvTimeStartEnd},				//202x.05 Time_start, Time_End
{&limitCheckParams[6].allowedFails,	TYP_UINT32,   &limchkSendFails,	&canO_recv16Bits},	//202x.06 allowedFails
{NULL,	TYP_UINT32,   &limchkSendMinMaxValue,	NULL},									//202x.07 Min / Max Values
{NULL,	TYP_UINT32,   &limchkSendAvgValue,		NULL},									//202x.08 Average Value
{NULL,									TYP_UINT32,   &limchkSendTestStatus,	NULL},	//202x.09 testStatus + (measValue * 0x10000)
{NULL,	TYP_UINT32,   &limchkSendMeasValueClassic,		NULL},							//202x.0A measValue Classic
// -- new native commands for limit check
{&limitCheckParams[6].comparisonResult,	TYP_UINT32,   &canO_send16Bits,	NULL},			//202x.0B comparisonResult Native
{&limitCheckParams[6].measValue			,TYP_UINT32,   &canO_send32Bits,	NULL},		//202x.0C Meas Value Native
{&limitCheckParams[6].limitCheckState,	TYP_UINT32,   &canO_send16Bits,	NULL}};			//202x.0D limitCheckState Native

// Limit Check Channel 8
const struct CAN_COMMAND index_202A[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,limchkResetOneChannel},
// (void  *)data    	               Uint16     send_funct              recv_funct
//--------------------------------   ----------- --------------------  -----------------
{&limitCheckParams[7].enableTest,	TYP_UINT32,   &canO_send16Bits,	&limchkRecvEnable}, //202x.01 1=TEST_ENABLED, 0=TEST_DISABLED
{NULL,	TYP_UINT32,   &limchkSendWhichInput, &limchkRecvWhichInput},					//202x.02 Which_Input  & No_Halt
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.03 Inner Limits
{NULL,	TYP_UINT32,   &limchkSendLimits,	&limchkRecvLimits},							//202x.04 Outer Limits
{NULL,	TYP_UINT32,   &limchkSendTimeStartEnd,	&limchkRecvTimeStartEnd},				//202x.05 Time_start, Time_End
{&limitCheckParams[7].allowedFails,	TYP_UINT32,   &limchkSendFails,	&canO_recv16Bits},	//202x.06 allowedFails
{NULL,	TYP_UINT32,   &limchkSendMinMaxValue,	NULL},									//202x.07 Min / Max Values
{NULL,	TYP_UINT32,   &limchkSendAvgValue,		NULL},									//202x.08 Average Value
{NULL,									TYP_UINT32,   &limchkSendTestStatus,	NULL},	//202x.09 testStatus + (measValue * 0x10000)
{NULL,	TYP_UINT32,   &limchkSendMeasValueClassic,		NULL},							//202x.0A measValue Classic
// -- new native commands for limit check
{&limitCheckParams[7].comparisonResult,	TYP_UINT32,   &canO_send16Bits,	NULL},			//202x.0B comparisonResult Native
{&limitCheckParams[7].measValue			,TYP_UINT32,   &canO_send32Bits,	NULL},		//202x.0C Meas Value Native
{&limitCheckParams[7].limitCheckState,	TYP_UINT32,   &canO_send16Bits,	NULL}};			//202x.0D limitCheckState Native


const struct CAN_COMMAND index_202B[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_202C[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_202D[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_202E[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_202F[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2030[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};

// RS232/485  on I/O module, compatible w/ Classic Test Station
const struct CAN_COMMAND index_2031[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
		{&sci2.sciccr,			TYP_UINT16,  &canO_send16Bits,	&canO_recv16Bits },		//2031.01
		{&sci2.scibaud,			TYP_UINT16,  &canO_send16Bits,	&canO_recv16Bits },		//2031.02
		{&sci2.rs232_not_rs485,	TYP_UINT16,  &canO_send16Bits,	canO_recv16Bits },		//2031.03
		{&sci2.idle_line,		TYP_UINT16,  &canO_send16Bits,	&canO_recv16Bits },		//2031.04
		{NULL,					TYP_UINT16,  NULL,				&sci2_init },			//2031.05
		{&sci2.tx_status,		TYP_UINT16,  &canO_send16Bits,	&canO_recv16Bits },		//2031.06
		{&sci2.rx_err_status,	TYP_UINT16,	&canO_send16Bits,	&canO_recv16Bits },		//2031.07
		{&multi_packet_buf,	TYP_OCT_STRING,  NULL,				&sci2_recvTxBuf	 },		//2031.08
		{&sci2_Rx_Buf.count_of_bytes_in_buf,TYP_UINT16, &canO_send16Bits,&canO_recv16Bits },//2031.09
		{&multi_packet_buf,  TYP_OCT_STRING,   &sci2_sendRxBuf,		NULL 			},	//2031.0A
		{NULL,  				TYP_UINT16,   		NULL,			sci2_xmit_test},	//2031.0B
		{&multi_packet_buf,	TYP_OCT_STRING,  NULL,			&sci2_recvTxBufAppend}};	//2031.0C


// 2032  Classic compatible date and time stamps
const struct CAN_COMMAND index_2032[] =  {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
{NULL, TYP_UINT32, &ts_sendDateStampClassic,  NULL        }, //2032.01
{NULL, TYP_UINT32, &ts_sendTimeStampClassic,  NULL        }};//2032.02

const struct CAN_COMMAND index_2033[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2034[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2035[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};

// 0x2036 Bootloader
//  3) Handle following CAN requests
//      2036.1 enter Boot Loader Mode
//      2036.2 Boot Loader: Download new code
//      2036.3 Boot Loader: Reset DSP
const struct CAN_COMMAND index_2036[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
	{NULL, TYP_UINT32, NULL,  			&canO_doJumpToBootLoader	}, //2036.01
	{NULL, TYP_UINT32, NULL,  			NULL				        }, //2036.02
	{NULL, TYP_UINT32, NULL,			&canO_doReset		        }};//2036.03

const struct CAN_COMMAND index_2037[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2038[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2039[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_203A[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_203B[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_203C[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_203D[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_203E[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_203F[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2040[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2041[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2042[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2043[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2044[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2045[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2046[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2047[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_2048[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};

// for Testing Simple CAN operations
const struct CAN_COMMAND index_2049[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},                              //2049.00
	// (void  *)data    	          Uint16      send_funct               recv_funct
	//-------------------   ----------- ------------------------  -----------------
	{canTestData16,         TYP_UINT32, &canO_send32Bits,         &canO_recv32Bits        }, //2049.01
	{canTestData0x98765432, TYP_UINT32, &canO_send32Bits,         NULL                    }, //2049.02
	{canTestData16,         TYP_UINT32, &canO_givesErrForTesting, &canO_givesErrForTesting}};//2049.03

// for Testing CAN multi-packet operations -- CanFile virtual file upload/download to RAM buffer
const struct CAN_COMMAND index_204A[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},                               //204A.00
	// *data16    	          Uint16      send_funct               recv_funct
	//-------------------   ----------- ------------------------  -----------------
	{&dummyFileBufInCharCount, TYP_UINT16,   &canO_send16Bits,     &canO_recv16Bits   },   //204A.01
    {&multi_packet_buf,      TYP_OCT_STRING, &canF_sendDummyFile,  &canF_recvDummyFile},   //204A.02
    {&dummyFileBufOutCharCount, TYP_UINT16,  &canO_send16Bits,     &canO_recv16Bits   },   //204A.03
    {&dummyFileBufOutPacketSize, TYP_UINT16, &canO_send16Bits,     &canO_recv16Bits   },   //204A.04
    {&multi_packet_buf,          TYP_UINT16, &canO_send16Bits,     NULL               }};  //204A.05

// used in Testing CAN multi-packet operations -- turn on/off CanFile diagnostics
const struct CAN_COMMAND index_204B[] =  { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
    // *data16    	          Uint16      send_funct               recv_funct
    //-------------------   ----------- ------------------------  -----------------
    {&canF_diagOnOff,        TYP_UINT16, &canO_send16Bits,  &canO_recv16Bits        }}; //204B.01

// for FLASH reading and writing
const struct CAN_COMMAND index_204C[] = {{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// *data16    	          Uint16      send_funct               recv_funct
//-------------------   ----------- ------------------------  -----------------
    {&frwFlashAddr,      TYP_UINT32, &canO_send32Bits,         &canO_recv32Bits  },  //204C.01
    {canTestData16,      TYP_UINT32, &frw_readFlashStatusReg,   NULL  },             //204C.02
    {canTestData16,      TYP_UINT32, &frw_readFlashRDID,        NULL  },             //204C.03
    {canTestData16,      TYP_UINT32, NULL,                 &frw_startMcsFileRecv},   //204C.04
    {canTestData16,      TYP_UINT32, &frw_mcsFileRecvStatus,    NULL  },             //204C.05
    {&multi_packet_buf,  TYP_OCT_STRING, NULL,                &frw_mcsFileRecvData}, //204C.06
    {canTestData16,      TYP_UINT32, &frw_bulkEraseFlashSend,&frw_bulkEraseFlashRecv},//204C.07
    {&mcsFileRecvParseStatus, TYP_UINT32, &canO_send16Bits,  NULL},                  //204C.08
    {canTestData16,      TYP_UINT32, NULL,              &frw_diagDisplFlashPage},    //204C.09
    {&multi_packet_buf,  TYP_OCT_STRING, &frw_mcsFileSendData,             NULL},    //204C.0A
    {&mcsFileSendByteCount,  TYP_UINT32, &canO_send32Bits,    &canO_recv32Bits },    //204C.0B
    {&mcsFileSendByteAddr,   TYP_UINT32, &canO_send16Bits,    &canO_recv16Bits },    //204C.0C
    {canTestData16,      TYP_UINT32, &frw_releasePowerdownRES,  NULL  },             //204C.0D
    {canTestData16,      TYP_UINT32, NULL, &frw_startLoadFpgaFromFlash },            //204C.0E
    {canTestData16,      TYP_UINT32, NULL, &frw_turnOnFlashWriteProtect },           //204C.0F
    {canTestData16,      TYP_UINT32, &frw_readWhichFlashChip, &frw_writeWhichFlashChip}, //204C.10
    {CPLD_F1_XA(FPGA_READ_FIRMWARE_TIMESTAMP_1), TYP_UINT32, &canO_send16Bits, NULL  }, //204C.11
    {CPLD_F1_XA(FPGA_READ_FIRMWARE_TIMESTAMP_2), TYP_UINT32, &canO_send16Bits, NULL  }, //204C.12
    {CPLD_F1_XA(FPGA_READ_FIRMWARE_TIMESTAMP_3), TYP_UINT32, &canO_send16Bits, NULL  }, //204C.13
    {CPLD_F1_XA(FPGA_READ_FIRMWARE_REVISION_1),  TYP_UINT32, &canO_send16Bits, NULL  }, //204C.14
    {CPLD_F1_XA(FPGA_READ_FIRMWARE_REVISION_2),  TYP_UINT32, &canO_send16Bits, NULL  }, //204C.15
    {CPLD_F2_XA(FPGA_READ_FIRMWARE_TIMESTAMP_1), TYP_UINT32, &canO_send16Bits, NULL  }, //204C.16
    {CPLD_F2_XA(FPGA_READ_FIRMWARE_TIMESTAMP_2), TYP_UINT32, &canO_send16Bits, NULL  }, //204C.17
    {CPLD_F2_XA(FPGA_READ_FIRMWARE_TIMESTAMP_3), TYP_UINT32, &canO_send16Bits, NULL  }, //204C.18
    {CPLD_F2_XA(FPGA_READ_FIRMWARE_REVISION_1),  TYP_UINT32, &canO_send16Bits, NULL  }, //204C.19
    {CPLD_F2_XA(FPGA_READ_FIRMWARE_REVISION_2),  TYP_UINT32, &canO_send16Bits, NULL  }, //204C.1A
    {&multi_packet_buf,  TYP_OCT_STRING,    NULL,              &frw_fastFileRecvData }, //204C.1B
	{CPLD_F3_XA(FPGA_READ_FIRMWARE_TIMESTAMP_1), TYP_UINT32, &canO_send16Bits, NULL  }, //204C.1C
	{CPLD_F3_XA(FPGA_READ_FIRMWARE_TIMESTAMP_2), TYP_UINT32, &canO_send16Bits, NULL  }, //204C.1D
	{CPLD_F3_XA(FPGA_READ_FIRMWARE_TIMESTAMP_3), TYP_UINT32, &canO_send16Bits, NULL  }, //204C.1E
	{CPLD_F3_XA(FPGA_READ_FIRMWARE_REVISION_1),  TYP_UINT32, &canO_send16Bits, NULL  }, //204C.1F
	{CPLD_F3_XA(FPGA_READ_FIRMWARE_REVISION_2),  TYP_UINT32, &canO_send16Bits, NULL  }, //204C.20
	{NULL,  									 TYP_UINT32, NULL, &frw_runTest2005 }}; //204C.21


// open / close SWITCHES for io_pins, self_test, loopback, short_integrator
const struct CAN_COMMAND index_204D[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},                                    //204D.00
	// (void  *)data    	                        Uint16     send_funct   recv_funct
	//-----------------------------------------    ----------- ----------  -----------------
	{CPLD_F2_XA(FPGA2_WRITE_IO_PIN_SWITCHES),		TYP_UINT32,   NULL,    &canO_recv16Bits },     //204D.01
	{CPLD_F2_XA(FPGA2_WRITE_SELF_TEST_SWITCHES),	TYP_UINT32,   NULL,    &canO_recv16Bits },     //204D.02
	{CPLD_F2_XA(FPGA2_WRITE_INTEGRATOR_SWITCH),		TYP_UINT32,   NULL,    &canO_recv16Bits },     //204D.03
	{CPLD_F2_XA(FPGA2_WRITE_LOOPBACK_MUX),			TYP_UINT32,   NULL,    &canO_recv16Bits },     //204D.04
	{CPLD_F2_XA(FPGA2_READ_IO_PIN_SWITCHES),		TYP_UINT32,   &canO_send16Bits,	NULL	},     //204D.05
	{CPLD_F2_XA(FPGA2_READ_SELF_TEST_SWITCHES),		TYP_UINT32,   &canO_send16Bits,	NULL	},     //204D.06
	{CPLD_F2_XA(FPGA2_READ_INTEGRATOR_SWITCH),		TYP_UINT32,   &canO_send16Bits,	NULL	},     //204D.07
	{CPLD_F2_XA(FPGA2_READ_LOOPBACK_MUX),			TYP_UINT32,   &canO_send16Bits,	NULL	},     //204D.08
	{CPLD_F2_XA(FPGA2_WRITE_ANLG_IN_B1_SWITCH),		TYP_UINT32,   NULL,    &canO_recv16Bits },     //204D.09
	{CPLD_F2_XA(FPGA2_READ_ANLG_IN_B1_SWITCH),		TYP_UINT32,   &canO_send16Bits,	NULL	}};    //204D.0A

// ADC
const struct CAN_COMMAND index_204E[] = {{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// (void  *)data    	                        Uint16     send_funct   recv_funct
//-----------------------------------------    ----------- ----------  -----------------
{CPLD_F2_XA(FPGA2_WRITE_ADC_CAPTURE),			TYP_UINT32,   NULL,    &canO_recv16Bits },     //204E.01
{CPLD_F2_XA(FPGA2_READ_ADC_A1),					TYP_UINT32,   &canO_send16Bits,	NULL	},     //204E.02
{CPLD_F2_XA(FPGA2_READ_ADC_A2),					TYP_UINT32,   &canO_send16Bits,	NULL	},     //204E.03
{CPLD_F2_XA(FPGA2_READ_ADC_A3),					TYP_UINT32,   &canO_send16Bits,	NULL	},     //204E.04
{CPLD_F2_XA(FPGA2_READ_ADC_A4),					TYP_UINT32,   &canO_send16Bits,	NULL	},     //204E.05
{CPLD_F2_XA(FPGA2_WRITE_ADC_CAPTURE_MODE),		TYP_UINT32,   NULL,    &canO_recv16Bits },     //204E.06
{CPLD_F2_XA(FPGA2_READ_ADC_A5),					TYP_UINT32,   &canO_send16Bits,	NULL	},     //204E.07
{CPLD_F2_XA(FPGA2_READ_ADC_A6),					TYP_UINT32,   &canO_send16Bits,	NULL	},     //204E.08
{CPLD_F2_XA(FPGA2_READ_ADC_A7),					TYP_UINT32,   &canO_send16Bits,	NULL	},     //204E.09
{CPLD_F2_XA(FPGA2_READ_ADC_A8),					TYP_UINT32,   &canO_send16Bits,	NULL	},     //204E.0A
{canTestData16,									TYP_UINT32,   NULL, &ain_startOffsetCalc},     //204E.0B
{(ain_offsets+0),						TYP_UINT32,   &ain_calibratedValue, 		NULL},     //204E.0C
{(ain_offsets+1),						TYP_UINT32,   &ain_calibratedValue, 		NULL},     //204E.0D
{(ain_offsets+2),						TYP_UINT32,   &ain_calibratedValue, 		NULL},     //204E.0E
{(ain_offsets+3),						TYP_UINT32,   &ain_calibratedValue, 		NULL},     //204E.0F
{(ain_offsets+4),						TYP_UINT32,   &ain_calibratedValue, 		NULL},     //204E.10
{(ain_offsets+5),						TYP_UINT32,   &ain_calibratedValue, 		NULL},     //204E.11
{(ain_offsets+6),						TYP_UINT32,   &ain_calibratedValue, 		NULL},     //204E.12
{(ain_offsets+7),						TYP_UINT32,   &ain_calibratedValue, 		NULL},     //204E.13
{&ain_ad7175_single_write_data.all,		TYP_UINT32,   NULL,				&canO_recv32Bits},     //204E.14
{canTestData16,							TYP_UINT32,   NULL,	 		&ain_ad7175_request},      //204E.15
{canTestData16,							TYP_UINT32,   &ain_ad7175_fetch_status,		NULL},	   //204E.16
{canTestData16,							TYP_UINT32,   &ain_ad7175_fetch_data_read,	NULL},	   //204E.17
{(ain_offsets+0),						TYP_UINT32,   &canO_send16Bits,				NULL},	   //204E.18
{(ain_offsets+1),						TYP_UINT32,   &canO_send16Bits,				NULL},	   //204E.19
{(ain_offsets+2),						TYP_UINT32,   &canO_send16Bits,				NULL},	   //204E.1A
{(ain_offsets+3),						TYP_UINT32,   &canO_send16Bits,				NULL},	   //204E.1B
{(ain_offsets+4),						TYP_UINT32,   &canO_send16Bits,				NULL},	   //204E.1C
{(ain_offsets+5),						TYP_UINT32,   &canO_send16Bits,				NULL},	   //204E.1D
{(ain_offsets+6),						TYP_UINT32,   &canO_send16Bits,				NULL},	   //204E.1E
{(ain_offsets+7),						TYP_UINT32,   &canO_send16Bits,				NULL}};    //204E.1F

// DSP Firmware Rev & Timestamp
const struct CAN_COMMAND index_204F[] = {{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// (void  *)data    Uint16     send_funct   recv_funct
//---------------  -------   -----------    ----------  -----------------
{(void*)(&timeStamp_t1),  TYP_UINT32, &canO_send16Bits, NULL  }, //204F.1
{(void*)(&timeStamp_t2),  TYP_UINT32, &canO_send16Bits, NULL  }, //204F.2
{(void*)(&timeStamp_t3),  TYP_UINT32, &canO_send16Bits, NULL  }, //204F.3
{(void*)(&revision_rv1),  TYP_UINT32, &canO_send16Bits, NULL  }, //204F.4
{(void*)(&revision_rv2),  TYP_UINT32, &canO_send16Bits, NULL }}; //204F.5

// SSE (Sinusoidal Encoder)
const struct CAN_COMMAND index_2050[] = {{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// (void  *)data    	                        Uint16     send_funct   recv_funct
//-----------------------------------------    ----------- ----------  -----------------
{CPLD_F2_XA(FPGA2_WRITE_SSE_ACTION),			TYP_UINT32,   NULL,    &canO_recv16Bits },     //2050.01
{CPLD_F2_XA(FPGA2_READ_SSE_1ST_16),				TYP_UINT32,   &canO_send16Bits,	NULL	},     //2050.02
{CPLD_F2_XA(FPGA2_READ_SSE_2ND_16),				TYP_UINT32,   &canO_send16Bits,	NULL	},     //2050.03
{CPLD_F2_XA(FPGA2_READ_SSE_STATUS),				TYP_UINT32,   &canO_send16Bits,	NULL	},     //2050.04
{CPLD_F2_XA(FPGA2_WRITE_SSE_DATA_1ST_16),		TYP_UINT32,   NULL,    &canO_recv16Bits },     //2050.05
{CPLD_F2_XA(FPGA2_WRITE_SSE_DATA_2ND_16),		TYP_UINT32,   NULL,    &canO_recv16Bits },     //2050.06
{CPLD_F2_XA(FPGA2_WRITE_SSE_CRC5),				TYP_UINT32,   NULL,    &canO_recv16Bits },     //2050.07
{CPLD_F2_XA(FPGA2_READ_SSE_DIAG_1),				TYP_UINT32,   &canO_send16Bits,	NULL	},     //2050.08
{CPLD_F2_XA(FPGA2_READ_SSE_DIAG_2),				TYP_UINT32,   &canO_send16Bits,	NULL	},     //2050.09
{&f2i_SSEnc_Pos_1,      				TYP_UINT32, &canO_send16Bits, &canO_recv16Bits  },     //2050.0A
{&f2i_SSEnc_Pos_2,      				TYP_UINT32, &canO_send16Bits, &canO_recv16Bits  },     //2050.0B
{&f2i_SSEnc_Num_Pos_Bits,      			TYP_UINT32, &canO_send16Bits, &canO_recv16Bits  },     //2050.0C
{&f2i_SSEnc_Alarm_Bit,      			TYP_UINT32, &canO_send16Bits, &canO_recv16Bits  },     //2050.0D
{&f2i_SSEnc_Pos_32,   			TYP_UINT32, &canO_send32Bits, &f2i_recv_Pos_32_from_Host},     //2050.0E
{&f2i_SSEnc_CRC_5,      				TYP_UINT32, &canO_send16Bits, &canO_recv16Bits  },     //2050.0F
{&ssEnc_HiPrecisShaftAngle     ,TYP_UINT16,  &canO_send16Bits  ,&ssEnc_recvShaftAngleIincreasedPrecision }, //2050.10
{&ssEnc_velocity               ,TYP_UINT16,  &canO_send16Bits  ,&canO_recv16Bits }};		    //2050.11

// for Read/Write I2C EEProms
const struct CAN_COMMAND index_2051[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},                          	//2051.00
	// *data16    	          Uint16      send_funct               recv_funct
	//-------------------   ----------- ------------------------  -----------------
	// See CanOpen.Doc -- a Word file included in this CCS project
		{&i2ceeSelectedEeprom, TYP_UINT16,  &canO_send16Bits,     &canO_recv16Bits   },		//2051.01

	// 0x2051.2 & 3 work in conjunction with 0x204A "CanFile" commands.
	// Use 0x204A to read/write 256-byte i2cEEProm image to CanFile buffer in
	// test station, use 0x2051.2 & 3 to program/read 256-byte image from
	// I2CEEProm to CanFile Buffer.
	{canTestData16, TYP_UINT32, &i2cee_getTokenForEepromProg,&i2cee_progEEPromFromCanFileData},	//2051.02
	{canTestData16, TYP_UINT32,&i2cee_readEEPromStatus, &i2cee_readEEPromToCanFile},    		//2051.03

    // 0x2051.4 & 5 PC writes to set address of byte in eeprom (0-255), then reads/writes
    // 1 byte of data. Address increments so next read gets next 1 byte of eeprom data.
   {&eeProm1ByteRWAddr, TYP_UINT32,&canO_send16Bits,       &canO_recv16Bits  },          //2051.04
   {&eeProm1ByteRWAddr, TYP_UINT32,&i2cee_readEEProm1Byte, &i2cee_writeEEProm1Byte},     //2051.05

   // 0x2051.6-9 PC reads/writes 32 byte blocks from I2CEEProm
   {&eeProm32ByteRWAddr, TYP_UINT32,&canO_send16Bits,      &canO_recv16Bits  },               //2051.06
   {canTestData16,       TYP_UINT32, &i2cee_read32ByteStatus, &i2cee_32BytesFromEEpromToBuf}, //2051.07
   {&multi_packet_buf,   TYP_OCT_STRING,    &i2cee_32BytesToPC,  &i2cee_32BytesFromPC},       //2051.08
   {canTestData16, TYP_UINT32, &i2cee_getTokenForEepromProg,&i2cee_burn32BytesToEEProm}};     //2051.09


// for reading diagnostic log information from Log.C
#ifdef LOG_ENABL_CORE_INFRASTRUCTURE
const struct CAN_COMMAND index_2052[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},                          	//2052.00
	// *data16    	          Uint16      send_funct               recv_funct
	//-------------------   ----------- ------------------------  -----------------
	{canTestData16, 		TYP_UINT32,  &log_startReadingLog,     	NULL   },				//2052.01
	{canTestData16, 		TYP_UINT32,	 &log_readLog, 				NULL},					//2052.02
	{canTestData16, 		TYP_UINT32,	  NULL, 				&log_clear},				//2052.03
    {canTestData16, 		TYP_UINT32,  &log_t0Period, 			NULL}};					//2052.04
#else
const struct CAN_COMMAND index_2052[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
#endif


// Analog Outputs / DACS -- Native scaling
const struct CAN_COMMAND index_2053[] = {	{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
	{(digio_nativeDacVal + 0),	TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.01
	{(digio_nativeDacVal + 1),	TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.02
	{(digio_nativeDacVal + 2),	TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.03
	{(digio_nativeDacVal + 3),	TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.04
	{(digio_nativeDacVal + 4),	TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.05
	{(digio_nativeDacVal + 5),	TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.06
	{(digio_nativeDacVal + 6),	TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.07
	{(digio_nativeDacVal + 7),	TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.08
	{(digio_nativeDacVal + 8),		TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.09
	{(digio_nativeDacVal + 9),		TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.0A
	{(digio_nativeDacVal + 10),		TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.0B
	{(digio_nativeDacVal + 11),		TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.0C
	{(digio_nativeDacVal + 12),		TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.0D
	{(digio_nativeDacVal + 13),		TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.0E
	{(digio_nativeDacVal + 14),		TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal },     //2053.0F
	{(digio_nativeDacVal + 15),		TYP_UINT32,   &canO_send16Bits,    &digio_receiveNativeDacVal }};    //2053.10

// PWM Frequency for Digital Outputs, 32-bit version
// see index 0x200A for 16-bit commands compatible w/ classic test station
const struct CAN_COMMAND index_2054[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// *data16    	          Uint16      send_funct               recv_funct
//-------------------   ----------- ------------------------  -----------------
{&digio_PwmOutputFreq32,	 TYP_UINT32, &canO_send32Bits, &digio_recvPwmOutputFreq32 },      //2054.01
{&digio_PwmOutputDutyCycl32, TYP_UINT32, &canO_send32Bits, &digio_recvPwmOutputDutyCyc32 }};  //2054.02

// Encoder output: Simulate 2 encoders, native 32 bit version
// see index 0x2008, 0x2009 for 16-bit commands compatible w/ classic test station
const struct CAN_COMMAND index_2055[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// *data16    	          Uint16      send_funct               recv_funct
//-------------------   ----------- ------------------------  -----------------
{&digio_Enc1OutFreq32,	 TYP_UINT32, &canO_send32Bits, &digio_recvEncOutParam },		//2055.01
{&digio_Enc1OutIndex32,	 TYP_UINT32, &canO_send32Bits, &digio_recvEncOutParam },		//2055.02
{&digio_Enc1OutDir,	     TYP_UINT32, &canO_send16Bits, &digio_recvEncOutParam },		//2055.03
{&digio_Enc2OutFreq32,	 TYP_UINT32, &canO_send32Bits, &digio_recvEncOutParam },		//2055.04
{&digio_Enc2OutIndex32,	 TYP_UINT32, &canO_send32Bits, &digio_recvEncOutParam },		//2055.05
{&digio_Enc2OutDir,      TYP_UINT32, &canO_send16Bits, &digio_recvEncOutParam },		//2055.06
{&digio_Enc1StopAfterN,	 TYP_UINT32, &canO_send32Bits, &digio_recvEncOutParam },		//2055.07
{&digio_Enc2StopAfterN,	 TYP_UINT32, &canO_send32Bits, &digio_recvEncOutParam },		//2055.08
{&digio_Enc1ManualStop,	 TYP_UINT32, &canO_send16Bits, &digio_recvEncOutParam },		//2055.09
{&digio_Enc2ManualStop,	 TYP_UINT32, &canO_send16Bits, &digio_recvEncOutParam }};  		//2055.0A

// Digital Input Machines, native 32-bit interface
// Read freq and duty cycle for 2 PWM inputs, also read encoder input freq, duty cycle, and direction.
// See 0x2013, 0x2014, 0x2015 for classic-compatible 16-bit interface
// See 0x2011 for selecting which digital inputs are input into digital machines
const struct CAN_COMMAND index_2056[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// *data16    	          Uint16      send_funct               recv_funct
//-------------------   ----------- ------------------------  -----------------
{NULL,	 TYP_UINT32, &digio2_sendPwmIn1Period, NULL },		//2056.01
{NULL,	 TYP_UINT32, &digio2_sendPwmIn1OnTime, NULL },		//2056.02
{NULL,	 TYP_UINT32, &digio2_sendPwmIn2Period, NULL },		//2056.03
{NULL,	 TYP_UINT32, &digio2_sendPwmIn2OnTime, NULL },		//2056.04
{NULL,	 TYP_UINT32, &digio2_sendEncAInPeriod, NULL },		//2056.05
{NULL,	 TYP_UINT32, &digio2_sendEncAInOnTime, NULL },		//2056.06
{NULL,	 TYP_UINT32, &digio2_sendEncIInPeriod, NULL },		//2056.07
{NULL,	 TYP_UINT32, NULL					 , NULL },		//2056.08 placeholder for unsupported EncI OnTime
{NULL,	 TYP_UINT32, &digio2_sendEncInDir, NULL },			//2056.09
{NULL,	 TYP_UINT32, &digio2_sendEncCounts, NULL },			//2056.0A
{CPLD_F2_XA(FPGA2_WRITE_DIGINMACHINE_RESET_ENC_COUNTS), TYP_UINT32, NULL, &canO_recv16Bits }}; //2056.0B - data is ignored
																						// writing any value causes reset
// 0x2057 -- CAN commands to exercise Low Level FPGA and CPLD Bi-Dir Bus tests
const struct CAN_COMMAND index_2057[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// *data16    	                 Uint16      send_funct               recv_funct
//-------------------          ----------- ------------------------  -----------------
{(Uint16*)(&led_fpga1Pattern),	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2057.01
{(Uint16*)(&led_fpga2Pattern),	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2057.02
{(Uint16*)(&led_fpga3Pattern),	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2057.03
{(Uint16*)(&led_cpldIoPattern),	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2057.04
{(Uint16*)(&led_cpldPmPattern),  TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2057.05
{CPLD_F1_XA(FPGA1_READ_0x0000),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.06
{CPLD_F1_XA(FPGA1_READ_0xFFFF),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.07
{CPLD_F1_XA(FPGA1_READ_0xA5A5),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.08
{CPLD_F1_XA(FPGA1_READ_0x5A5A),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.09
{CPLD_F2_XA(FPGA2_READ_0x0000),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.0A
{CPLD_F2_XA(FPGA2_READ_0xFFFF),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.0B
{CPLD_F2_XA(FPGA3_READ_0xA5A5),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.0C
{CPLD_F2_XA(FPGA2_READ_0x5A5A),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.0D
{CPLD_F3_XA(FPGA3_READ_0x0000),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.0E
{CPLD_F3_XA(FPGA3_READ_0xFFFF),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.0F
{CPLD_F3_XA(FPGA3_READ_0xA5A5),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.10
{CPLD_F3_XA(FPGA3_READ_0x5A5A),  TYP_UINT32, &canO_send16Bits,  NULL },					//2057.11
{CPLD_F1_XA(FPGA1_READ_STORED_VAL_1),  TYP_UINT32, &canO_send16Bits,  NULL },			//2057.12
{CPLD_F2_XA(FPGA2_READ_STORED_VAL_1),  TYP_UINT32, &canO_send16Bits,  NULL },			//2057.13
{CPLD_F3_XA(FPGA3_READ_STORED_VAL_1),  TYP_UINT32, &canO_send16Bits,  NULL },			//2057.14
{CPLD_F1_XA(FPGA1_READ_STORED_VAL_2),  TYP_UINT32, &canO_send16Bits,  NULL },			//2057.15
{CPLD_F2_XA(FPGA2_READ_STORED_VAL_2),  TYP_UINT32, &canO_send16Bits,  NULL },			//2057.16
{CPLD_F3_XA(FPGA3_READ_STORED_VAL_2),  TYP_UINT32, &canO_send16Bits,  NULL },			//2057.17
{CPLD_F1_XA(FPGA1_WRITE_STORED_VAL_1),  TYP_UINT32, NULL, &canO_recv16Bits },			//2057.18
{CPLD_F2_XA(FPGA1_WRITE_STORED_VAL_1),  TYP_UINT32, NULL, &canO_recv16Bits },			//2057.19
{CPLD_F3_XA(FPGA1_WRITE_STORED_VAL_1),  TYP_UINT32, NULL, &canO_recv16Bits },			//2057.1A
{CPLD_F1_XA(FPGA1_WRITE_STORED_VAL_2),  TYP_UINT32, NULL, &canO_recv16Bits },			//2057.1B
{CPLD_F2_XA(FPGA1_WRITE_STORED_VAL_2),  TYP_UINT32, NULL, &canO_recv16Bits },			//2057.1C
{CPLD_F3_XA(FPGA1_WRITE_STORED_VAL_2),  TYP_UINT32, NULL, &canO_recv16Bits },			//2057.1D
{CPLD_F1_XA(FPGA1_WRITE_RESET_COUNT_CLK),  TYP_UINT32, &fpgaT_send32Clk, &canO_recv16Bits},//2057.1E
{CPLD_F2_XA(FPGA2_WRITE_RESET_COUNT_CLK),  TYP_UINT32, &fpgaT_send32Clk, &canO_recv16Bits},//2057.1F
{CPLD_F3_XA(FPGA3_WRITE_RESET_COUNT_CLK),  TYP_UINT32, &fpgaT_send32Clk, &canO_recv16Bits}};//2057.20

// 0x2058 -- FPGA Test -- read/write stored values in FPGA registers
const struct CAN_COMMAND index_2058[] = { {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// *data16    	                 Uint16      send_funct               recv_funct
//-------------------          ----------- ------------------------  -----------------
{(Uint16*)(&fpgaT_sv_test_Control), TYP_UINT32, NULL, 	 &fpgaT_sv_test_ctrl },		//2058.01
{&fpgaT_Fpga1_sv1_write,	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.02
{&fpgaT_Fpga1_sv1_read,		 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.03
{&fpgaT_Fpga1_sv2_write,	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.04
{&fpgaT_Fpga1_sv2_read,		 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.05
{&fpgaT_Fpga2_sv1_write,	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.06
{&fpgaT_Fpga2_sv1_read,		 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.07
{&fpgaT_Fpga2_sv2_write,	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.08
{&fpgaT_Fpga2_sv2_read,		 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.09
{&fpgaT_Fpga3_sv1_write,	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.0A
{&fpgaT_Fpga3_sv1_read,		 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.0B
{&fpgaT_Fpga3_sv2_write,	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.0C
{&fpgaT_Fpga3_sv2_read,		 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.0D
{&fpgaT_sv_test_which_Fpgas, TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.0E
{&fpgaT_sv_test_Per_Loop, 	 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.0F
{&fpgaT_sv_test_Error, 		 TYP_UINT32, &canO_send16Bits, &canO_recv16Bits },		//2058.10
{(Uint16*)&fpgaT_sv_test_Count_Tests,  TYP_UINT32, &canO_send32Bits, &canO_recv32Bits },//2058.11
{&fpgaT_sv_test_Throw_Error,  TYP_UINT32, &canO_send16Bits, &canO_recv16Bits }};		//2058.12

const struct CAN_COMMAND index_2059[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_205A[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_205B[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_205C[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_205D[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};
const struct CAN_COMMAND index_205E[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};

// TEMPORARY -- For Experimenting with  Limit_Check primitives
const struct CAN_COMMAND index_205F[] = {{NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL},
// (void  *)data    	                        Uint16     send_funct   recv_funct
//-----------------------------------------    ----------- ----------  -----------------
{&limChkAnlgInLimitsClassic.Hi_Outer,	TYP_UINT16,   &canO_send32Bits,		&limChkAnlgInClassic}, //205F.01
{&limChkAnlgInLimitsClassic.Hi_Inner,	TYP_UINT16,   &canO_send32Bits,		&limChkAnlgInClassic}, //205F.02
{&limChkAnlgInLimitsClassic.Low_Inner,	TYP_UINT16,   &canO_send32Bits,		&limChkAnlgInClassic}, //205F.03
{&limChkAnlgInLimitsClassic.Low_Outer,	TYP_UINT16,   &canO_send32Bits,		&limChkAnlgInClassic}, //205F.04
{&limChkAnlgInLimits.Hi_Outer,			TYP_UINT16,   &canO_send32Bits,    &canO_recv32Bits },     //205F.05
{&limChkAnlgInLimits.Hi_Inner,			TYP_UINT16,   &canO_send32Bits,    &canO_recv32Bits },     //205F.06
{&limChkAnlgInLimits.Low_Inner,			TYP_UINT16,   &canO_send32Bits,    &canO_recv32Bits },     //205F.07
{&limChkAnlgInLimits.Low_Outer,			TYP_UINT16,   &canO_send32Bits,    &canO_recv32Bits },     //205F.08
{&limChkAnlgInChannel,					TYP_UINT16,   &canO_send32Bits,    &canO_recv32Bits },     //205F.09
{NULL,               					TYP_UINT16,  &limChkAnlgInComparison  ,NULL }};		   	   //205F.0A


const struct CAN_COMMAND index_2060[] = {NULL,TYP_INT8,&canO_sendMaxSubIndex,NULL};

#define MAX_CAN_COMMAND_INDEX 0x2060;

const struct CAN_INDEX can_index[] =
{		{index_2000, 0},
		{index_2001, 6},
		{index_2002, 2},
		{index_2003, 6},
		{index_2004, 6},
		{index_2005, 3},
		{index_2006, 2},
		{index_2007, 4},
		{index_2008, 3},
		{index_2009, 3},
		{index_200A, 2},
		{index_200B, 0x08},
		{index_200C, 7},
		{index_200D, 0},
		{index_200E, 4},
		{index_200F, 0},
		{index_2010, 9},
		{index_2011, 3},
		{index_2012, 2},
		{index_2013, 6},
		{index_2014, 4},
		{index_2015, 4},
		{index_2016, 0},
		{index_2017, 0},
		{index_2018, 0},
		{index_2019, 0},
		{index_201A, 0},
		{index_201B, 0},
		{index_201C, 0},
		{index_201D, 0},
		{index_201E, 0},
		{index_201F, 0},
		{index_2020, 0},
		{index_2021, 0},
		{index_2022, 0},
		{index_2023, 0x0D},
		{index_2024, 0x0D},
		{index_2025, 0x0D},
		{index_2026, 0x0D},
		{index_2027, 0x0D},
		{index_2028, 0x0D},
		{index_2029, 0x0D},
		{index_202A, 0x0D},
		{index_202B, 0},
		{index_202C, 0},
		{index_202D, 0},
		{index_202E, 0},
		{index_202F, 0},
		{index_2030, 0},
		{index_2031, 0x0C},
		{index_2032, 2},
		{index_2033, 0},
		{index_2034, 0},
		{index_2035, 0},
		{index_2036, 3},
		{index_2037, 0},
		{index_2038, 0},
		{index_2039, 0},
		{index_203A, 0},
		{index_203B, 0},
		{index_203C, 0},
		{index_203D, 0},
		{index_203E, 0},
		{index_203F, 0},
		{index_2040, 0},
		{index_2041, 0},
		{index_2042, 0},
		{index_2043, 0},
		{index_2044, 0},
		{index_2045, 0},
		{index_2046, 0},
		{index_2047, 0},
		{index_2048, 0},
		{index_2049, 3},
		{index_204A, 5},
		{index_204B, 1},
		{index_204C, 0x21},
		{index_204D, 0x0A},
		{index_204E, 0x1F},
		{index_204F, 5},
		{index_2050, 0x11},
		{index_2051, 9},

#ifdef LOG_ENABL_CORE_INFRASTRUCTURE
		{index_2052, 4},
		#else
		{index_2052, 4},
#endif

		{index_2053, 0x10},
		{index_2054, 2},
		{index_2055, 0x0A},
		{index_2056, 0x0B},
		{index_2057, 0x20},
		{index_2058, 0x12},
		{index_2059, 0},
		{index_205A, 0},
		{index_205B, 0},
		{index_205C, 0},
		{index_205D, 0},
		{index_205E, 0},
		{index_205F, 0x0A},
		{index_2060, 0}
};

#define MIN_CAN_INDEX 0x2000
#define MAX_CAN_INDEX 0x2060

//===========================================================================
// Structure used to manage multi-packet uploads and downloads.
//
//===========================================================================
// used to manage multi-packet SDO up & downloads
struct SDO_MULTI_SEGMENT {
   Uint16  in_progress;  // in progress = 1, not in progress = 0
   char  *buf_ptr;     // start of buffer for Tx or Rx
   Uint16 byte_count_so_far;  // next bytes read or written to/from *(buf_ptr + byte_count_so_far)
   Uint16 expected_total_byte_count;
   Uint16  toggle;  // toggle bit from last received message
   const struct CAN_COMMAND*  can_command_ptr; // Pointer to parameters for index.subindex
   enum CANOPEN_STATUS (*process)(const struct CAN_COMMAND* can_command, Uint16* data);  // pointer to a procedure
   Uint16  max_buff_size; // from MfgrSpcDeviceMemoryMap
} sdo_multi_segment_control_block = {0,NULL,0,0,0};

#define SDO_MS_CB sdo_multi_segment_control_block

struct MULTI_PACKET_BUF multi_packet_buf = {128,0,0}; //  buffer size = 128, count of chars = 0, 1st char = 0

void init_SDO_MS_CB(void) {
	// set initial values for sdo_multi_segment_control_block
	SDO_MS_CB.in_progress = 0;
	SDO_MS_CB.buf_ptr = NULL;
	SDO_MS_CB.byte_count_so_far = 0;
	SDO_MS_CB.expected_total_byte_count = 0;
}

Uint16 copyDataToMultiPacketBuf(char* fromPtr, Uint16 count){
	//SAFE utility to copy characters from *fromPtr into multi_packet_buf.buf
	// "count" specifies desired # of characters to copy
    // Return value is actual # of chars
	Uint16 actualCount = 0;
	Uint16 i;

	// return 0 if a CAN multi-segment operation is in progress
	if (sdo_multi_segment_control_block.in_progress != 0){
		return actualCount;
	}
	if (count > multi_packet_buf.max_char_in_buf) {
		actualCount = multi_packet_buf.max_char_in_buf;
	} else {
		actualCount = count;
	}

	for (i=0;i<actualCount;i++){
		multi_packet_buf.buff[i] = *(fromPtr++);
	}
	multi_packet_buf.count_of_bytes_in_buf = actualCount;

	return actualCount;
}

Uint16 copyPacked32BytesToMultiPacketBuf(Uint16* fromPtr){
	// Utility to copy 32 bytes of data from *fromPtr into multi_packet_buf
	// Serves i2cee_32BytesToPC() function.
	// *fromPtr" points to 16 words of packed data -- 2-bytes per word.
	// multi_packet_buf is (CCS)  char with 1 byte per 16-byte word.
	// Returns # of bytes copied: 32 is OK, 0 means the multi_packet_buf is not available

	Uint16 i;

	// return 0 if a CAN multi-segment operation is in progress
	if (sdo_multi_segment_control_block.in_progress != 0){
		return 0;
	}

	for (i=0;i<32; ){
		multi_packet_buf.buff[i++] = (*(fromPtr) >> 8) & 0x00FF;
		multi_packet_buf.buff[i++] = *(fromPtr++) & 0x00FF;
	}
	multi_packet_buf.count_of_bytes_in_buf = 32;

    return 32;
}

Uint16 copy32BytesFromMultiPacketBufToPackedBuf(Uint16* toPtr){
	// Utility to copy 32 bytes of data from multi_packet_buf *fromPtr into
	// Serves i2cee_32BytesFromPC() function.
	// *toPtr" points to 16 words of packed data buffer -- 2-bytes per word.
	// multi_packet_buf is (CCS)  char with 1 byte per 16-byte word.
	// Returns # of bytes copied: 32 is OK, 0 indicates some sort of problem.

	Uint16 i;
	Uint16 j;

	// return 0 if we don't have 32 bytes in multi-packet buffer
	if (multi_packet_buf.count_of_bytes_in_buf != 32){
		return 0;
	}

	j = 0;
	for (i=0;i<16; i++){
		*(toPtr) = (multi_packet_buf.buff[j++] << 8) & 0xFF00;
		*(toPtr++) |= (multi_packet_buf.buff[j++] & 0x00FF);
	}
    return 32;
}

//===========================================================================
// Entry point from canC_pollRecv()after detecting receipt of a CAN packet
// Here we start applying CanOpen protocol to it.
//===========================================================================

enum CANOPEN_STATUS canO_HandleCanOpenMessage(Uint16 *rcvMsg, Uint16 *xmtMsg){
	// CanComm just received an 8-byte packet and handed it off to us.
	// Here we interpret it as a CanOpen message and see what the sender is requesting,
	// and do it.
	// If we are successful we return status CANOPEN_NO_ERR, and the xmtMsg buffer
	// is ready to transmit to the sender.
	// Otherwise we return a non-zero error status.

	//Uint16 msgType;
    Uint16 index;
    Uint16 subIndex;
    Uint16 cmndSpc;
    enum REPLY_DATA_TYPE replyDataType;

    const struct CAN_COMMAND *can_command;

    enum CANOPEN_STATUS (*functPtr)(const struct CAN_COMMAND* can_command, Uint16* data);
    enum CANOPEN_STATUS canOpenStatus;

    union CANOPENMBOXA *mboxaBitsRecv = (union CANOPENMBOXA *)rcvMsg;
    union CANOPENMBOXA *mboxaBitsXmit = (union CANOPENMBOXA *)xmtMsg;

    cmndSpc = mboxaBitsRecv->exp_sdo.CmndSpc;

    if ((SDO_MS_CB.in_progress == 1)) {
    	// Multi-packet operation in progress, see if this is SEND or RECEIVE
    	if (cmndSpc == 0) {          // RECV -- 2nd, 3rd, etc in multi-packet from PC, (download)
    		canOpenStatus = canO_multiPktRecv2nd3rdEtc(SDO_MS_CB.can_command_ptr, rcvMsg, xmtMsg);
		    return canOpenStatus;
    	}else if (cmndSpc == 3) {    // SEND -- 2nd, 3rd, etc in multi-packet to PC, (upload)
    		canOpenStatus = canO_multiPktSend2nd3rdEtc(SDO_MS_CB.can_command_ptr, rcvMsg, xmtMsg);
		    return canOpenStatus;
    	}else {
    		init_SDO_MS_CB(); // assume an earlier multi-segment transmission was interrupted
		    return CANOPEN_CMND_SPC_ERR;
    	}
    }

    index = ((*rcvMsg>>8) & 0x00FF)|((*(rcvMsg+1)<<8) & 0xFF00);
    subIndex = ((*(rcvMsg+1)>>8) & 0x00FF);

    // Check to see if this is a legal index.subindex
    if ((index < MIN_CAN_INDEX) || (index > MAX_CAN_INDEX)) {
    	return CANOPEN_INDEX_ERR;
    }
    if (subIndex > can_index[index - 0x2000].max_subIndex) {
    	return CANOPEN_SUBINDEX_ERR;
    }

    // calculate pointer to CAN_COMMAND struct for index.subindex
    can_command = &((can_index[index - 0x2000].canCommand)[subIndex]);

    // From bit fields in MboxA, see if we SEND or RECV data in this message
    if (cmndSpc == 1) {          // RECV -- single packet from PC, (download)
    	                         //         or start of multi-packet from PC
        // locate function pointer for RECV for this index.subindex;
    	functPtr = (can_index[index - 0x2000].canCommand)[subIndex].recvProcess;
    	if (functPtr == NULL) return CANOPEN_NULL_FUNC_PTR_ERR;

    	// if expedite bit == 0, then this is the first packet in a multi-packet download
    	if (mboxaBitsRecv->exp_sdo.expedite == 0) {
    		canOpenStatus = canO_multiPktRecvFirst(can_command, rcvMsg, xmtMsg);
    		return canOpenStatus;
    	}

        // call the RECV function for this index.subindex
    	canOpenStatus = functPtr(can_command,rcvMsg);
        if (canOpenStatus != CANOPEN_NO_ERR) return canOpenStatus;

    	// mboxaBitsXmit->all = (mboxaBitsXmit->all & 0xFF00) | 0x60;

    	mboxaBitsXmit->exp_sdo.CmndSpc = 3;
    	mboxaBitsXmit->exp_sdo.fill0 = 0;
    	mboxaBitsXmit->exp_sdo.bytes_no_data = 0;
    	mboxaBitsXmit->exp_sdo.expedite = 0;
    	mboxaBitsXmit->exp_sdo.size_indctr = 0;


    } else if (cmndSpc == 2) {    // SEND -- single packet with data to PC {upload}
        // locate function pointer for RECV for this index.subindex;
    	functPtr = (can_index[index - 0x2000].canCommand)[subIndex].sendProcess;
    	if (functPtr == NULL) return CANOPEN_NULL_FUNC_PTR_ERR;
        // call the SEND function for this index.subindex
    	canOpenStatus = functPtr(can_command,xmtMsg);
        if (canOpenStatus != CANOPEN_NO_ERR) return canOpenStatus;

    	// As a COMPATIBILITY issue, with DSP2407 CAN_Main.c, we do not look
    	// at the expedite bit when cmndSpc == 2, instead we look at the
    	// replyDataType in the CAN_COMMAND struct, and we treat the packet as
    	// first packet in a non-expedited exchange
    	// if replyDataType is TYP_OCT_STRING,
    	// otherwise we treat it as an expedited packet
    	replyDataType = (can_index[index - 0x2000].canCommand)[subIndex].replyDataType;
    	if (replyDataType == TYP_OCT_STRING) {
    		canOpenStatus = canO_multiPktSendFirst(can_command, rcvMsg, xmtMsg);
    		return canOpenStatus;
    	}

        if (canOpenStatus != CANOPEN_NO_ERR) return canOpenStatus;

    	//mboxaBitsXmit->all = (mboxaBitsXmit->all & 0xFF00) | 0x43;

    	mboxaBitsXmit->exp_sdo.CmndSpc = 2;
    	mboxaBitsXmit->exp_sdo.fill0 = 0;
    	mboxaBitsXmit->exp_sdo.bytes_no_data = 0;
    	mboxaBitsXmit->exp_sdo.expedite = 1;
    	mboxaBitsXmit->exp_sdo.size_indctr = 1;

    } else {                     // we don't recognize this (yet)
    	return CANOPEN_CMND_SPC_ERR;
    }

    return CANOPEN_NO_ERR;
}
//===========================================================================
// These functions handle first and subsequent packets in multi-packet
// uploads and downloads.
//===========================================================================

// A little research from DSP2407 CAN_Main.c . . .
//
// Incoming packet has CmdSpc = 1 (eg. 0x20, since CmdSpc is 3 MS bits)
//     this is a request for us to Recv data from the host
//     if expedite bit = 0, then this is the first packet in a Multi-Packet receive
//     2nd, 3rd, etc packets come with CmdSpc = 0
//
// Incoming packet has CmdSpc = 2 (eg. 0x40, since CmdSpc is 3 MS bits)
//     this is a request for us to Send data back to the host
//     if data Type in CAN_COMMAND struct is TYPE_OCT_STR then this is
//        the first packet in a Multi-Packet send)
//     2nd, 3rd, etc packets come with CmdSpc = 3 (0x60)
//     NOTE: DSP2407 CAN_Main.c, does not look at the expedite bit in incoming
//     packets, and treats them as expedited (single packet) unless the
//     replyDataType in CAN_COMMAND struct is TYPE_OCT_STRING.  We must be compatible.
//
// Also be aware that CAN_Main multi-packet code may have been written using "upload" and "download"
// opposite of how those terms are used in CANopen documentation.  As you read it, just check to
// see if bytes are going from the packet into the buffer, or the other way round. -- DH

enum CANOPEN_STATUS canO_multiPktRecvFirst(const struct CAN_COMMAND *can_command, Uint16 *rcvMsg, Uint16 *xmtMsg){
	// Call here when we receive the first packet in a multi-packet upload (receive)

    union CANOPENMBOXA *mboxaBitsRecv = (union CANOPENMBOXA *)rcvMsg;
    union CANOPENMBOXA *mboxaBitsXmit = (union CANOPENMBOXA *)xmtMsg;
	struct MULTI_PACKET_BUF* mpb;

    // Type should be TYP_OCT_STRING
	if (can_command->replyDataType != TYP_OCT_STRING) {
		return CANOPEN_DATA_TYPE_ERR;
	}

	mpb = (struct MULTI_PACKET_BUF*)(can_command->datapointer);
	SDO_MS_CB.buf_ptr = (mpb->buff);
	SDO_MS_CB.max_buff_size = mpb->max_char_in_buf;
	mpb->count_of_bytes_in_buf = 0;

	// See if the sender provided a byte count
	// If sender did provide a byte count, then it is in MboxC, *(rcvMsg+2)
	SDO_MS_CB.expected_total_byte_count = *(rcvMsg+2);
	// note: according to CAN Open protocol, the byte count can go up to 2^^32, but
	//  we don't handle anything that large, so we just look at the 2 ls bytes.
	// Then, if they leave the S-bit 0, and don't specify a byte count we
	//  record it as 0 here and take the byte-count from the # actually sent.
	if (mboxaBitsRecv->exp_sdo.size_indctr == 0) {
	      SDO_MS_CB.expected_total_byte_count = 0;
	}
	if (  (SDO_MS_CB.max_buff_size > 1) // 1 is a special flag value for testing
	   && (SDO_MS_CB.expected_total_byte_count > SDO_MS_CB.max_buff_size) )
	{
	      return CANOPEN_MULTI_SEG_000_ERR;
	}
	SDO_MS_CB.in_progress = 1;
	SDO_MS_CB.can_command_ptr = can_command;
	SDO_MS_CB.process = can_command->recvProcess;;
	SDO_MS_CB.byte_count_so_far = 0;
	SDO_MS_CB.toggle = 1;  // next message we expect toggle bit to be 0

	mboxaBitsXmit->exp_sdo.CmndSpc = 3;
	mboxaBitsXmit->exp_sdo.fill0 = 0;
	mboxaBitsXmit->exp_sdo.bytes_no_data = 0;
	mboxaBitsXmit->exp_sdo.expedite = 0;
	mboxaBitsXmit->exp_sdo.size_indctr = 0;

    return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS canO_multiPktRecv2nd3rdEtc(const struct CAN_COMMAND *can_command, Uint16 *rcvMsg, Uint16 *xmtMsg){
	// Call here when we receive the 2nd, 3rd, etc. packet in a multi-packet upload (receive)
	// Design Decision: Here we use a char type receive buffer -- 1 byte of data per 16-bit memory word.
	//                  We assume the data format for our file transfers will be some sort of readable ascii.
	//                  Later when we need to work with the file contents as 16-bit binary, we will
	//                  do that conversion separately.
	//                  This will simplify debugging and maintenance.
	Uint16 i; // # bytes remaining in message
	Uint16 j; // # bytes in this packet
	char *c;  // used to point into data buffer
	Uint16 *ptr;
	struct MULTI_PACKET_BUF* mpb;

    union CANOPENMBOXA *mboxaBitsRecv = (union CANOPENMBOXA *)rcvMsg;
    union CANOPENMBOXA *mboxaBitsXmit = (union CANOPENMBOXA *)xmtMsg;
    struct MULTI_PACKET_PROTOCOL *mpPacketRecv = (struct MULTI_PACKET_PROTOCOL *)rcvMsg;
    struct MULTI_PACKET_PROTOCOL *mpPacketXmit = (struct MULTI_PACKET_PROTOCOL *)xmtMsg;

	mpb = (struct MULTI_PACKET_BUF*)(can_command->datapointer);

	if (SDO_MS_CB.in_progress == 0) return CANOPEN_MULTI_SEG_001_ERR;

	if(mboxaBitsRecv->non_exp_sdo.toggle == SDO_MS_CB.toggle) {
	   // toggle bit should toggle in each sequential message, and
	   // should differ from copy of previous msg toggle bit saved in SDO_MS_CB
	   SDO_MS_CB.in_progress = 0; // shut down the multi-segment transfer in process
	   return CANOPEN_MULTI_SEG_002_ERR;
	} else {
	   SDO_MS_CB.toggle = mboxaBitsRecv->non_exp_sdo.toggle;  // save toggle bit from current message
	}

	// set a value for local variable, i, "# bytes remaining in message"
	if (SDO_MS_CB.expected_total_byte_count == 0) {
	   // 1st message didn't include a byte count, so we check against
	   // max buffer size rather than expected total byte count
	   i = SDO_MS_CB.max_buff_size -  SDO_MS_CB.byte_count_so_far;
	   if (SDO_MS_CB.max_buff_size == 1) {  // 1 is a special flag value for testing
	       i = 8; // big enough so test can continue
	   }
	} else {
	   // 1st message did include a byte count, so we check against that
	   i = SDO_MS_CB.expected_total_byte_count -  SDO_MS_CB.byte_count_so_far;
	}

	// set a value for local variable, 6, "# bytes of data in this packet"
	j = 7 - mboxaBitsRecv->non_exp_sdo.n_unused_bytes;

    // insure # bytes of data in this packet is <= # bytes remaining in the total message
	if (i < j) {
	   // more data bytes in packet than bytes remaining in the message
		  SDO_MS_CB.in_progress = 0; // shut down the multi-segment transfer in process
	      return CANOPEN_MULTI_SEG_003_ERR;
	   }

	// go ahead and append all 7 data bytes   from packet to
	//    data buffer.  Note: buffers are declared with 7 extra words
	//  to avoid problems in the case of overfill
	c =  SDO_MS_CB.buf_ptr + SDO_MS_CB.byte_count_so_far;
	if (SDO_MS_CB.max_buff_size == 1) {  // 1 is a special flag value for testing
	   c = SDO_MS_CB.buf_ptr; // continuously write over first 7 bytes in buffer
	}
	*(c++) = mpPacketRecv->MboxA.data_byte_1;
	*(c++) = mpPacketRecv->MboxB.data_byte_2;
	*(c++) = mpPacketRecv->MboxB.data_byte_3;
	*(c++) = mpPacketRecv->MboxC.data_byte_4;
	*(c++) = mpPacketRecv->MboxC.data_byte_5;
	*(c++) = mpPacketRecv->MboxD.data_byte_6;
	*(c++) = mpPacketRecv->MboxD.data_byte_7;
	SDO_MS_CB.byte_count_so_far += j;
	mpb->count_of_bytes_in_buf += j;


   if (mboxaBitsRecv->non_exp_sdo.final_packet) {
      SDO_MS_CB.in_progress = 0;
      // our internal convention is that the word preceeding the buffer
      // receives the count of bytes downloaded, see struct MULTI_PACKET_BUF
      *(SDO_MS_CB.buf_ptr - 1) = SDO_MS_CB.byte_count_so_far;

      if (SDO_MS_CB.expected_total_byte_count != 0) {
         // 1st message did include a byte count, so we check it against what we got
         if (SDO_MS_CB.expected_total_byte_count != SDO_MS_CB.byte_count_so_far) {
   	        return CANOPEN_MULTI_SEG_004_ERR;
	     }
      }

      if (SDO_MS_CB.process != NULL) {
         // optional procedure to call at end of download
    	 ptr = (Uint16*)SDO_MS_CB.buf_ptr;
    	 // calling params: CAN_COMMAND struct for index.subindex, Uint16* pointer to received byte stream
         SDO_MS_CB.process(SDO_MS_CB.can_command_ptr,ptr);
      }
   }

   mboxaBitsXmit->all = 0;
   mboxaBitsXmit->non_exp_sdo.CmdSpc = 1;
   mboxaBitsXmit->non_exp_sdo.toggle = mboxaBitsRecv->non_exp_sdo.toggle;

   mpPacketXmit->MboxA.data_byte_1 = 0;
   mpPacketXmit->MboxB.data_byte_2 = 0;
   mpPacketXmit->MboxB.data_byte_3 = 0;
   mpPacketXmit->MboxC.data_byte_4 = 0;
   mpPacketXmit->MboxC.data_byte_5 = 0;
   mpPacketXmit->MboxD.data_byte_6 = 0;
   mpPacketXmit->MboxD.data_byte_7 = 0;

   return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS canO_multiPktSendFirst(const struct CAN_COMMAND *can_command, Uint16 *rcvMsg, Uint16 *xmtMsg){
	// Call here when we receive the first packet in a multi-packet download (send)

    union CANOPENMBOXA *mboxaBitsXmit = (union CANOPENMBOXA *)xmtMsg;
    struct MULTI_PACKET_BUF *mpb;

    // Type should be TYP_OCT_STRING
	if (can_command->replyDataType != TYP_OCT_STRING) {
		return CANOPEN_DATA_TYPE_ERR;
	}

	mpb = (struct MULTI_PACKET_BUF *)(can_command->datapointer);
	SDO_MS_CB.buf_ptr = (mpb->buff);
	SDO_MS_CB.expected_total_byte_count = mpb->count_of_bytes_in_buf;

	SDO_MS_CB.in_progress = 1;
	SDO_MS_CB.can_command_ptr = can_command;
	SDO_MS_CB.process = can_command->sendProcess;;
	SDO_MS_CB.byte_count_so_far = 0;
	SDO_MS_CB.toggle = 1;  // next message we expect toggle bit to be 0

	mboxaBitsXmit->exp_sdo.CmndSpc = 2;
	mboxaBitsXmit->exp_sdo.fill0 = 0;
	mboxaBitsXmit->exp_sdo.bytes_no_data = 0;
	mboxaBitsXmit->exp_sdo.expedite = 0;
	mboxaBitsXmit->exp_sdo.size_indctr = 0;

    return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS canO_multiPktSend2nd3rdEtc(const struct CAN_COMMAND *can_command, Uint16 *rcvMsg, Uint16 *xmtMsg){
	// Call here when we receive rewuests for the 2nd, 3rd, etc. packet in a multi-packet download (send)
	// Design Decision: Here we use a char type receive buffer -- 1 byte of data per 16-bit memory word.
	//                  We assume the data format for our file transfers will be some sort of readable ascii.
	//                  Later when we need to work with the file contents as 16-bit binary, we will
	//                  do that conversion separately.
	//                  This will simplify debugging and maintenance.
	Uint16 i; // # bytes remaining in message
	Uint16 j; // # bytes in this packet
	char *c;  // used to point into data buffer

    union CANOPENMBOXA *mboxaBitsRecv = (union CANOPENMBOXA *)rcvMsg;
    union CANOPENMBOXA *mboxaBitsXmit = (union CANOPENMBOXA *)xmtMsg;
    struct MULTI_PACKET_PROTOCOL *mpPacketXmit = (struct MULTI_PACKET_PROTOCOL *)xmtMsg;
    struct MULTI_PACKET_BUF *mpb;

	if (SDO_MS_CB.in_progress == 0) return CANOPEN_MULTI_SEG_001_ERR;

	if(mboxaBitsRecv->non_exp_sdo.toggle == SDO_MS_CB.toggle) {
	   // toggle bit should toggle in each sequential message, and
	   // should differ from copy of previous msg toggle bit saved in SDO_MS_CB
	   SDO_MS_CB.in_progress = 0; // shut down the multi-segment transfer in process
	   return CANOPEN_MULTI_SEG_002_ERR;
	} else {
	   SDO_MS_CB.toggle = mboxaBitsRecv->non_exp_sdo.toggle;  // save toggle bit from current message
	}

	// i = bytes remaining in message; j =	bytes in this packet
	i = SDO_MS_CB.expected_total_byte_count -  SDO_MS_CB.byte_count_so_far; // # bytes remaining in message
	j = i;
	if (i > 7)
	{
	   j = 7; // # bytes in this packet
	}

	mboxaBitsXmit->all = 0; // all 16 bits of MboxA, including data_byte_1 & CmdSpc, Toggle, etc

	mpb = (struct MULTI_PACKET_BUF *)(can_command->datapointer); //
	c = (mpb->buff) + SDO_MS_CB.byte_count_so_far; // chr* pointer into multi-packet buf
	// go ahead and append all 7 next data bytes from data buf
	// to packet.  Note: garbage bytes are no problem at end of last packet
	mpPacketXmit->MboxA.data_byte_1 = *(c++);
	mpPacketXmit->MboxB.data_byte_2 = *(c++);
	mpPacketXmit->MboxB.data_byte_3 = *(c++);
	mpPacketXmit->MboxC.data_byte_4 = *(c++);
	mpPacketXmit->MboxC.data_byte_5 = *(c++);
	mpPacketXmit->MboxD.data_byte_6 = *(c++);
	mpPacketXmit->MboxD.data_byte_7 = *(c++);
	SDO_MS_CB.byte_count_so_far += j;

	mboxaBitsXmit->non_exp_sdo.CmdSpc = 3;
	mboxaBitsXmit->non_exp_sdo.toggle = mboxaBitsRecv->non_exp_sdo.toggle;

	if (SDO_MS_CB.byte_count_so_far >= SDO_MS_CB.expected_total_byte_count)
	{
	   // final packet
	   mboxaBitsXmit->non_exp_sdo.final_packet = 1;
	   mboxaBitsXmit->non_exp_sdo.n_unused_bytes = 7 - j;
	   SDO_MS_CB.in_progress = 0;
//
//  - - - We actually call the process at the beginning of the multi-packet SEND - - -
//	   if (SDO_MS_CB.process != NULL)
//	   {
//	      // optional procedure to call at end of download
//	      ptr = (Uint16*)SDO_MS_CB.buf_ptr;
//	      // calling params: CAN_COMMAND struct for index.subindex, Uint16* pointer to received byte stream
//	      SDO_MS_CB.process(SDO_MS_CB.can_command_ptr,ptr);
//	   }
	}

   return CANOPEN_NO_ERR;
}


//===========================================================================
// Send and Recv Functions, activated via function pointers in CAN_COMMAND table,
// acting on data for specific CanOpen index.subindex.
//===========================================================================

enum CANOPEN_STATUS canO_recv32Bits(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received data from host, store in memory at dest = (Uint16*)can_command->datapointer
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *dest;
	dest = (Uint16*)can_command->datapointer;
	*dest++ = *(data+2); // MboxC
	*dest = *(data+3);   // MboxD
	return CANOPEN_NO_ERR;
}
enum CANOPEN_STATUS canO_send32Bits(const struct CAN_COMMAND* can_command, Uint16* data){
	// Host asks us to send data from memory location src = (Uint16*)can_command->datapointer
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *src;
	src = (Uint16*)can_command->datapointer;
	*(data+2) = *src++; //MboxC
	*(data+3) = *src;   //MboxD
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS canO_recv16Bits(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received data from host, store in memory at dest = (Uint16*)can_command->datapointer
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *dest;
	dest = (Uint16*)can_command->datapointer;
	*dest = *(data+2); // MboxC
	// MboxD ignored
	return CANOPEN_NO_ERR;
}
enum CANOPEN_STATUS canO_send16Bits(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 *src;
	src = (Uint16*)can_command->datapointer;
	*(data+2) = *src; //MboxC
	*(data+3) = 0;   //MboxD
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS canO_givesErrForTesting(const struct CAN_COMMAND* can_command, Uint16* data){
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
    // This function always returns an error message -- for testing the handling of error messages
	return CANOPEN_TEST_ERR;
}

enum CANOPEN_STATUS canO_sendMaxSubIndex(const struct CAN_COMMAND* can_command, Uint16* data){
	// response to subindex 0x00 for each index, returns # of legal subindeces.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex
	Uint16 canIndex;
	Uint16 max_subIndex;

	// Start by pulling the CAN index value out of the CAN Mailbox

	//  canIndex = ((*(data+1)) << 8) & 0xFF00; // this gives you the MS 16 bits of the CAN index
	canIndex = (((*(data+0)) >> 8) & 0x00FF);  // but we only use the LS 16 bits

     // read max_subIndex value from table
	max_subIndex = can_index[(canIndex & 0xFF)].max_subIndex;

	*(data+2) = max_subIndex; //MboxC
	*(data+3) = 0;   //MboxD
	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS canO_doReset(const struct CAN_COMMAND* can_command, Uint16* data){
	// Request to do a watchdog reset.  Here we store a non-zero value into
	// the variable "do_reset." It gets counted down each time thru the main loop
	// and when it hits 0, we execure a watchdog reset.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	co_reset = 100;

	return CANOPEN_NO_ERR;
}

enum CANOPEN_STATUS canO_doJumpToBootLoader(const struct CAN_COMMAND* can_command, Uint16* data){
	// Request to jump to main program via vector.  Here we store a non-zero value into
	// the variable "do_jump_to_main." It gets counted down each time thru the main loop
	// and when it hits 0, we go ahead and do the jump.
	// *data is MboxA of transmit Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	// Disable CPU interrupts
	DINT;

	co_jump_to_bootloader = 100;

	return CANOPEN_NO_ERR;
}
