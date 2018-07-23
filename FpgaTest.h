// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     FpgaTest.H
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef FPGATx_H
#define FPGATx_H

#include "CanOpen.H"

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessors for FpgaT non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
void fpgaT_setTestData3003(Uint16 dataWord);
void fpgaT_setTestSelection3004(Uint16 dataWord);
Uint16 fpgaT_getledStatus(void);
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

void fpgaT_initTest(void);
void fpgaT_testUnderTimer0(void);
void fpgaT_test3005(Uint16 dataWord);
void fpgaT_sv_test_one_rw(void);
void fpgaT_sv_test_Task(void);

#define FPGAT_LED_DISP_OFF 0
#define FPGAT_LED_DISP_FROM_COUNT_CLOCK 1

enum CANOPEN_STATUS fpgaT_send32Clk(const struct CAN_COMMAND *can_command,Uint16 *data);
enum CANOPEN_STATUS fpgaT_sv_test_ctrl(const struct CAN_COMMAND *can_command,Uint16 *data);

enum SVTEST_CONTROL {
	SVTEST_CTRL_STOP		= 0, // Stop RUN mode
	SVTEST_CTRL_STEP		= 1, // Increment values and test once
	SVTEST_CTRL_RUN			= 2, // Iterate incrementing values and testing
	SVTEST_CTRL_TEST		= 3  // test once without incrementing values
};

extern Uint16 fpgaT_Fpga1_sv1_write;
extern Uint16 fpgaT_Fpga1_sv1_read;
extern Uint16 fpgaT_Fpga1_sv2_write;
extern Uint16 fpgaT_Fpga1_sv2_read;
extern Uint16 fpgaT_Fpga2_sv1_write;
extern Uint16 fpgaT_Fpga2_sv1_read;
extern Uint16 fpgaT_Fpga2_sv2_write;
extern Uint16 fpgaT_Fpga2_sv2_read;
extern Uint16 fpgaT_Fpga3_sv1_write;
extern Uint16 fpgaT_Fpga3_sv1_read;
extern Uint16 fpgaT_Fpga3_sv2_write;
extern Uint16 fpgaT_Fpga3_sv2_read;
extern Uint16 fpgaT_sv_test_which_Fpgas;
extern Uint16 fpgaT_sv_test_Per_Loop;
extern enum SVTEST_CONTROL fpgaT_sv_test_Control;
extern Uint16 fpgaT_sv_test_Error;
extern Uint32 fpgaT_sv_test_Count_Tests;
extern Uint16 fpgaT_sv_test_Throw_Error;

#define SVTEST_FPGA1 0x1
#define SVTEST_FPGA2 0x2
#define SVTEST_FPGA3 0x4


#endif
