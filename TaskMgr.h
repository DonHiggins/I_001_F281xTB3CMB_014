// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     TaskMgr.H
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifndef TSKMGRx_H
#define TSKMGRx_H

void taskMgr_init(void);
void taskMgr_setTask(Uint16);
void taskMgr_setTaskWithDelay(Uint16 task_number, Uint16 delayInSecTenths);
void taskMgr_setTaskRoundRobin(Uint16 task_number, Uint16 delayInSecTenths);
void taskMgr_runBkgndTasks(void);
void taskMgr_nulTask(void);
void taskMgr_wDogReset(void);
void taskMgr_ageTaskDelays(void);

void taskMgr_testTask_1(void);
void taskMgr_testTask_2(void);
void taskMgr_testTask_3(void);
void taskMgr_testTask_4(void);
void taskMgr_testTask_5(void);
void taskMgr_testSchedAlgorithms(Uint16 dataWord);


// Create a user type called TASK
//  pointer to function: void(task_name)(void):
typedef void(*TASK)(void);

// Eash task gets its own number starting at 0, 1, 2, 3, . . .
// order of task ordinal numbers in enum TaskNumber, coordinates with
// order of task entry points in task_vectors[] table in TaskMgr.c
// eg: task_vectors[TASKNUM_BgTask_echo]is rs232_BgTask_echo.

enum TaskNumber {
	TASKNUM_taskMgr_nulTask_0,
	TASKNUM_BgTask_TxDone,
	TASKNUM_F2Int_SSEnc,
	TASKNUM_taskMgr_nulTask_3,
	TASKNUM_timer0_task,
	TASKNUM_F1Int,
	TASKNUM_taskMgr_nulTask_6,
	TASKNUM_taskMgr_nulTask_7,
	TASKNUM_taskMgr_nulTask_8,
	TASKNUM_taskMgr_nulTask_9,
	TASKNUM_timer0_miliSecTask,
	TASKNUM_fpgaT_sv_test_Task,
	TASKNUM_taskMgr_nulTask_C,
	TASKNUM_taskMgr_nulTask_D,
	TASKNUM_taskMgr_nulTask_E,
	TASKNUM_taskMgr_nulTask_F,

	TASKNUM_taskMgr_nulTask_10,
	TASKNUM_BgTask_ooad,
	TASKNUM_RS232_commandDecode,
	TASKNUM_r232Out_circBufOutput,
	TASKNUM_BgTask_ts3StartUp,
	TASKNUM_wDogReset,
	TASKNUM_SpiFlashTask,
	TASKNUM_DisplayComintSpeedDialList,
	TASKNUM_MiscFlashTasks,
	TASKNUM_DiagFlashTasks,
	TASKNUM_i2cee_diag4203Task,
	TASKNUM_DisplayAdcResults,
	TASKNUM_Rs232BaudChange,
	TASKNUM_RS232_breakOrError,
	TASKNUM_RS232_ackAutobaud,
	TASKNUM_testTask_1,

	TASKNUM_testTask_2,
	TASKNUM_testTask_3,
	TASKNUM_testTask_4,
	TASKNUM_ain_offsetCalcTask,
	TASKNUM_i2cee_readEEPromToCanFileTask,
	TASKNUM_i2cee_progEEPromFromCanFileTask,
	TASKNUM_i2cee_read32FromEEpromToBufTask,
	TASKNUM_i2cee_burn32ToEEpromTask,
	TASKNUM_timer0_tenthOfSecTask,
	TASKNUM_ain_ad7175_setup_task,
	TASKNUM_testTask_5,
	TASKNUM_res_ShaftAngleIincreasedPrecision,
	TASKNUM_ssEnc_ShaftAngleIincreasedPrecision,
	TASKNUM_DigIO_Pwm_Freq_Out16,
	TASKNUM_taskMgr_nulTask_2E,
	TASKNUM_main_startupTask,

	MAX_NUMBER_OF_TASKS
};
#define MAX_TASKFLAG_WORDS ((MAX_NUMBER_OF_TASKS + 15)/16)

enum TASK_SCHED_SCHEME {
	TASK_SCHED_PRIORITY   = 0,
	TASK_SCHED_ROUNDROBIN = 1
};




#endif
