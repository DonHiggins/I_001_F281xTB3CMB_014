// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     TaskMgr.c
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "RS232.h"
#include "Rs232Out.h"
#include "Timer0.h"
#include "TaskMgr.h"
#include "FlashRW.H"
#include "Comint.h"
#include "I2CEE.h"
#include "ADC.H"
#include "F1Int.H"
#include "F2Int.H"
#include "AnlgIn.H"
#include "stdbool.h"            // needed for bool data types
#include "Main.H"
#include "Resolver.H"
#include "SSEnc.H"
#include "DigIO.H"
#include "FpgaTest.H"

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// TASK MANAGER FEATURES
//   Tasks are activities run in the background -- from the main loop,
//   not in interrupts.
//   Task functions are typed as (void*)funct_name(void)
//
//   For example, an interrupt routine may want to kick off some activity to
//   run in the background, and it does so by calling taskMgr_setTask(),
//   with a parameter to identify the desired task.  Other task-setting calls
//   specify a time-delay before the task -- taskMgr_setTaskWithDelay( ),
//   or the option to use Round-Robin scheduling rather than Priority scheduling.
//
//   taskMgr_setTask() sets the corresponding bit in one of the words in taskFlags[]
//
//   Some time later, when the main loop calls taskMgr_runBkgndTasks(), it sees the
//   bit set in taskFlags[] and calls the appropriate task.
//
//   DELAY: calls to taskMgr_setTaskWithDelay() store a time delay value -- 0.1 sec
//   units -- into the task's entry in the taskDelays[] array.  Non-zero delay values
//   are aged (decremented) by a background task kicked off from the Timer0 interrupt.
//   the function taskMgr_runBkgndTasks() won't kick off a task unless it's
//   taskDelays[] entry is 0.
//
//   SCHEDULING:  Priority scheduling (the default) means that the taskMgr_runBkgndTasks()
//   function always runs the highest priority task scheduled.  Each time it is called,
//   it starts with task #0 in its search for the next scheduled task.  If a task reschedules
//   itself, then it blocks lower priority tasks (higher task #'s) from getting launched
//   the next time through taskMgr_runBkgndTasks().  And, to make the point, if the task
//   continues to reschedule itself every time it runs, than lower priority tasks will never
//   run.  As an alternative, we provide a taskMgr_setTaskRoundRobin() call to request
//   that a task not be allowed to block lower priority tasks.
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// TASK MANAGER -- HOW TO USE IT
//
//   (1) Put your task code into a task procedure -- a "void yourTaskName(void)".
//   (2) Add your task procedure name, eg. "yourTaskName"into the task_vectors[] table
//       below. If it's not a high priority task, then just add it to the bottom of
//       table. If it is a high priority task, then consider using it to replace one
//       of the "taskMgr_nulTask" entries near the top of the table.  I left a bunch
//       of "taskMgr_nulTask" entries as place-holders for future high priority tasks.
//   (3) Add an entry into "enum TaskNumber{" creating a mnemonic to represent your
//       new task.  The value of the enum gives the index into task_vectors[] for
//       your task.  All I'm saying is -- put your new task mnemonic into "enum TaskNumber{"
//       in the same relative place that you added your task procedure name into
//       "task_vectors[]".
//   (4) Set your task to run by calling one of these routines
//          void taskMgr_setTask(Uint16 task_number)
//          void taskMgr_setTaskWithDelay(Uint16 task_number, Uint16 delayInSecTenths)
//          void taskMgr_setTaskRoundRobin(Uint16 task_number, Uint16 delayInSecTenths)
//   (5) Priority and Scheduling -- Tasks That Re-Schedule Themselves
//       Say, for example you have a task that checks to see if something has
//       completed, and if not, then the task reschedules itself to check again
//       later.  In this case, you probably want to reschedule with either a
//       time-delay or Round-Robin scheduling.  Otherwise you risk having all
//       lower priority tasks blocked from running until your task  completes.
//   (6) Priority -- Tasks Providing Services  For Other Tasks
//       The "r232Out_circBufOutput" task provides service for any other tasks
//       that generate diagnostic RS232 output.  This works best if the
//       "r232Out_circBufOutput" task is higher priority than any other tasks that
//       use its services.
//
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void taskMgr_nulTask(void);
Uint16 taskMgr_firstSetTaskFlag();

// order of task entry points in this table is coordinated with the
// order of task ordinal numbers in enum TaskNumber in TaskMgr.h,
// eg: task_vectors[TASKNUM_BgTask_echo]is rs232_BgTask_echo.

const TASK task_vectors[] = {

		taskMgr_nulTask,		// 0x00
		rs232_BgTask_TxDone,	// 0x01
		f2i_BgTask_SSEnc,		// 0x02
		taskMgr_nulTask,		// 0x03
		timer0_task,			// 0x04
		f1i_BgTask,				// 0x05
		taskMgr_nulTask,		// 0x06
		taskMgr_nulTask,		// 0x07
		taskMgr_nulTask,		// 0x08
		taskMgr_nulTask,		// 0x09
		timer0_miliSecTask,		// 0x0A
		fpgaT_sv_test_Task,		// 0x0B
		taskMgr_nulTask,		// 0x0C
		taskMgr_nulTask,		// 0x0D
		taskMgr_nulTask,		// 0x0E
		taskMgr_nulTask,		// 0x0F

		taskMgr_nulTask,				// 0x10
		rs232_BgTask_ooad,				// 0x11
		rs232_commandDecode,			// 0x12
		r232Out_circBufOutput,			// 0x13
		rs232_BgTask_ts3StartUp,		// 0x14
		taskMgr_wDogReset,				// 0x15
		frw_SpiFlashTask,				// 0x16
		comint_DisplaySpeedDialList,	// 0x17
		frw_MiscFlashTasks,				// 0x18
		frw_diagFlashTasks,				// 0x19
		i2cee_diag4203Task,				// 0x1A
		adc_DisplayAdcResults,			// 0x1B
		rs232_BgTask_Rs232BaudChange,	// 0x1C
		rs232_BgTask_Rs232BreakOrError,	// 0x1D
		rs232_BgTask_AckAutobaud,		// 0x1E
		taskMgr_testTask_1,				// 0x1F

		taskMgr_testTask_2,				// 0x20
		taskMgr_testTask_3,				// 0x21
		taskMgr_testTask_4,				// 0x22
		ain_offsetCalcTask,				// 0x23
		i2cee_readEEPromToCanFileTask,	// 0x24
		i2cee_progEEPromFromCanFileTask,// 0x25
		i2cee_read32FromEEpromToBufTask,// 0x26
		i2cee_burn32ToEEpromTask,		// 0x27
		timer0_tenthOfSecTask,			// 0x28
		ain_ad7175_setup_task,			// 0x29
		taskMgr_testTask_5,				// 0x2A
		res_ShaftAngleOutTask,			// 0x2B
		ssEnc_ShaftAngleOutTask,	    // 0x2C
		digio_PwmOutputFreq16Task,		// 0x2D
		taskMgr_nulTask,				// 0x2E
		main_startupTask				// 0x2F
};

Uint16 taskFlags[((MAX_NUMBER_OF_TASKS + 15)/16)]; // rounds up (MAX_NUMBER_OF_TASKS/16)
Uint16 taskRoundRobinFlags[((MAX_NUMBER_OF_TASKS + 15)/16)]; // rounds up (MAX_NUMBER_OF_TASKS/16)
Uint16 prevTaskNumber;
enum TASK_SCHED_SCHEME schedulingScheme;
Uint16 JustForDebug;

Uint16 taskDelays[MAX_NUMBER_OF_TASKS];

void taskMgr_init(void){
	int i;
	for (i = 0; i < MAX_TASKFLAG_WORDS; i++){
		taskFlags[i] = 0;
	}
	for (i = 0; i < MAX_NUMBER_OF_TASKS; i++){
		taskDelays[i] = 0;
	}

	prevTaskNumber = MAX_NUMBER_OF_TASKS;
	schedulingScheme = TASK_SCHED_PRIORITY;
}

// Set a bit in the task table, to schedule future running of a task.
// NOTE: not yet protected from bad input
void taskMgr_setTask(Uint16 task_number){
	Uint16 taskFlag_wordOffset;
	Uint16 taskFlag_bit;

	taskFlag_wordOffset = task_number >> 4;
	taskFlag_bit = 0x0001 << (task_number & 0x000F);
	taskFlags[taskFlag_wordOffset] |= taskFlag_bit;
}

// Schedule running a task after a prescribed time delay.
void taskMgr_setTaskWithDelay(Uint16 task_number, Uint16 delayInSecTenths){

	taskMgr_setTask(task_number);
	taskDelays[task_number] = delayInSecTenths;
}

// Schedule running a task after a prescribed time delay (possibly 0).
// And Set the Round-Robin flag for that task, so that after the task runs
// we run any requested lower priority tasks (higher task #), before returning to
// priority scheduling.
void taskMgr_setTaskRoundRobin(Uint16 task_number, Uint16 delayInSecTenths){
	Uint16 taskRRFlag_wordOffset;
	Uint16 taskRRFlag_bit;

	taskMgr_setTaskWithDelay(task_number, delayInSecTenths);
	taskRRFlag_wordOffset = task_number >> 4;
	taskRRFlag_bit = 0x0001 << (task_number & 0x000F);
	taskRoundRobinFlags[taskRRFlag_wordOffset] |= taskRRFlag_bit;
}

void taskMgr_nulTask(void){
	JustForDebug++;
}

// Check the task flags,
// Run the task corresponding to the first non-zero flag you find.
void taskMgr_runBkgndTasks (void){
	Uint16 taskNumber;

	taskNumber = taskMgr_firstSetTaskFlag();
	prevTaskNumber = taskNumber; // save this for next time thru here
	if (taskNumber >= MAX_NUMBER_OF_TASKS){
		// no task flags set -- we reached the end of the list
		// If we were operating under round robin scheduling, then we got all the
		// way to the end of the task list -- gave everybody a chance -- so now
		// we can go back to priority scheduling.
		schedulingScheme = TASK_SCHED_PRIORITY;
		return;
	} else {
		// Run the task, given an integer index into an array of function pointers
		// Either of the following 2 forms succeeds to call through a table of pointers
		(*task_vectors[taskNumber])();
		// task_vectors[taskNumber](); This form also does the same thing
	}
}

// Search taskFlags[] for first flag that is Set,(non-0), meaning "run the task"
//   AND where taskDelays[i] == 0, meaning "no wait-time remaining before running the task."
// Reset the flag, and
// Return corresponding task number for first set flag
// Or return MAX_NUMBER_OF_TASKS if no flag is set
Uint16 taskMgr_firstSetTaskFlag(){
	Uint16 i,j;
	Uint16 taskNumber=0;
	Uint16 flags;
	Uint16 shiftedBit;

	for (i = 0; i < MAX_TASKFLAG_WORDS; i++){
		flags = taskFlags[i];
		if (flags == 0) {        // if all 16 bits are 0, don't bother shifting through them
			taskNumber += 16;    // just advance to the next task word
	        if (taskNumber >= MAX_NUMBER_OF_TASKS){
				return taskNumber;
	        }
			continue;     // skip to next iteration of for loop
		}
		shiftedBit = 0x0001;
		for (j = 0;j<16;j++){

			if((flags & shiftedBit) && (taskDelays[taskNumber] == 0)) {
				//*** FOUND a flag set
				if ((schedulingScheme == TASK_SCHED_PRIORITY)
            	|| ((taskNumber > prevTaskNumber) || (prevTaskNumber >= MAX_NUMBER_OF_TASKS))) {
            	   //If we are doing priority scheduling, we look at first task we find.
            	   //If previous task requested round robin scheduling, then we only consider
            	   // higher numbered tasks.
					taskFlags[i] ^= shiftedBit; //XOR to reset task flag bit
					if (taskRoundRobinFlags[i] & shiftedBit){
						schedulingScheme = TASK_SCHED_ROUNDROBIN;
					}
					return taskNumber;
				}
			}
			taskNumber++;
			shiftedBit <<= 1; // 1 bit left shift
		}
        if (taskNumber >= MAX_NUMBER_OF_TASKS){
			return taskNumber;
        }
	}
	return taskNumber;
}

void taskMgr_ageTaskDelays(void){
	// Called each 0.1 sec from a background task kicked of by Timer0
	// We decrement any non-zero entries in the taskDelays[] table.
	// Non zero entries in the taskDelays[] table block the task from running
	// until after we decrement them to zero.
	// We don't start any tasks here, that's done elsewhere.

	int i;
	for (i = 0; i < MAX_NUMBER_OF_TASKS; i++){
		if (taskDelays[i] != 0){
			taskDelays[i]--;
		}
	}
}

// ==========================================================================
//    T A S K S
// ==========================================================================

void taskMgr_wDogReset(void){
	// Background task to use watchdog to reset the DSP
	// Monitors the rs232 output and waits until everything has been transmitted
    if ((r232Out_transmit_status_busy()) | (rs232_txFifo_Busy())){
    	taskMgr_setTask(TASKNUM_wDogReset); // re-queue this task
    	return;
    }
    EALLOW;
    //SysCtrlRegs.WDCR= 0x0068; // this disables watchdog
    SysCtrlRegs.WDCR= 0x0028; // lets see if this enables watchdog
    SysCtrlRegs.WDKEY= 0x0000; // Anything other than writing 0x55 immediately
                               // followed by 0xAA causes a watchdog reset
    EDIS;

}

// ==========================================================================
//    T A S K S for testing Task Manager Features
// ==========================================================================
char msg_testTask_1[] = {"PRI Task_1\n\r"};
char msg_testTask_2[] = {"RR Task_2\n\r"};
char msg_testTask_3[] = {"PRI Task_3\n\r"};
char msg_testTask_4[] = {"RR Task_4\n\r"};
char msg_testTask_5[] = {"PRI Task_5\n\r"};
char msg_illegalOption[] = {"Illegal Option\n\r"};

Uint16 testTaskCount;
void taskMgr_testTask_1(void){
	/* success = */ r232Out_outCharsNT(msg_testTask_1);
	if (testTaskCount++ > 20) {return;} // exit w/out re-launching task
	taskMgr_setTask(TASKNUM_testTask_1); // re-queue this task
}

void taskMgr_testTask_2(void){
	/* success = */ r232Out_outCharsNT(msg_testTask_2);
	if (testTaskCount++ > 20) {return;} // exit w/out re-launching task
	taskMgr_setTaskRoundRobin(TASKNUM_testTask_2, 0);
}
void taskMgr_testTask_3(void){
	/* success = */ r232Out_outCharsNT(msg_testTask_3);
	if (testTaskCount++ > 20) {return;} // exit w/out re-launching task
	taskMgr_setTask(TASKNUM_testTask_3); // re-queue this task
}

void taskMgr_testTask_4(void){
	/* success = */ r232Out_outCharsNT(msg_testTask_4);
	if (testTaskCount++ > 20) {return;} // exit w/out re-launching task
	taskMgr_setTaskRoundRobin(TASKNUM_testTask_4, 0);
}
void taskMgr_testTask_5(void){
	/* success = */ r232Out_outCharsNT(msg_testTask_5);
	if (testTaskCount++ > 20) {return;} // exit w/out re-launching task
	taskMgr_setTask(TASKNUM_testTask_5); // re-queue this task
}

void taskMgr_testSchedAlgorithms(Uint16 dataWord){
// called from C1020:ddddCr command in comint.c
// dataWord is dddd value, it specifies which test to run.

	testTaskCount = 0;

	switch(dataWord){
    case 0x0001: // Task1 Priority, Task2 RoundRobin
    	taskMgr_setTask(TASKNUM_testTask_1);
    	taskMgr_setTaskRoundRobin(TASKNUM_testTask_2, 0);
        break;
    case 0x0002: // Task2 RoundRobin, Task3 Priority,
    	taskMgr_setTask(TASKNUM_testTask_3);
    	taskMgr_setTaskRoundRobin(TASKNUM_testTask_2, 0);
        break;
    case 0x0003: // Task2 RoundRobin, Task3 Priority, Task4 RoundRobin
    	taskMgr_setTask(TASKNUM_testTask_3);
    	taskMgr_setTaskRoundRobin(TASKNUM_testTask_2, 0);
    	taskMgr_setTaskRoundRobin(TASKNUM_testTask_4, 0);
        break;
    case 0x0004: // Task2 RoundRobin, Task3 Priority, Task5 Priority
    	taskMgr_setTask(TASKNUM_testTask_3);
    	taskMgr_setTask(TASKNUM_testTask_5);
    	taskMgr_setTaskRoundRobin(TASKNUM_testTask_2, 0);
        break;
    case 0xFFFF: // Reset All Tasks & RoundRobin Flags
    	taskMgr_init();
        break;
    default:
    	/* success = */ r232Out_outCharsNT(msg_illegalOption);
   	break;
   }
}
