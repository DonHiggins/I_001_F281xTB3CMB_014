// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//     FpgaTest.C
//
//   Initialize, configure, and test the FPGA
//
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "Rs232Out.H"
#include "stdbool.h"            // needed for bool data types
#include "HexUtil.H"
#include "StrUtil.H"
#include "CanOpen.H"
#include "FpgaTest.h"
#include "CPLD.h"
#include "TaskMgr.H"

void fpgaT_init_3004_5000_Test(void);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// fpgaT non-public variables
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
Uint16 fpgaT_testData3003;
Uint16 fpgaT_testSelection3004;
Uint16 fpgaT_testUnderTimer0_count;
Uint16 fpgaT_ledStatus;
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// Accessor for Xintf non-public variables
void fpgaT_setTestData3003(Uint16 dataWord){
	fpgaT_testData3003 = dataWord;
}
void fpgaT_setTestSelection3004(Uint16 dataWord){
	fpgaT_testSelection3004 = dataWord;
}
Uint16 fpgaT_getledStatus(void){
	return fpgaT_ledStatus;
}

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

void fpgaT_initTest(void){
	fpgaT_testSelection3004 = 0; // turns testing off
	fpgaT_testUnderTimer0_count = 0;
	fpgaT_testData3003 = 0x0001;
	fpgaT_init_3004_5000_Test();

	fpgaT_test3005(0x1105); // turn all 4 LED's off
}

Uint16 fpgaT_3004_5000_count;
Uint16 fpgaT_3004_5000_passes;
Uint16 fpgaT_3004_5000_errors;
Uint16 fpgaT_3004_5000_immediate_report;
Uint16 fpgaT_3004_5000_immediate_err_test;

void fpgaT_init_3004_5000_Test(void){
	fpgaT_3004_5000_count = 0;
	fpgaT_3004_5000_passes = 0;
	fpgaT_3004_5000_errors = 0;
	fpgaT_3004_5000_immediate_report = 0;
	fpgaT_3004_5000_immediate_err_test = 0;
}


void fpgaT_testUnderTimer0(void){
	// Called periodically from Timer0 task
	// Perform something to exercise the Fpga based
	// on state value in fpgaT_testSelection3004

	Uint16 *extData_B1_01;
	Uint16 *extData_B1_02;
	Uint16 *extData_B2_01;
	Uint16 *extData_B2_02;
	Uint16 tData_11;
	Uint16 tData_12;
	Uint16 tData_21;
	Uint16 tData_22;
	Uint16 tempData16;
	Uint16 tempData16_2;
	// bool success;
	char msgOut[64];
    char *ptr;
	Uint16 i;

	fpgaT_testUnderTimer0_count++;

	switch(fpgaT_testSelection3004){
    case 0x0000: // test turned off
    	fpgaT_init_3004_5000_Test(); // initialize variables used in test C3004:5000
    	return;
        // break; // commented out because it yields a compiler warning as "unreachable"

    case 0x0001: // Configure FPGA LED's -- LED_DIRECTLY_FROM_DSP
    	//
    	// WRITE_LED_FUNCTION = 8'h0004 directs one of three sources for 4-bit LED output
    	//   LED_DIRECTLY_FROM_DSP            = 2'b00;
    	//   LED_FROM_COUNT_CLK               = 2'b01; // bits[28:25] counts up at 0.89 sec
    	//   LED_FROM_COUNT_WR_STORERD_VAL_1  = 2'b10;
    	//   SLOW_HEARTBEAT                   = 2'b11;
    	// WRITE_LED_DIRECTLY = 8'h0005; stores a 4-bit value from DSP to display in LEDs
    	//
    	//  LED's are connected to 1st instance of the bi-directional bus,
    	//  associated with ~CS_FPGA_1 which is asserted LOW for DSP addresses 0x09,0000 – 0x09,FFFF
    	//
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,4);
    	extData_B1_02 = (Uint16 *)0x0800FB; // CPLD_TP_0 (eg NOT ~CS_FPGA_1)
       	*extData_B1_01 = 0x0000; // write config value: LED_DIRECTLY_FROM_DSP
        *extData_B1_02 = 0xFFFF; // write access out of target area, toggle all data lines
        break;

    case 0x0002: // Configure FPGA LED's -- LED_FROM_COUNT_CLK
    	//
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,4);
    	extData_B1_02 = (Uint16 *)0x0800FB; // CPLD_TP_0 (eg NOT ~CS_FPGA_1)
       	*extData_B1_01 = 0x0001; // write config value: LED_FROM_COUNT_CLK
        *extData_B1_02 = 0xFFFE; // write access out of target area, toggle all data lines
        break;

    case 0x0003: // Configure FPGA LED's -- LED_FROM_COUNT_WR_STORERD_VAL_1
    	//
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,4);
    	extData_B1_02 = (Uint16 *)0x0800FB; // CPLD_TP_0 (eg NOT ~CS_FPGA_1)
       	*extData_B1_01 = 0x0002; // write config value: LED_FROM_COUNT_WR_STORERD_VAL_1
        *extData_B1_02 = 0xFFFD; // write access out of target area, toggle all data lines
        break;

    case 0x0004: // FPGA LED's -- write a data value to display in LED's
    	//  WRITE_LED_DIRECTLY = 8'h0005; stores a 4-bit value from DSP to display in LEDs
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,5);
    	extData_B1_02 = (Uint16 *)0x0800FA; // CPLD_TP_0 (eg NOT ~CS_FPGA_1)
       	*extData_B1_01 = fpgaT_testData3003; // write data value for display in LED's
        *extData_B1_02 = fpgaT_testData3003 ^ 0xFFFF; // write access out of target area, toggle all data lines
        break;

    case 0x4000: // Periodically read 2-word count clk
    	if ((fpgaT_testUnderTimer0_count & 0x0003) == 0){
        	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,9); // Count Clk HI (throw away)
        	tempData16 = *extData_B1_01;
        	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,8); // Count Clk LOW
        	tempData16 = *extData_B1_01;
        	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,9); // Count Clk HI (keep this one)
        	tempData16_2 = *extData_B1_01;

            // Transmit results out rs232
            ptr = strU_strcpy(msgOut,"Count Clk F1 0x");
            ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16_2);
            ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
            ptr = strU_strcpy(ptr,"\n\r");
            /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	}
    	break;

    case 0x4001: // Periodically read 2-word count clk FPGA2 TB3IOMB
    	if ((fpgaT_testUnderTimer0_count & 0x0003) == 0){
        	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR,9); // Count Clk HI (throw away)
        	tempData16 = *extData_B1_01;
        	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR,8); // Count Clk LOW
        	tempData16 = *extData_B1_01;
        	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR,9); // Count Clk HI (keep this one)
        	tempData16_2 = *extData_B1_01;

            // Transmit results out rs232
            ptr = strU_strcpy(msgOut,"Count Clk F2 0x");
            ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16_2);
            ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
            ptr = strU_strcpy(ptr,"\n\r");
            /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	}
    	break;

    case 0x5000: // Read/Write Stored Values 1 & 2, test all 65536 -- 2 busses on FPGA in TB3IOMA
    case 0x5001: // Read/Write Stored Values 1 & 2, test all 65536 -- FPGA1 & FPGA2 on TB3IOMB
    	// (this runs continually, reporting to RS232)
    	for(i=0;i<512;i++){
    		fpgaT_3004_5000_count++;

    		// do read & write and check results
    		// 4 data values: N, N^0xFFFF, 0-N, (0-N)^0xFFFF
        	tData_11 = fpgaT_3004_5000_count;
        	tData_12 = fpgaT_3004_5000_count ^ 0xFFFF;
        	tData_21 = (0 - fpgaT_3004_5000_count);
        	tData_22 = (0 - fpgaT_3004_5000_count) ^ 0xFFFF;
            // Addresses for R & W
        	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,1);
        	extData_B1_02 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,5);
        	if (fpgaT_testSelection3004 == 0x5000){
        	   extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR,0x81);
        	   extData_B2_02 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR,0x85);
        	} else {
         	   extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR,0x01);
         	   extData_B2_02 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR,0x05);
        	}
        	// write 4 data values
        	*(extData_B1_01++) = tData_11; // write N to STORED_VAL_1 Bus 1
           	*(extData_B1_01) = tData_12; // write inverted N value to STORED_VAL_2 Bus 1
        	*(extData_B2_01++) = tData_21; // write (0 - N) to STORED_VAL_1 Bus 2
           	*(extData_B2_01) = tData_22; // write inverted (0 - N) value to STORED_VAL_2 Bus 2

            // Read back 4 data values
           	tData_11 = *(extData_B1_02++);
           	tData_12 = *extData_B1_02;
           	tData_21 = *(extData_B2_02++);
           	tData_22 = *extData_B2_02;

    		// if we we don't read back what we have written then we have an error, report it
        	if ((tData_11 != fpgaT_3004_5000_count)
        	|| (tData_12 != (fpgaT_3004_5000_count ^ 0xFFFF))
        	|| (tData_21 != (0 - fpgaT_3004_5000_count))
        	|| (tData_22 != ((0 - fpgaT_3004_5000_count) ^ 0xFFFF))
        	|| (fpgaT_3004_5000_immediate_err_test != 0)) {
        		// I built in a mechanism to test the error message C3005:5002
        		if (fpgaT_3004_5000_immediate_err_test != 0){
            		ptr = strU_strcpy(msgOut," Tst B1, W1 0x");
        		} else {
            		fpgaT_3004_5000_errors++;
        		    ptr = strU_strcpy(msgOut," Err B1, W1 0x");
        		}
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,fpgaT_3004_5000_count);
        		ptr = strU_strcpy(ptr,", R1 0x");
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,tData_11);
        		ptr = strU_strcpy(ptr,", W2 0x");
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,(fpgaT_3004_5000_count ^ 0xFFFF));
        		ptr = strU_strcpy(ptr,", R2 0x");
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,tData_12);
        		ptr = strU_strcpy(ptr,"\n\r");
        		/* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

        		if (fpgaT_3004_5000_immediate_err_test != 0){
        			fpgaT_3004_5000_immediate_err_test = 0;
            		ptr = strU_strcpy(msgOut," Tst B2, W1 0x");
        		} else {
        		    ptr = strU_strcpy(msgOut," Err B2, W1 0x");
        		}
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,(0 - fpgaT_3004_5000_count));
        		ptr = strU_strcpy(ptr,", R1 0x");
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,tData_21);
        		ptr = strU_strcpy(ptr,", W2 0x");
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,((0 - fpgaT_3004_5000_count) ^ 0xFFFF));
        		ptr = strU_strcpy(ptr,", R2 0x");
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,tData_22);
        		ptr = strU_strcpy(ptr,"\n\r");
        		/* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

            	break;
        	}
        	// report every 0x1000 -- count of tested so far.
        	if ((fpgaT_3004_5000_count & 0x0FFF) == 0){
        		// Transmit results out rs232
        		ptr = strU_strcpy(msgOut,"RW 1&2 stored val, count 0x");
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,fpgaT_3004_5000_count);
        		ptr = strU_strcpy(ptr,"\n\r");
        		/* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

        		if (fpgaT_3004_5000_count == 0){
    				// finished a complete pass thru all 65536 data bus values
        			fpgaT_3004_5000_passes++;
    			}
        	}

			// Print a report of status: how many passes through all 65536 data bus values,
			// how many errors, and if this is an "immediate" request then where are we within the 65536.
    		if ((fpgaT_3004_5000_count == 0) || (fpgaT_3004_5000_immediate_report != 0)) {
        		ptr = strU_strcpy(msgOut,"RW 1&2 ");
    			if (fpgaT_3004_5000_immediate_report != 0) {
    				fpgaT_3004_5000_immediate_report = 0;
    				ptr = strU_strcpy(ptr,"Count: 0x");
    				ptr = hexUtil_binTo4HexAsciiChars(ptr,fpgaT_3004_5000_count);
            		ptr = strU_strcpy(ptr,", ");
    			}
        		ptr = strU_strcpy(ptr,"Passes: 0x");
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,fpgaT_3004_5000_passes);
        		ptr = strU_strcpy(ptr,", Errors: 0x");
        		ptr = hexUtil_binTo4HexAsciiChars(ptr,fpgaT_3004_5000_errors);
        		ptr = strU_strcpy(ptr,"\n\r");
        		/* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    		}
    	}
    	break;

    default:
    	break;
    }

//	tempData16 = dataRead; // this statement does NOTHING except
	                       // to prevent getting a warning on
	                       // dataRead set but never used -- DH

}

void fpgaT_test3005(Uint16 dataWord){
	// These tests are invoked from comint for command C3005:nnnn
	// These tests execute 1 time, as opposed to the ones above
	// that repeat for each T0 Timer interrupt
	Uint16 *extData_B1_01;
	Uint16 *extData_B1_02;
	Uint16 *extData_B2_01;
	Uint16 *extData_B2_02;
	Uint16 tempData16;
	// Uint32 tempData32;
	// bool success;
	char msgOut[64];
    char *ptr;
	Uint16 tData_11;
	Uint16 tData_12;
	Uint16 tData_21;


    // - - - - - - - - - - - -
    //   CS_FPGA_1 -- 1st data bus into FPGA
    // - - - - - - - - - - - -

	switch(dataWord){
    case 0x0100:
    case 0x0101:
    case 0x0102:
    case 0x0103:
    	//  WRITE_LED_DIRECTLY = 8'h0005; stores a 4-bit value from DSP to display in LEDs
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,5);   //
       	*extData_B1_01 = dataWord; // write config value: LED_DIRECTLY_FROM_DSP
       	// 2nd write to other address to turn off FPGA chip select
    	extData_B1_02 = (Uint16 *)0x0800FB; // CPLD_TP_0 (eg NOT ~CS_FPGA_1)
        *extData_B1_02 = dataWord ^ 0xFFFF; // write access out of target area, toggle all data lines
        break;

    case 0x1100:
    case 0x1101:
    case 0x1102:
    case 0x1103:
    	//  WRITE_LED_FUNCTION = 8'h0004;
        //       LED_DIRECTLY_FROM_DSP            = 2'b00;
        //       LED_FROM_COUNT_CLK               = 2'b01;
        //       LED_FROM_COUNT_WR_STORERD_VAL_1  = 2'b10;
    	//       note 3 is actually a bogus value, lets see what FPGA does
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,4);   //
       	*extData_B1_01 = dataWord; // FPGA ignores everything other than 2 ls bits
       	// 2nd write to other address to turn off FPGA chip select
    	extData_B1_02 = (Uint16 *)0x0800FB; // CPLD_TP_0 (eg NOT ~CS_FPGA_1)
        *extData_B1_02 = dataWord ^ 0xFFFF; // write access out of target area, toggle all data lines
        break;

    case 0x1105:
    	// Turn all 4 led's off FPGA_1
    	//  WRITE_LED_FUNCTION = 8'h0004;
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,4);
       	*extData_B1_01 = 0; // LED_DIRECTLY_FROM_DSP = 2'b00;
    	//  WRITE_LED_DIRECTLY = 8'h0005; stores a 4-bit value from DSP to display in LEDs
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,5);
       	*extData_B1_01 = 0; // value to display in LEDs
        fpgaT_ledStatus = FPGAT_LED_DISP_OFF;
        break;

    case 0x1106:
    	// Set LED's to display from count clock FPGA_2
    	//  WRITE_LED_FUNCTION = 8'h0004;
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,4);
       	*extData_B1_01 = 1; // LED_FROM_COUNT_CLK = 2'b01;
        fpgaT_ledStatus = FPGAT_LED_DISP_FROM_COUNT_CLOCK;
        break;

    case 0x1107:
    	// Turn all 4 led's off FPGA_2
    	//  WRITE_LED_FUNCTION = 8'h0004;
    	extData_B1_01 = CPLD_F2_XA(FPGA2_WRITE_LED_FUNCTION);
       	*extData_B1_01 = 0; // LED_DIRECTLY_FROM_DSP = 2'b00;
    	//  WRITE_LED_DIRECTLY = 8'h0005; stores a 4-bit value from DSP to display in LEDs
    	extData_B1_01 = CPLD_F2_XA(FPGA2_WRITE_LED_DIRECTLY);
       	*extData_B1_01 = 0; // value to display in LEDs
        fpgaT_ledStatus = FPGAT_LED_DISP_OFF;
        break;

    case 0x1108:
    	// Set LED's to display from count clock FPGA_2
    	//  WRITE_LED_FUNCTION = 8'h0004;
    	extData_B1_01 = CPLD_F2_XA(FPGA2_WRITE_LED_FUNCTION);
       	*extData_B1_01 = 1; // LED_FROM_COUNT_CLK = 2'b01;
        fpgaT_ledStatus = FPGAT_LED_DISP_FROM_COUNT_CLOCK;
        break;

    case 0x2100:
    case 0x2101:
    case 0x2102:
    case 0x2103:
    case 0x2104:
    	// Simple FPGA read operations
        //  0x2100:        READ_0x0000  = 8'h000B;
        //  0x2101:        READ_0xFFFF  = 8'h000C;
        //  0x2102:        READ_0xA5A5  = 8'h000D;
        //  0x2103:        READ_0x5A5A  = 8'h000E;
        //  0x2104:        READ_BUS_ID  = 8'h000F;


    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR,((dataWord & 0x000F) + 0x0B));
    	tempData16 = *extData_B1_01;
       	// 2nd write to other address to turn off FPGA chip select
    	extData_B1_02 = (Uint16 *)0x0800FB; // CPLD_TP_0 (eg NOT ~CS_FPGA_1)
        *extData_B1_02 = tempData16 ^ 0xFFFF; // write access out of target area, toggle all data lines

        // Transmit results out rs232
        ptr = strU_strcpy(msgOut,"FPGA Read 1 0x");
        ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
        ptr = strU_strcpy(ptr,"\n\r");
        /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

        break;

    case 0x3100:
    	// Write data (3203:dddd) to stored value 1
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR, 1);
    	*extData_B1_01 = fpgaT_testData3003;
        break;

    case 0x3101:
    	// Write data (3203:dddd) to stored value 2
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR, 2);
    	*extData_B1_01 = fpgaT_testData3003;
        break;

    case 0x3102:
    	// Read stored value 1
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR, 5);
    	tempData16 = *extData_B1_01;

        // Transmit results out rs232
        ptr = strU_strcpy(msgOut,"Read 1 Stored Val 1 0x");
        ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
        ptr = strU_strcpy(ptr,"\n\r");
        /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	break;

    case 0x3103:
    	// Read stored value 2
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR, 6);
    	tempData16 = *extData_B1_01;

        // Transmit results out rs232
        ptr = strU_strcpy(msgOut,"Read 1 Stored Val 2 0x");
        ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
        ptr = strU_strcpy(ptr,"\n\r");
        /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	break;

    case 0x3104:
    	// Read stored value 1 xor 2
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR, 7);
    	tempData16 = *extData_B1_01;

        // Transmit results out rs232
        ptr = strU_strcpy(msgOut,"Read 1 Stored Val 1 ^ 2 0x");
        ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
        ptr = strU_strcpy(ptr,"\n\r");
        /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
        break;

    case 0x3110:
    	// Is FPGA talking ?
    	// Write and read to stored value locations in FPGA, then report to RS232

    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR, 1); // write stored val 1
    	extData_B1_02 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR, 5); // reading stored val 1

    	// write 2 data values
    	*(extData_B1_01++) = 0xAA55; // write to STORED_VAL_1 Bus 1
       	*(extData_B1_01) = 0xA5A5; // write STORED_VAL_2 Bus 1

        // Read back 3 data values
       	tData_11 = *(extData_B1_02++);  // STORED_VAL_1
       	tData_12 = *(extData_B1_02++);  // STORED_VAL_2
       	tData_21 = *(extData_B1_02);    // STORED_VAL_1 xor STORED_VAL_2

       	if ((tData_11 == 0xAA55) && (tData_12 == 0xA5A5) && (tData_21 == 0x0FF0)){
            ptr = strU_strcpy(msgOut,"Talking to FPGA OK\n\r");
       	} else {
            ptr = strU_strcpy(msgOut,"FPGA ** NOT ** Talking\n\r");
       	}
        r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
        break;

    case 0x4100:
    	// Write Reset Count Clk
    	extData_B1_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_1_BASE_ADDR, 3);
    	*extData_B1_01 = 0;
        break;

    case 0x5001: // Request an immediate report on progress of RW Stored Val Test 3004:5000
    	//
    	fpgaT_3004_5000_immediate_report = 1;
        break;

    case 0x5002: // Request an immediate error Test of RW Stored Val Test 3004:5000
    	//
    	fpgaT_3004_5000_immediate_err_test = 1;
        break;

    // - - - - - - - - - - - -
    //   CS_FPGA_2 -- 2nd data bus into FPGA1 on TB3IOMA
    // - - - - - - - - - - - -

    case 0x2200:
    case 0x2201:
    case 0x2202:
    case 0x2203:
    case 0x2204:
    	// Simple FPGA read operations
        //  0x2200:        READ_0x0000  = 8'h000B;
        //  0x2201:        READ_0xFFFF  = 8'h000C;
        //  0x2202:        READ_0xA5A5  = 8'h000D;
        //  0x2203:        READ_0x5A5A  = 8'h000E;
        //  0x2204:        READ_BUS_ID  = 8'h000F;

    	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR,((dataWord & 0x000F) + 0x8B));
    	tempData16 = *extData_B2_01;
       	// 2nd write to other address to turn off FPGA chip select
    	extData_B2_02 = (Uint16 *)0x0800FB; // CPLD_TP_0 (eg NOT ~CS_FPGA_2)
        *extData_B2_02 = tempData16 ^ 0xFFFF; // write access out of target area, toggle all data lines

        // Transmit results out rs232
        ptr = strU_strcpy(msgOut,"FPGA Read 2 0x");
        ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
        ptr = strU_strcpy(ptr,"\n\r");
        /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

        break;

    case 0x3200:
    	// Write data (3203:dddd) to stored value 1
    	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x81);
    	*extData_B2_01 = fpgaT_testData3003;
        break;

    case 0x3201:
    	// Write data (3203:dddd) to stored value 2
    	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x82);
    	*extData_B2_01 = fpgaT_testData3003;
        break;

    case 0x3202:
    	// Read stored value 1
    	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x85);
    	tempData16 = *extData_B2_01;

        // Transmit results out rs232
        ptr = strU_strcpy(msgOut,"Read 2 Stored Val 1 0x");
        ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
        ptr = strU_strcpy(ptr,"\n\r");
        /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	break;

    case 0x3203:
    	// Read stored value 2
    	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x86);
    	tempData16 = *extData_B2_01;

        // Transmit results out rs232
        ptr = strU_strcpy(msgOut,"Read 2 Stored Val 2 0x");
        ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
        ptr = strU_strcpy(ptr,"\n\r");
        /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
    	break;

    case 0x3204:
    	// Read stored value 1 xor 2
    	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x87);
    	tempData16 = *extData_B2_01;

        // Transmit results out rs232
        ptr = strU_strcpy(msgOut,"Read 2 Stored Val 1 ^ 2 0x");
        ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
        ptr = strU_strcpy(ptr,"\n\r");
        /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
        break;

    case 0x4200:
    	// Write Reset Count Clk
    	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x83);
    	*extData_B2_01 = 0;
        break;

        // - - - - - - - - - - - -
        //   CS_FPGA_2 -- FPGA2 on TB3IOMB
        // - - - - - - - - - - - -

        case 0x2300:
        case 0x2301:
        case 0x2302:
        case 0x2303:
        case 0x2304:
        	// Simple FPGA read operations
            //  0x2300:        READ_0x0000  = 8'h000B;
            //  0x2301:        READ_0xFFFF  = 8'h000C;
            //  0x2302:        READ_0xA5A5  = 8'h000D;
            //  0x2303:        READ_0x5A5A  = 8'h000E;
            //  0x2304:        READ_BUS_ID  = 8'h000F;

        	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR,((dataWord & 0x000F) + 0x0B));
        	tempData16 = *extData_B2_01;
           	// 2nd write to other address to turn off FPGA chip select
        	extData_B2_02 = (Uint16 *)0x0800FB; // CPLD_TP_0 (eg NOT ~CS_FPGA_2)
            *extData_B2_02 = tempData16 ^ 0xFFFF; // write access out of target area, toggle all data lines

            // Transmit results out rs232
            ptr = strU_strcpy(msgOut,"FPGA2 Read 0x");
            ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
            ptr = strU_strcpy(ptr,"\n\r");
            /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));

            break;

        case 0x3300:
         	// Write data (3003:dddd) to stored value 1
        	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x01);
        	*extData_B2_01 = fpgaT_testData3003;
            break;

        case 0x3301:
        	// Write data (3003:dddd) to stored value 2
        	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x02);
        	*extData_B2_01 = fpgaT_testData3003;
            break;

        case 0x3302:
        	// Read stored value 1
        	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x05);
        	tempData16 = *extData_B2_01;

            // Transmit results out rs232
            ptr = strU_strcpy(msgOut,"Read F2 Stored Val 1 0x");
            ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
            ptr = strU_strcpy(ptr,"\n\r");
            /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
        	break;

        case 0x3303:
        	// Read stored value 2
        	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x06);
        	tempData16 = *extData_B2_01;

            // Transmit results out rs232
            ptr = strU_strcpy(msgOut,"Read F2 Stored Val 2 0x");
            ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
            ptr = strU_strcpy(ptr,"\n\r");
            /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
        	break;

        case 0x3304:
        	// Read stored value 1 xor 2
        	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x07);
        	tempData16 = *extData_B2_01;

            // Transmit results out rs232
            ptr = strU_strcpy(msgOut,"Read F2 Stored Val 1 ^ 2 0x");
            ptr = hexUtil_binTo4HexAsciiChars(ptr,tempData16);
            ptr = strU_strcpy(ptr,"\n\r");
            /* success = */ r232Out_outChars(msgOut, (Uint16)(ptr - msgOut));
            break;

        case 0x4300:
        	// Write Reset Count Clk
        	extData_B2_01 = CPLD_FPGA_XINTF_ADDR(TBIOM_FPGA_2_BASE_ADDR, 0x03);
        	*extData_B2_01 = 0;
            break;

    default:
    	break;
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   C A N   C O M M A N D S
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum CANOPEN_STATUS fpgaT_send32Clk(const struct CAN_COMMAND *can_command,Uint16 *data) {
// We received a request from the PC to send the value of an Fpga 32-bit internal clock counter
// This is probably part of a test to verify we have good DSP/FPGA communications
// across the bi-directional parallel bus.
// *data is MboxA of transmit Message
// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 subindex;
	Uint16 *source;

	// Subindex is MS 8 bits of MboxB
	subindex = ((*(data+1)) >> 8) & 0x00FF;

	if (subindex == 0x1E) {
		//Fpga1
        source = CPLD_F1_XA(FPGA1_READ_COUNT_CLK_LOW);
	} else if (subindex == 0x1F) {
		//Fpga2
        source = CPLD_F2_XA(FPGA2_READ_COUNT_CLK_LOW);
	} else if (subindex == 0x20) {
		//Fpga3
        source = CPLD_F3_XA(FPGA3_READ_COUNT_CLK_LOW);
	} else {
		return CANOPEN_SUBINDEX_ERR;
	}

	// Convention on reading 32-bit values from the FPGA:
	// Read LS 16 bits first.
	// FPGA caches corresponding MS 16-Bits for subsequent read.
	*(data+2) = *(source++); // LS 16 bits
	*(data+3) = *source;     // MS 16 Bits
	return CANOPEN_NO_ERR;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//   C A N - B a s e d   F P G A   S t o r e d   V a l u e   T e s t
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
// fpgaT non-public variables -- used in CAN-based test to write / read stored values in FPGA
//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
Uint16 fpgaT_Fpga1_sv1_write;
Uint16 fpgaT_Fpga1_sv1_read;
Uint16 fpgaT_Fpga1_sv2_write;
Uint16 fpgaT_Fpga1_sv2_read;
Uint16 fpgaT_Fpga2_sv1_write;
Uint16 fpgaT_Fpga2_sv1_read;
Uint16 fpgaT_Fpga2_sv2_write;
Uint16 fpgaT_Fpga2_sv2_read;
Uint16 fpgaT_Fpga3_sv1_write;
Uint16 fpgaT_Fpga3_sv1_read;
Uint16 fpgaT_Fpga3_sv2_write;
Uint16 fpgaT_Fpga3_sv2_read;

Uint16 fpgaT_sv_test_which_Fpgas;
Uint16 fpgaT_sv_test_Per_Loop;
enum SVTEST_CONTROL fpgaT_sv_test_Control;
Uint16 fpgaT_sv_test_Error;
Uint32 fpgaT_sv_test_Count_Tests;
Uint16 fpgaT_sv_test_Throw_Error;

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

enum CANOPEN_STATUS fpgaT_sv_test_ctrl(const struct CAN_COMMAND *can_command,Uint16 *data) {
	// We received 16-bit command from PC relating to FPGA stored-value test.
	// In this test each FPGA has two 16-bit registers into which we can store values
	// and read them back out.  This is done to verify the low level functioning of the
	// bi-directional data bus connecting the DSP and the FPGAs.  In other words
	// we are verifying DSP's ability to talk with the FPGAs.
	// TBD -- this is a dummy proc stub, functioning is to be determined later.
	// *data is MboxA of received Message
	// *can_command is Table entry (struct) of parameters for CANOpen Index.Subindex

	Uint16 *dest;

	dest = (Uint16*)can_command->datapointer; // fpgaT_sv_test_Control
	*dest = *(data+2); // MboxC
	// MboxD ignored

	if (fpgaT_sv_test_Control == SVTEST_CTRL_STOP){
    	return CANOPEN_NO_ERR;
	}

    fpgaT_sv_test_Error = 0;

	// repetative task under task manager control
	taskMgr_setTaskRoundRobin(TASKNUM_fpgaT_sv_test_Task, 0);

	return CANOPEN_NO_ERR;
}


void fpgaT_sv_test_one_rw(void){
	// Write values to each of the 2 stored value registers in each FPGA.

    // First we write them
	if (fpgaT_sv_test_which_Fpgas & SVTEST_FPGA1){
		//FPGA1
		if ((fpgaT_sv_test_Control == SVTEST_CTRL_RUN)
		|| (fpgaT_sv_test_Control == SVTEST_CTRL_STEP)){
			// incrememnt values in RUN mode or STEP mode, not in TEST or STOP
			fpgaT_Fpga1_sv1_write++;
			fpgaT_Fpga1_sv2_write++;
		}
		*CPLD_F1_XA(FPGA1_WRITE_STORED_VAL_1) = fpgaT_Fpga1_sv1_write;
		*CPLD_F1_XA(FPGA1_WRITE_STORED_VAL_2) = fpgaT_Fpga1_sv2_write;
	}

	if (fpgaT_sv_test_which_Fpgas & SVTEST_FPGA2){
		//FPGA2
		if ((fpgaT_sv_test_Control == SVTEST_CTRL_RUN)
		|| (fpgaT_sv_test_Control == SVTEST_CTRL_STEP)){
			// incrememnt values in RUN mode or STEP mode, not in TEST or STOP
			fpgaT_Fpga2_sv1_write++;
			fpgaT_Fpga2_sv2_write++;
		}
		*CPLD_F2_XA(FPGA2_WRITE_STORED_VAL_1) = fpgaT_Fpga2_sv1_write;
		*CPLD_F2_XA(FPGA2_WRITE_STORED_VAL_2) = fpgaT_Fpga2_sv2_write;
	}

	if (fpgaT_sv_test_which_Fpgas & SVTEST_FPGA3){
		//FPGA3
		if ((fpgaT_sv_test_Control == SVTEST_CTRL_RUN)
		|| (fpgaT_sv_test_Control == SVTEST_CTRL_STEP)){
			// incrememnt values in RUN mode or STEP mode, not in TEST or STOP
			fpgaT_Fpga3_sv1_write++;
			fpgaT_Fpga3_sv2_write++;
		}
		*CPLD_F3_XA(FPGA3_WRITE_STORED_VAL_1) = fpgaT_Fpga3_sv1_write;
		*CPLD_F3_XA(FPGA3_WRITE_STORED_VAL_2) = fpgaT_Fpga3_sv2_write;
	}

	// Now read them back
	if (fpgaT_sv_test_which_Fpgas & SVTEST_FPGA1){
		//FPGA1
		fpgaT_Fpga1_sv1_read = *CPLD_F1_XA(FPGA1_READ_STORED_VAL_1);
		fpgaT_Fpga1_sv2_read = *CPLD_F1_XA(FPGA1_READ_STORED_VAL_2);
        // If we have an external request to force an error to test error handling
		if (fpgaT_sv_test_Throw_Error != 0){
			fpgaT_sv_test_Throw_Error--;
			if (fpgaT_sv_test_Throw_Error == 0){
				fpgaT_Fpga1_sv1_read++;
				fpgaT_Fpga1_sv2_read--;
			}
		}
		// Check to verify we read back same value we wrote
		if ((fpgaT_Fpga1_sv1_read != fpgaT_Fpga1_sv1_write)
		|| (fpgaT_Fpga1_sv2_read != fpgaT_Fpga1_sv2_write)){
		    fpgaT_sv_test_Error |= SVTEST_FPGA1;
		}
	}

	if (fpgaT_sv_test_which_Fpgas & SVTEST_FPGA2){
		//FPGA2
		fpgaT_Fpga2_sv1_read = *CPLD_F2_XA(FPGA2_READ_STORED_VAL_1);
		fpgaT_Fpga2_sv2_read = *CPLD_F2_XA(FPGA2_READ_STORED_VAL_2);
        // If we have an external request to force an error to test error handling
		if (fpgaT_sv_test_Throw_Error != 0){
			fpgaT_sv_test_Throw_Error--;
			if (fpgaT_sv_test_Throw_Error == 0){
				fpgaT_Fpga2_sv1_read++;
				fpgaT_Fpga2_sv2_read--;
			}
		}
		// Check to verify we read back same value we wrote
		if ((fpgaT_Fpga2_sv1_read != fpgaT_Fpga2_sv1_write)
		|| (fpgaT_Fpga2_sv2_read != fpgaT_Fpga2_sv2_write)){
		    fpgaT_sv_test_Error |= SVTEST_FPGA2;
		}
	}

	if (fpgaT_sv_test_which_Fpgas & SVTEST_FPGA3){
		//FPGA3
		fpgaT_Fpga3_sv1_read = *CPLD_F3_XA(FPGA3_READ_STORED_VAL_1);
		fpgaT_Fpga3_sv2_read = *CPLD_F3_XA(FPGA3_READ_STORED_VAL_2);
        // If we have an external request to force an error to test error handling
		if (fpgaT_sv_test_Throw_Error != 0){
			fpgaT_sv_test_Throw_Error--;
			if (fpgaT_sv_test_Throw_Error == 0){
				fpgaT_Fpga3_sv1_read++;
				fpgaT_Fpga3_sv2_read--;
			}
		}
		// Check to verify we read back same value we wrote
		if ((fpgaT_Fpga3_sv1_read != fpgaT_Fpga3_sv1_write)
		|| (fpgaT_Fpga3_sv2_read != fpgaT_Fpga3_sv2_write)){
		    fpgaT_sv_test_Error |= SVTEST_FPGA3;
		}
	}
}

void fpgaT_sv_test_Task(void){
	// Background task to serve the Fpgs Stored Value test
	Uint16 i;

	for (i = fpgaT_sv_test_Per_Loop;i > 0;i--) {
	   if (fpgaT_sv_test_Error == 0) {
	       fpgaT_sv_test_one_rw(); // for starters, call this to write and re-read stored values
	       fpgaT_sv_test_Count_Tests++;
	   } else {
		   fpgaT_sv_test_Control = SVTEST_CTRL_STOP;
	   }
	}

	if (fpgaT_sv_test_Control == SVTEST_CTRL_STOP){
		return; // don't re-launch Task
	}
	if ((fpgaT_sv_test_Control == SVTEST_CTRL_STEP)
    || (fpgaT_sv_test_Control == SVTEST_CTRL_TEST)) {
		fpgaT_sv_test_Control = SVTEST_CTRL_STOP; // Stop after one step
		return; // single step, don't re-launch Task
	}

    if (fpgaT_sv_test_Control == SVTEST_CTRL_RUN){
		taskMgr_setTaskRoundRobin(TASKNUM_fpgaT_sv_test_Task, 0); // re-launch this task
	}
}

