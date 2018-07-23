################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv5/tools/compiler/c2000_6.1.0

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./access32.obj" \
"./Xintf.obj" \
"./WorkAround.obj" \
"./Timer0.obj" \
"./TimeStamp.obj" \
"./TaskMgr.obj" \
"./StrUtil.obj" \
"./SSEnc.obj" \
"./SPI.obj" \
"./SCI2.obj" \
"./Rs232Out.obj" \
"./Resolver.obj" \
"./RS232.obj" \
"./McsParse.obj" \
"./Main.obj" \
"./Log.obj" \
"./LimitChk.obj" \
"./LED.obj" \
"./I2CEE.obj" \
"./HexUtil.obj" \
"./GpioUtil.obj" \
"./FpgaTest.obj" \
"./FlashRW.obj" \
"./F2Int.obj" \
"./F1Int.obj" \
"./ExtRam.obj" \
"./EVTimer4.obj" \
"./DigIO2.obj" \
"./DigIO.obj" \
"./DSP281x_usDelay.obj" \
"./DSP281x_SysCtrl.obj" \
"./DSP281x_PieVect.obj" \
"./DSP281x_PieCtrl.obj" \
"./DSP281x_GlobalVariableDefs.obj" \
"./DSP281x_ECan.obj" \
"./DSP281x_DefaultIsr.obj" \
"./DSP281x_CodeStartBranch.obj" \
"./Comint.obj" \
"./CanOpen.obj" \
"./CanFile.obj" \
"./CanComm.obj" \
"./BootLdEntry.obj" \
"./AnlgIn.obj" \
"./ADC.obj" \
-l"rts2800_ml.lib" \
"C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_headers/cmd/DSP281x_Headers_nonBIOS.cmd" \
"../F2812.cmd" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: TB3CM_DSP_Code.out

# Tool invocations
TB3CM_DSP_Code.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv5/tools/compiler/c2000_6.1.0/bin/cl2000" -v28 -ml -mt -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --display_error_number --issue_remarks --output_all_syms -z --stack_size=0x400 -m"TB3CM_DSP_Code.map" --warn_sections -i"C:/ti/ccsv5/tools/compiler/c2000_6.1.0/lib" -i"C:/ti/ccsv5/tools/compiler/c2000_6.1.0/include" -i"R:/ti_CCS5c_workspaces/I_001_F281xTB3CMB_014" -i"C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_headers/include" --reread_libs --display_error_number --diag_suppress=10270 --rom_model -o "TB3CM_DSP_Code.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C2000_EXECUTABLE_OUTPUTS__QUOTED) "TB3CM_DSP_Code.out"
	-$(RM) "Comint.pp" "DSP281x_DefaultIsr.pp" "DSP281x_ECan.pp" "DSP281x_GlobalVariableDefs.pp" "DSP281x_PieCtrl.pp" "DSP281x_PieVect.pp" "DSP281x_SysCtrl.pp" "DigIO.pp" "DigIO2.pp" "EVTimer4.pp" "ExtRam.pp" "F1Int.pp" "F2Int.pp" "FpgaTest.pp" "GpioUtil.pp" "HexUtil.pp" "Main.pp" "McsParse.pp" "RS232.pp" "Rs232Out.pp" "SSEnc.pp" "StrUtil.pp" "TaskMgr.pp" "TimeStamp.pp" "Timer0.pp" "Xintf.pp" 
	-$(RM) "ADC.pp" "AnlgIn.pp" "CanComm.pp" "CanFile.pp" "CanOpen.pp" "FlashRW.pp" "I2CEE.pp" "LED.pp" "LimitChk.pp" "Log.pp" "Resolver.pp" "SCI2.pp" "SPI.pp" 
	-$(RM) "ADC.obj" "AnlgIn.obj" "BootLdEntry.obj" "CanComm.obj" "CanFile.obj" "CanOpen.obj" "Comint.obj" "DSP281x_CodeStartBranch.obj" "DSP281x_DefaultIsr.obj" "DSP281x_ECan.obj" "DSP281x_GlobalVariableDefs.obj" "DSP281x_PieCtrl.obj" "DSP281x_PieVect.obj" "DSP281x_SysCtrl.obj" "DSP281x_usDelay.obj" "DigIO.obj" "DigIO2.obj" "EVTimer4.obj" "ExtRam.obj" "F1Int.obj" "F2Int.obj" "FlashRW.obj" "FpgaTest.obj" "GpioUtil.obj" "HexUtil.obj" "I2CEE.obj" "LED.obj" "LimitChk.obj" "Log.obj" "Main.obj" "McsParse.obj" "RS232.obj" "Resolver.obj" "Rs232Out.obj" "SCI2.obj" "SPI.obj" "SSEnc.obj" "StrUtil.obj" "TaskMgr.obj" "TimeStamp.obj" "Timer0.obj" "WorkAround.obj" "Xintf.obj" "access32.obj" 
	-$(RM) "BootLdEntry.pp" "DSP281x_CodeStartBranch.pp" "DSP281x_usDelay.pp" "WorkAround.pp" "access32.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
