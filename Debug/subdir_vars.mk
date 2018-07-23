################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_headers/cmd/DSP281x_Headers_nonBIOS.cmd \
../F2812.cmd 

C_UPPER_SRCS += \
../ADC.C \
../AnlgIn.C \
../CanComm.C \
../CanFile.C \
../CanOpen.C \
../FlashRW.C \
../I2CEE.C \
../LED.C \
../LimitChk.C \
../Log.C \
../Resolver.C \
../SCI2.C \
../SPI.C 

ASM_SRCS += \
../BootLdEntry.asm \
C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_common/source/DSP281x_CodeStartBranch.asm \
../DSP281x_usDelay.asm \
../WorkAround.asm \
../access32.asm 

C_SRCS += \
../Comint.c \
C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_common/source/DSP281x_DefaultIsr.c \
../DSP281x_ECan.c \
C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_headers/source/DSP281x_GlobalVariableDefs.c \
C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_common/source/DSP281x_PieCtrl.c \
C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_common/source/DSP281x_PieVect.c \
C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_common/source/DSP281x_SysCtrl.c \
../DigIO.c \
../DigIO2.c \
../EVTimer4.c \
../ExtRam.c \
../F1Int.c \
../F2Int.c \
../FpgaTest.c \
../GpioUtil.c \
../HexUtil.c \
../Main.c \
../McsParse.c \
../RS232.c \
../Rs232Out.c \
../SSEnc.c \
../StrUtil.c \
../TaskMgr.c \
../TimeStamp.c \
../Timer0.c \
../Xintf.c 

OBJS += \
./ADC.obj \
./AnlgIn.obj \
./BootLdEntry.obj \
./CanComm.obj \
./CanFile.obj \
./CanOpen.obj \
./Comint.obj \
./DSP281x_CodeStartBranch.obj \
./DSP281x_DefaultIsr.obj \
./DSP281x_ECan.obj \
./DSP281x_GlobalVariableDefs.obj \
./DSP281x_PieCtrl.obj \
./DSP281x_PieVect.obj \
./DSP281x_SysCtrl.obj \
./DSP281x_usDelay.obj \
./DigIO.obj \
./DigIO2.obj \
./EVTimer4.obj \
./ExtRam.obj \
./F1Int.obj \
./F2Int.obj \
./FlashRW.obj \
./FpgaTest.obj \
./GpioUtil.obj \
./HexUtil.obj \
./I2CEE.obj \
./LED.obj \
./LimitChk.obj \
./Log.obj \
./Main.obj \
./McsParse.obj \
./RS232.obj \
./Resolver.obj \
./Rs232Out.obj \
./SCI2.obj \
./SPI.obj \
./SSEnc.obj \
./StrUtil.obj \
./TaskMgr.obj \
./TimeStamp.obj \
./Timer0.obj \
./WorkAround.obj \
./Xintf.obj \
./access32.obj 

ASM_DEPS += \
./BootLdEntry.pp \
./DSP281x_CodeStartBranch.pp \
./DSP281x_usDelay.pp \
./WorkAround.pp \
./access32.pp 

C_DEPS += \
./Comint.pp \
./DSP281x_DefaultIsr.pp \
./DSP281x_ECan.pp \
./DSP281x_GlobalVariableDefs.pp \
./DSP281x_PieCtrl.pp \
./DSP281x_PieVect.pp \
./DSP281x_SysCtrl.pp \
./DigIO.pp \
./DigIO2.pp \
./EVTimer4.pp \
./ExtRam.pp \
./F1Int.pp \
./F2Int.pp \
./FpgaTest.pp \
./GpioUtil.pp \
./HexUtil.pp \
./Main.pp \
./McsParse.pp \
./RS232.pp \
./Rs232Out.pp \
./SSEnc.pp \
./StrUtil.pp \
./TaskMgr.pp \
./TimeStamp.pp \
./Timer0.pp \
./Xintf.pp 

C_UPPER_DEPS += \
./ADC.pp \
./AnlgIn.pp \
./CanComm.pp \
./CanFile.pp \
./CanOpen.pp \
./FlashRW.pp \
./I2CEE.pp \
./LED.pp \
./LimitChk.pp \
./Log.pp \
./Resolver.pp \
./SCI2.pp \
./SPI.pp 

C_DEPS__QUOTED += \
"Comint.pp" \
"DSP281x_DefaultIsr.pp" \
"DSP281x_ECan.pp" \
"DSP281x_GlobalVariableDefs.pp" \
"DSP281x_PieCtrl.pp" \
"DSP281x_PieVect.pp" \
"DSP281x_SysCtrl.pp" \
"DigIO.pp" \
"DigIO2.pp" \
"EVTimer4.pp" \
"ExtRam.pp" \
"F1Int.pp" \
"F2Int.pp" \
"FpgaTest.pp" \
"GpioUtil.pp" \
"HexUtil.pp" \
"Main.pp" \
"McsParse.pp" \
"RS232.pp" \
"Rs232Out.pp" \
"SSEnc.pp" \
"StrUtil.pp" \
"TaskMgr.pp" \
"TimeStamp.pp" \
"Timer0.pp" \
"Xintf.pp" 

C_UPPER_DEPS__QUOTED += \
"ADC.pp" \
"AnlgIn.pp" \
"CanComm.pp" \
"CanFile.pp" \
"CanOpen.pp" \
"FlashRW.pp" \
"I2CEE.pp" \
"LED.pp" \
"LimitChk.pp" \
"Log.pp" \
"Resolver.pp" \
"SCI2.pp" \
"SPI.pp" 

OBJS__QUOTED += \
"ADC.obj" \
"AnlgIn.obj" \
"BootLdEntry.obj" \
"CanComm.obj" \
"CanFile.obj" \
"CanOpen.obj" \
"Comint.obj" \
"DSP281x_CodeStartBranch.obj" \
"DSP281x_DefaultIsr.obj" \
"DSP281x_ECan.obj" \
"DSP281x_GlobalVariableDefs.obj" \
"DSP281x_PieCtrl.obj" \
"DSP281x_PieVect.obj" \
"DSP281x_SysCtrl.obj" \
"DSP281x_usDelay.obj" \
"DigIO.obj" \
"DigIO2.obj" \
"EVTimer4.obj" \
"ExtRam.obj" \
"F1Int.obj" \
"F2Int.obj" \
"FlashRW.obj" \
"FpgaTest.obj" \
"GpioUtil.obj" \
"HexUtil.obj" \
"I2CEE.obj" \
"LED.obj" \
"LimitChk.obj" \
"Log.obj" \
"Main.obj" \
"McsParse.obj" \
"RS232.obj" \
"Resolver.obj" \
"Rs232Out.obj" \
"SCI2.obj" \
"SPI.obj" \
"SSEnc.obj" \
"StrUtil.obj" \
"TaskMgr.obj" \
"TimeStamp.obj" \
"Timer0.obj" \
"WorkAround.obj" \
"Xintf.obj" \
"access32.obj" 

ASM_DEPS__QUOTED += \
"BootLdEntry.pp" \
"DSP281x_CodeStartBranch.pp" \
"DSP281x_usDelay.pp" \
"WorkAround.pp" \
"access32.pp" 

C_UPPER_SRCS__QUOTED += \
"../ADC.C" \
"../AnlgIn.C" \
"../CanComm.C" \
"../CanFile.C" \
"../CanOpen.C" \
"../FlashRW.C" \
"../I2CEE.C" \
"../LED.C" \
"../LimitChk.C" \
"../Log.C" \
"../Resolver.C" \
"../SCI2.C" \
"../SPI.C" 

ASM_SRCS__QUOTED += \
"../BootLdEntry.asm" \
"C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_common/source/DSP281x_CodeStartBranch.asm" \
"../DSP281x_usDelay.asm" \
"../WorkAround.asm" \
"../access32.asm" 

C_SRCS__QUOTED += \
"../Comint.c" \
"C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_common/source/DSP281x_DefaultIsr.c" \
"../DSP281x_ECan.c" \
"C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_headers/source/DSP281x_GlobalVariableDefs.c" \
"C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_common/source/DSP281x_PieCtrl.c" \
"C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_common/source/DSP281x_PieVect.c" \
"C:/TI_SPRCO97_C281x_Header_n_Peripherals_Ex/tidcs/c28/DSP281x/v120/DSP281x_common/source/DSP281x_SysCtrl.c" \
"../DigIO.c" \
"../DigIO2.c" \
"../EVTimer4.c" \
"../ExtRam.c" \
"../F1Int.c" \
"../F2Int.c" \
"../FpgaTest.c" \
"../GpioUtil.c" \
"../HexUtil.c" \
"../Main.c" \
"../McsParse.c" \
"../RS232.c" \
"../Rs232Out.c" \
"../SSEnc.c" \
"../StrUtil.c" \
"../TaskMgr.c" \
"../TimeStamp.c" \
"../Timer0.c" \
"../Xintf.c" 


