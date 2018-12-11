################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../peripherals/DSP2833x_ADC_cal.asm \
../peripherals/DSP2833x_CSMPasswords.asm \
../peripherals/DSP2833x_CodeStartBranch.asm \
../peripherals/DSP2833x_DBGIER.asm \
../peripherals/DSP2833x_DisInt.asm \
../peripherals/DSP2833x_usDelay.asm 

C_SRCS += \
../peripherals/CircleBuffer.c \
../peripherals/DSP2833x_Adc.c \
../peripherals/DSP2833x_CpuTimers.c \
../peripherals/DSP2833x_DMA.c \
../peripherals/DSP2833x_DefaultIsr.c \
../peripherals/DSP2833x_ECan.c \
../peripherals/DSP2833x_ECap.c \
../peripherals/DSP2833x_EPwm.c \
../peripherals/DSP2833x_EQep.c \
../peripherals/DSP2833x_GlobalVariableDefs.c \
../peripherals/DSP2833x_Gpio.c \
../peripherals/DSP2833x_I2C.c \
../peripherals/DSP2833x_Mcbsp.c \
../peripherals/DSP2833x_MemCopy.c \
../peripherals/DSP2833x_PieCtrl.c \
../peripherals/DSP2833x_PieVect.c \
../peripherals/DSP2833x_Sci.c \
../peripherals/DSP2833x_Spi.c \
../peripherals/DSP2833x_SysCtrl.c \
../peripherals/DSP2833x_Xintf.c \
../peripherals/swfifo.c 

C_DEPS += \
./peripherals/CircleBuffer.d \
./peripherals/DSP2833x_Adc.d \
./peripherals/DSP2833x_CpuTimers.d \
./peripherals/DSP2833x_DMA.d \
./peripherals/DSP2833x_DefaultIsr.d \
./peripherals/DSP2833x_ECan.d \
./peripherals/DSP2833x_ECap.d \
./peripherals/DSP2833x_EPwm.d \
./peripherals/DSP2833x_EQep.d \
./peripherals/DSP2833x_GlobalVariableDefs.d \
./peripherals/DSP2833x_Gpio.d \
./peripherals/DSP2833x_I2C.d \
./peripherals/DSP2833x_Mcbsp.d \
./peripherals/DSP2833x_MemCopy.d \
./peripherals/DSP2833x_PieCtrl.d \
./peripherals/DSP2833x_PieVect.d \
./peripherals/DSP2833x_Sci.d \
./peripherals/DSP2833x_Spi.d \
./peripherals/DSP2833x_SysCtrl.d \
./peripherals/DSP2833x_Xintf.d \
./peripherals/swfifo.d 

OBJS += \
./peripherals/CircleBuffer.obj \
./peripherals/DSP2833x_ADC_cal.obj \
./peripherals/DSP2833x_Adc.obj \
./peripherals/DSP2833x_CSMPasswords.obj \
./peripherals/DSP2833x_CodeStartBranch.obj \
./peripherals/DSP2833x_CpuTimers.obj \
./peripherals/DSP2833x_DBGIER.obj \
./peripherals/DSP2833x_DMA.obj \
./peripherals/DSP2833x_DefaultIsr.obj \
./peripherals/DSP2833x_DisInt.obj \
./peripherals/DSP2833x_ECan.obj \
./peripherals/DSP2833x_ECap.obj \
./peripherals/DSP2833x_EPwm.obj \
./peripherals/DSP2833x_EQep.obj \
./peripherals/DSP2833x_GlobalVariableDefs.obj \
./peripherals/DSP2833x_Gpio.obj \
./peripherals/DSP2833x_I2C.obj \
./peripherals/DSP2833x_Mcbsp.obj \
./peripherals/DSP2833x_MemCopy.obj \
./peripherals/DSP2833x_PieCtrl.obj \
./peripherals/DSP2833x_PieVect.obj \
./peripherals/DSP2833x_Sci.obj \
./peripherals/DSP2833x_Spi.obj \
./peripherals/DSP2833x_SysCtrl.obj \
./peripherals/DSP2833x_Xintf.obj \
./peripherals/DSP2833x_usDelay.obj \
./peripherals/swfifo.obj 

ASM_DEPS += \
./peripherals/DSP2833x_ADC_cal.d \
./peripherals/DSP2833x_CSMPasswords.d \
./peripherals/DSP2833x_CodeStartBranch.d \
./peripherals/DSP2833x_DBGIER.d \
./peripherals/DSP2833x_DisInt.d \
./peripherals/DSP2833x_usDelay.d 

OBJS__QUOTED += \
"peripherals\CircleBuffer.obj" \
"peripherals\DSP2833x_ADC_cal.obj" \
"peripherals\DSP2833x_Adc.obj" \
"peripherals\DSP2833x_CSMPasswords.obj" \
"peripherals\DSP2833x_CodeStartBranch.obj" \
"peripherals\DSP2833x_CpuTimers.obj" \
"peripherals\DSP2833x_DBGIER.obj" \
"peripherals\DSP2833x_DMA.obj" \
"peripherals\DSP2833x_DefaultIsr.obj" \
"peripherals\DSP2833x_DisInt.obj" \
"peripherals\DSP2833x_ECan.obj" \
"peripherals\DSP2833x_ECap.obj" \
"peripherals\DSP2833x_EPwm.obj" \
"peripherals\DSP2833x_EQep.obj" \
"peripherals\DSP2833x_GlobalVariableDefs.obj" \
"peripherals\DSP2833x_Gpio.obj" \
"peripherals\DSP2833x_I2C.obj" \
"peripherals\DSP2833x_Mcbsp.obj" \
"peripherals\DSP2833x_MemCopy.obj" \
"peripherals\DSP2833x_PieCtrl.obj" \
"peripherals\DSP2833x_PieVect.obj" \
"peripherals\DSP2833x_Sci.obj" \
"peripherals\DSP2833x_Spi.obj" \
"peripherals\DSP2833x_SysCtrl.obj" \
"peripherals\DSP2833x_Xintf.obj" \
"peripherals\DSP2833x_usDelay.obj" \
"peripherals\swfifo.obj" 

C_DEPS__QUOTED += \
"peripherals\CircleBuffer.d" \
"peripherals\DSP2833x_Adc.d" \
"peripherals\DSP2833x_CpuTimers.d" \
"peripherals\DSP2833x_DMA.d" \
"peripherals\DSP2833x_DefaultIsr.d" \
"peripherals\DSP2833x_ECan.d" \
"peripherals\DSP2833x_ECap.d" \
"peripherals\DSP2833x_EPwm.d" \
"peripherals\DSP2833x_EQep.d" \
"peripherals\DSP2833x_GlobalVariableDefs.d" \
"peripherals\DSP2833x_Gpio.d" \
"peripherals\DSP2833x_I2C.d" \
"peripherals\DSP2833x_Mcbsp.d" \
"peripherals\DSP2833x_MemCopy.d" \
"peripherals\DSP2833x_PieCtrl.d" \
"peripherals\DSP2833x_PieVect.d" \
"peripherals\DSP2833x_Sci.d" \
"peripherals\DSP2833x_Spi.d" \
"peripherals\DSP2833x_SysCtrl.d" \
"peripherals\DSP2833x_Xintf.d" \
"peripherals\swfifo.d" 

ASM_DEPS__QUOTED += \
"peripherals\DSP2833x_ADC_cal.d" \
"peripherals\DSP2833x_CSMPasswords.d" \
"peripherals\DSP2833x_CodeStartBranch.d" \
"peripherals\DSP2833x_DBGIER.d" \
"peripherals\DSP2833x_DisInt.d" \
"peripherals\DSP2833x_usDelay.d" 

C_SRCS__QUOTED += \
"../peripherals/CircleBuffer.c" \
"../peripherals/DSP2833x_Adc.c" \
"../peripherals/DSP2833x_CpuTimers.c" \
"../peripherals/DSP2833x_DMA.c" \
"../peripherals/DSP2833x_DefaultIsr.c" \
"../peripherals/DSP2833x_ECan.c" \
"../peripherals/DSP2833x_ECap.c" \
"../peripherals/DSP2833x_EPwm.c" \
"../peripherals/DSP2833x_EQep.c" \
"../peripherals/DSP2833x_GlobalVariableDefs.c" \
"../peripherals/DSP2833x_Gpio.c" \
"../peripherals/DSP2833x_I2C.c" \
"../peripherals/DSP2833x_Mcbsp.c" \
"../peripherals/DSP2833x_MemCopy.c" \
"../peripherals/DSP2833x_PieCtrl.c" \
"../peripherals/DSP2833x_PieVect.c" \
"../peripherals/DSP2833x_Sci.c" \
"../peripherals/DSP2833x_Spi.c" \
"../peripherals/DSP2833x_SysCtrl.c" \
"../peripherals/DSP2833x_Xintf.c" \
"../peripherals/swfifo.c" 

ASM_SRCS__QUOTED += \
"../peripherals/DSP2833x_ADC_cal.asm" \
"../peripherals/DSP2833x_CSMPasswords.asm" \
"../peripherals/DSP2833x_CodeStartBranch.asm" \
"../peripherals/DSP2833x_DBGIER.asm" \
"../peripherals/DSP2833x_DisInt.asm" \
"../peripherals/DSP2833x_usDelay.asm" 


