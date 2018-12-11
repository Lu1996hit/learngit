################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
src/Control/MotionConfig/AxisConfig/AxisConfig.obj: ../src/Control/MotionConfig/AxisConfig/AxisConfig.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"G:/Program Files Use/Ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -Ooff --include_path="E:/Workspace/CCS/SPI20181020/src/Device/myADDA" --include_path="E:/Workspace/CCS/SPI20181020/src/Control/MotionConfig" --include_path="E:/Workspace/CCS/SPI20181020/src/Control/MotionConfig/AxisConfig" --include_path="E:/Workspace/CCS/SPI20181020/src/Device/myLED" --include_path="E:/Workspace/CCS/SPI20181020/src/Control/Kernel" --include_path="E:/Workspace/CCS/SPI20181020/src/Control/MotionMode" --include_path="E:/Workspace/CCS/SPI20181020/src/Device" --include_path="G:/Program Files Use/Ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/include" --include_path="E:/Workspace/CCS/SPI20181020/header/peripheral" --include_path="E:/Workspace/CCS/SPI20181020/header/lib" --include_path="E:/Workspace/CCS/SPI20181020/header/user" --include_path="E:/Workspace/CCS/SPI20181020/src/Device/myPeripherals" --include_path="E:/Workspace/CCS/SPI20181020/src/Device/myIsr" --include_path="E:/Workspace/CCS/SPI20181020/src/Device/myFlash" --include_path="E:/Workspace/CCS/SPI20181020/src/Device/myEXIO" --include_path="E:/Workspace/CCS/SPI20181020/src/Device/myMotors" --include_path="E:/Workspace/CCS/SPI20181020/src/Config" --include_path="E:/Workspace/CCS/SPI20181020/src/Control" --include_path="E:/Workspace/CCS/SPI20181020/src/Task" --include_path="E:/Workspace/CCS/SPI20181020/src/Kernal" --include_path="E:/Workspace/CCS/SPI20181020/src/Communication/arm-dsp" --include_path="E:/Workspace/CCS/SPI20181020/src/Communication/dsp-arm" --advice:performance=all --define=CHECK -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="src/Control/MotionConfig/AxisConfig/AxisConfig.d_raw" --obj_directory="src/Control/MotionConfig/AxisConfig" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


