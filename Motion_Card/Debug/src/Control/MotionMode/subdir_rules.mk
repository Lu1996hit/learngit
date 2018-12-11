################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
src/Control/MotionMode/Crdmode.obj: ../src/Control/MotionMode/Crdmode.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -Ooff --include_path="E:/v101/Motion_Card(Independence)/src/Device/myADDA" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionConfig" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionConfig/AxisSet" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myLED" --include_path="E:/v101/Motion_Card(Independence)/src/Control/Kernel" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionMode" --include_path="E:/v101/Motion_Card(Independence)/src/Device" --include_path="D:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/include" --include_path="E:/v101/Motion_Card(Independence)/header/peripheral" --include_path="E:/v101/Motion_Card(Independence)/header/lib" --include_path="E:/v101/Motion_Card(Independence)/header/user" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myPeripherals" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myIsr" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myFlash" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myEXIO" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myMotors" --include_path="E:/v101/Motion_Card(Independence)/src/Config" --include_path="E:/v101/Motion_Card(Independence)/src/Control" --include_path="E:/v101/Motion_Card(Independence)/src/Task" --include_path="E:/v101/Motion_Card(Independence)/src/Kernal" --include_path="E:/v101/Motion_Card(Independence)/src/Communication/arm-dsp" --include_path="E:/v101/Motion_Card(Independence)/src/Communication/dsp-arm" --advice:performance=all --define=CKECK --define=CHECK -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="src/Control/MotionMode/Crdmode.d_raw" --obj_directory="src/Control/MotionMode" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/Control/MotionMode/JOGmode.obj: ../src/Control/MotionMode/JOGmode.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -Ooff --include_path="E:/v101/Motion_Card(Independence)/src/Device/myADDA" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionConfig" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionConfig/AxisSet" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myLED" --include_path="E:/v101/Motion_Card(Independence)/src/Control/Kernel" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionMode" --include_path="E:/v101/Motion_Card(Independence)/src/Device" --include_path="D:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/include" --include_path="E:/v101/Motion_Card(Independence)/header/peripheral" --include_path="E:/v101/Motion_Card(Independence)/header/lib" --include_path="E:/v101/Motion_Card(Independence)/header/user" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myPeripherals" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myIsr" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myFlash" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myEXIO" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myMotors" --include_path="E:/v101/Motion_Card(Independence)/src/Config" --include_path="E:/v101/Motion_Card(Independence)/src/Control" --include_path="E:/v101/Motion_Card(Independence)/src/Task" --include_path="E:/v101/Motion_Card(Independence)/src/Kernal" --include_path="E:/v101/Motion_Card(Independence)/src/Communication/arm-dsp" --include_path="E:/v101/Motion_Card(Independence)/src/Communication/dsp-arm" --advice:performance=all --define=CHECK -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="src/Control/MotionMode/JOGmode.d_raw" --obj_directory="src/Control/MotionMode" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/Control/MotionMode/PPmode.obj: ../src/Control/MotionMode/PPmode.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -Ooff --include_path="E:/v101/Motion_Card(Independence)/src/Device/myADDA" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionConfig" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionConfig/AxisSet" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myLED" --include_path="E:/v101/Motion_Card(Independence)/src/Control/Kernel" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionMode" --include_path="E:/v101/Motion_Card(Independence)/src/Device" --include_path="D:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/include" --include_path="E:/v101/Motion_Card(Independence)/header/peripheral" --include_path="E:/v101/Motion_Card(Independence)/header/lib" --include_path="E:/v101/Motion_Card(Independence)/header/user" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myPeripherals" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myIsr" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myFlash" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myEXIO" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myMotors" --include_path="E:/v101/Motion_Card(Independence)/src/Config" --include_path="E:/v101/Motion_Card(Independence)/src/Control" --include_path="E:/v101/Motion_Card(Independence)/src/Task" --include_path="E:/v101/Motion_Card(Independence)/src/Kernal" --include_path="E:/v101/Motion_Card(Independence)/src/Communication/arm-dsp" --include_path="E:/v101/Motion_Card(Independence)/src/Communication/dsp-arm" --advice:performance=all --define=CHECK -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="src/Control/MotionMode/PPmode.d_raw" --obj_directory="src/Control/MotionMode" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/Control/MotionMode/PTmode.obj: ../src/Control/MotionMode/PTmode.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -Ooff --include_path="E:/v101/Motion_Card(Independence)/src/Device/myADDA" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionConfig" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionConfig/AxisSet" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myLED" --include_path="E:/v101/Motion_Card(Independence)/src/Control/Kernel" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionMode" --include_path="E:/v101/Motion_Card(Independence)/src/Device" --include_path="D:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/include" --include_path="E:/v101/Motion_Card(Independence)/header/peripheral" --include_path="E:/v101/Motion_Card(Independence)/header/lib" --include_path="E:/v101/Motion_Card(Independence)/header/user" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myPeripherals" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myIsr" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myFlash" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myEXIO" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myMotors" --include_path="E:/v101/Motion_Card(Independence)/src/Config" --include_path="E:/v101/Motion_Card(Independence)/src/Control" --include_path="E:/v101/Motion_Card(Independence)/src/Task" --include_path="E:/v101/Motion_Card(Independence)/src/Kernal" --include_path="E:/v101/Motion_Card(Independence)/src/Communication/arm-dsp" --include_path="E:/v101/Motion_Card(Independence)/src/Communication/dsp-arm" --advice:performance=all --define=CHECK -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="src/Control/MotionMode/PTmode.d_raw" --obj_directory="src/Control/MotionMode" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/Control/MotionMode/STOPmode.obj: ../src/Control/MotionMode/STOPmode.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -Ooff --include_path="E:/v101/Motion_Card(Independence)/src/Device/myADDA" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionConfig" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionConfig/AxisSet" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myLED" --include_path="E:/v101/Motion_Card(Independence)/src/Control/Kernel" --include_path="E:/v101/Motion_Card(Independence)/src/Control/MotionMode" --include_path="E:/v101/Motion_Card(Independence)/src/Device" --include_path="D:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.1.LTS/include" --include_path="E:/v101/Motion_Card(Independence)/header/peripheral" --include_path="E:/v101/Motion_Card(Independence)/header/lib" --include_path="E:/v101/Motion_Card(Independence)/header/user" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myPeripherals" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myIsr" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myFlash" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myEXIO" --include_path="E:/v101/Motion_Card(Independence)/src/Device/myMotors" --include_path="E:/v101/Motion_Card(Independence)/src/Config" --include_path="E:/v101/Motion_Card(Independence)/src/Control" --include_path="E:/v101/Motion_Card(Independence)/src/Task" --include_path="E:/v101/Motion_Card(Independence)/src/Kernal" --include_path="E:/v101/Motion_Card(Independence)/src/Communication/arm-dsp" --include_path="E:/v101/Motion_Card(Independence)/src/Communication/dsp-arm" --advice:performance=all --define=CHECK -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="src/Control/MotionMode/STOPmode.d_raw" --obj_directory="src/Control/MotionMode" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


