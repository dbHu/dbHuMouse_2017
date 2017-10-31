################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/portable/CCS/ARM_CM4F/port.obj: ../FreeRTOS/portable/CCS/ARM_CM4F/port.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" -mv4 --code_state=32 -O2 --include_path="D:/doubi/desktop/dbMouse_2017/ccs_test/dbMouse_FreeRTOS" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="FreeRTOS/portable/CCS/ARM_CM4F/port.d" --obj_directory="FreeRTOS/portable/CCS/ARM_CM4F" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

FreeRTOS/portable/CCS/ARM_CM4F/portasm.obj: ../FreeRTOS/portable/CCS/ARM_CM4F/portasm.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" -mv4 --code_state=32 -O2 --include_path="D:/doubi/desktop/dbMouse_2017/ccs_test/dbMouse_FreeRTOS" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="FreeRTOS/portable/CCS/ARM_CM4F/portasm.d" --obj_directory="FreeRTOS/portable/CCS/ARM_CM4F" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


