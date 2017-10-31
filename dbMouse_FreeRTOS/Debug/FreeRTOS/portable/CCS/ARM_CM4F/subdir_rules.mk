################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/portable/CCS/ARM_CM4F/port.obj: ../FreeRTOS/portable/CCS/ARM_CM4F/port.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O0 --opt_for_speed=0 --include_path="D:/doubi/desktop/dbMouse_2017/ccs_test/dbMouse_FreeRTOS" --include_path="D:/doubi/desktop/dbMouse_2017/ccs_test/dbMouse_FreeRTOS/TskMotor" --include_path="D:/doubi/desktop/dbMouse_2017/ccs_test/dbMouse_FreeRTOS/FreeRTOS/portable/CCS/ARM_CM4F" --include_path="D:/doubi/desktop/dbMouse_2017/ccs_test/dbMouse_FreeRTOS/FreeRTOS/include" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --include_path="C:/ti/tirtos_tivac_2_16_01_14/products/TivaWare_C_Series-2.1.1.71b" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA1 -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="FreeRTOS/portable/CCS/ARM_CM4F/port.d" --obj_directory="FreeRTOS/portable/CCS/ARM_CM4F" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

FreeRTOS/portable/CCS/ARM_CM4F/portasm.obj: ../FreeRTOS/portable/CCS/ARM_CM4F/portasm.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O0 --opt_for_speed=0 --include_path="D:/doubi/desktop/dbMouse_2017/ccs_test/dbMouse_FreeRTOS" --include_path="D:/doubi/desktop/dbMouse_2017/ccs_test/dbMouse_FreeRTOS/TskMotor" --include_path="D:/doubi/desktop/dbMouse_2017/ccs_test/dbMouse_FreeRTOS/FreeRTOS/portable/CCS/ARM_CM4F" --include_path="D:/doubi/desktop/dbMouse_2017/ccs_test/dbMouse_FreeRTOS/FreeRTOS/include" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --include_path="C:/ti/tirtos_tivac_2_16_01_14/products/TivaWare_C_Series-2.1.1.71b" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA1 -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="FreeRTOS/portable/CCS/ARM_CM4F/portasm.d" --obj_directory="FreeRTOS/portable/CCS/ARM_CM4F" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


