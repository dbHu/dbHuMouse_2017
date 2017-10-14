################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS

GEN_OPTS__FLAG := --cmd_file="configPkg/compiler.opt"
GEN_CMDS__FLAG := -l"configPkg/linker.cmd"

ORDERED_OBJS += \
"./dbmouse.obj" \
"./dbmouse_chassis.obj" \
"./Kalman/Kalman1Var.obj" \
"./Kalman/Kalman2Var.obj" \
"./Pid/Pid.obj" \
"./Queue/Queue.obj" \
"./TskIr/IrCorr.obj" \
"./TskIr/TskIr.obj" \
"./TskMotor/Imu.obj" \
"./TskMotor/MotorPwm.obj" \
"./TskMotor/TskMotor.obj" \
"./TskMotor/WheelEnc.obj" \
"./TskTop/DbgUart.obj" \
"./TskTop/TskTop.obj" \
"./TskTop/cmd.obj" \
"./action/action.obj" \
"./mmaze/mmaze.obj" \
"./mouse/mouse.obj" \
"./solve/solve.obj" \
"../dbmouse_chassis.cmd" \
$(GEN_CMDS__FLAG) \
-l"C:/ti/tirtos_tivac_2_16_01_14/products/TivaWare_C_Series-2.1.1.71b/grlib/ccs/Debug/grlib.lib" \
-l"C:/ti/tirtos_tivac_2_16_01_14/products/TivaWare_C_Series-2.1.1.71b/usblib/ccs/Debug/usblib.lib" \
-l"C:/ti/tirtos_tivac_2_16_01_14/products/TivaWare_C_Series-2.1.1.71b/driverlib/ccs/Debug/driverlib.lib" \
-llibc.a \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include Kalman/subdir_vars.mk
-include Pid/subdir_vars.mk
-include Queue/subdir_vars.mk
-include TskIr/subdir_vars.mk
-include TskMotor/subdir_vars.mk
-include TskTop/subdir_vars.mk
-include action/subdir_vars.mk
-include mmaze/subdir_vars.mk
-include mouse/subdir_vars.mk
-include solve/subdir_vars.mk
-include subdir_rules.mk
-include Kalman/subdir_rules.mk
-include Pid/subdir_rules.mk
-include Queue/subdir_rules.mk
-include TskIr/subdir_rules.mk
-include TskMotor/subdir_rules.mk
-include TskTop/subdir_rules.mk
-include action/subdir_rules.mk
-include mmaze/subdir_rules.mk
-include mouse/subdir_rules.mk
-include solve/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
dbHu_mouse.out \

EXE_OUTPUTS__QUOTED += \
"dbHu_mouse.out" \

BIN_OUTPUTS += \
dbHu_mouse.hex \

BIN_OUTPUTS__QUOTED += \
"dbHu_mouse.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(LDS_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "dbHu_mouse.out"

# Tool invocations
dbHu_mouse.out: $(OBJS) $(CMD_SRCS) $(LDS_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=ccs --define=TIVAWARE -g --gcc --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi -z -m"dbHu_mouse.map" --heap_size=1024 --stack_size=512 -i"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/lib" -i"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="dbHu_mouse_linkInfo.xml" --rom_model -o "dbHu_mouse.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

dbHu_mouse.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armhex"  -o "dbHu_mouse.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_CMDS__QUOTED)$(GEN_MISC_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)$(GEN_OPTS__QUOTED)
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-$(RM) "dbmouse.obj" "dbmouse_chassis.obj" "Kalman\Kalman1Var.obj" "Kalman\Kalman2Var.obj" "Pid\Pid.obj" "Queue\Queue.obj" "TskIr\IrCorr.obj" "TskIr\TskIr.obj" "TskMotor\Imu.obj" "TskMotor\MotorPwm.obj" "TskMotor\TskMotor.obj" "TskMotor\WheelEnc.obj" "TskTop\DbgUart.obj" "TskTop\TskTop.obj" "TskTop\cmd.obj" "action\action.obj" "mmaze\mmaze.obj" "mouse\mouse.obj" "solve\solve.obj" 
	-$(RM) "dbmouse_chassis.d" 
	-$(RM) "dbmouse.d" "Kalman\Kalman1Var.d" "Kalman\Kalman2Var.d" "Pid\Pid.d" "Queue\Queue.d" "TskIr\IrCorr.d" "TskIr\TskIr.d" "TskMotor\Imu.d" "TskMotor\MotorPwm.d" "TskMotor\TskMotor.d" "TskMotor\WheelEnc.d" "TskTop\DbgUart.d" "TskTop\TskTop.d" "TskTop\cmd.d" "action\action.d" "mmaze\mmaze.d" "mouse\mouse.d" "solve\solve.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

