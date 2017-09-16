################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS

GEN_OPTS__FLAG := --cmd_file="configPkg/compiler.opt"
GEN_CMDS__FLAG := -l"configPkg/linker.cmd"

ORDERED_OBJS += \
"./dbmouse.obj" \
"./dbmouse_chassis.obj" \
"./solve/solve.obj" \
"./mouse/mouse.obj" \
"./mmaze/mmaze.obj" \
"./action/action.obj" \
"./TskTop/DbgUart.obj" \
"./TskTop/TskTop.obj" \
"./TskMotor/Imu.obj" \
"./TskMotor/MotorPwm.obj" \
"./TskMotor/TskMotor.obj" \
"./TskMotor/WheelEnc.obj" \
"./TskIr/IrCorr.obj" \
"./TskIr/TskIr.obj" \
"./Queue/Queue.obj" \
"./Pid/Pid.obj" \
"./Kalman/Kalman1Var.obj" \
"../dbmouse_chassis.cmd" \
$(GEN_CMDS__FLAG) \
-l"C:/ti/tirtos_tivac_2_16_01_14/products/TivaWare_C_Series-2.1.1.71b/grlib/ccs/Debug/grlib.lib" \
-l"C:/ti/tirtos_tivac_2_16_01_14/products/TivaWare_C_Series-2.1.1.71b/usblib/ccs/Debug/usblib.lib" \
-l"C:/ti/tirtos_tivac_2_16_01_14/products/TivaWare_C_Series-2.1.1.71b/driverlib/ccs/Debug/driverlib.lib" \
-l"libc.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include solve/subdir_vars.mk
-include mouse/subdir_vars.mk
-include mmaze/subdir_vars.mk
-include action/subdir_vars.mk
-include TskTop/subdir_vars.mk
-include TskMotor/subdir_vars.mk
-include TskIr/subdir_vars.mk
-include Queue/subdir_vars.mk
-include Pid/subdir_vars.mk
-include Kalman/subdir_vars.mk
-include subdir_rules.mk
-include solve/subdir_rules.mk
-include mouse/subdir_rules.mk
-include mmaze/subdir_rules.mk
-include action/subdir_rules.mk
-include TskTop/subdir_rules.mk
-include TskMotor/subdir_rules.mk
-include TskIr/subdir_rules.mk
-include Queue/subdir_rules.mk
-include Pid/subdir_rules.mk
-include Kalman/subdir_rules.mk
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
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
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
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
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
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
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
EXE_OUTPUTS += \
dbHu_m.out \

EXE_OUTPUTS__QUOTED += \
"dbHu_m.out" \

BIN_OUTPUTS += \
dbHu_m.hex \

BIN_OUTPUTS__QUOTED += \
"dbHu_m.hex" \


# All Target
all: dbHu_m.out

# Tool invocations
dbHu_m.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O4 --opt_for_speed=1 --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=ccs --define=TIVAWARE -g --gcc --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi -z -m"dbHu_m.map" --heap_size=0 --stack_size=512 -i"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS/lib" -i"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="dbHu_m_linkInfo.xml" --rom_model -o "dbHu_m.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

dbHu_m.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS/bin/armhex"  -o "dbHu_m.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_OPTS__QUOTED)$(GEN_CMDS__QUOTED)
	-$(RM) "Kalman\Kalman1Var.pp" 
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-$(RM) "dbmouse_chassis.pp" 
	-$(RM) "dbmouse.obj" "dbmouse_chassis.obj" "solve\solve.obj" "mouse\mouse.obj" "mmaze\mmaze.obj" "action\action.obj" "TskTop\DbgUart.obj" "TskTop\TskTop.obj" "TskMotor\Imu.obj" "TskMotor\MotorPwm.obj" "TskMotor\TskMotor.obj" "TskMotor\WheelEnc.obj" "TskIr\IrCorr.obj" "TskIr\TskIr.obj" "Queue\Queue.obj" "Pid\Pid.obj" "Kalman\Kalman1Var.obj" 
	-$(RM) "dbmouse.pp" "solve\solve.pp" "mouse\mouse.pp" "mmaze\mmaze.pp" "action\action.pp" "TskTop\DbgUart.pp" "TskTop\TskTop.pp" "TskMotor\Imu.pp" "TskMotor\MotorPwm.pp" "TskMotor\TskMotor.pp" "TskMotor\WheelEnc.pp" "TskIr\IrCorr.pp" "TskIr\TskIr.pp" "Queue\Queue.pp" "Pid\Pid.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

