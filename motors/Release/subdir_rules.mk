################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
F2806x_CodeStartBranch.obj: ../F2806x_CodeStartBranch.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 -O2 --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.7/include" --include_path="C:/Users/Mishko Lytvyn/Dropbox/WORK/CCSworkspace/motorsPro/head" --define=FLASH --diag_warning=225 --display_error_number --diag_wrap=off --preproc_with_compile --preproc_dependency="F2806x_CodeStartBranch.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

clk.obj: C:/Users/user/Dropbox/WORK/CCSworkspace/MotorsGit/src/clk.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 -O2 --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.7/include" --include_path="C:/Users/Mishko Lytvyn/Dropbox/WORK/CCSworkspace/motorsPro/head" --define=FLASH --diag_warning=225 --display_error_number --diag_wrap=off --preproc_with_compile --preproc_dependency="clk.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

gpio.obj: C:/Users/user/Dropbox/WORK/CCSworkspace/MotorsGit/src/gpio.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 -O2 --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.7/include" --include_path="C:/Users/Mishko Lytvyn/Dropbox/WORK/CCSworkspace/motorsPro/head" --define=FLASH --diag_warning=225 --display_error_number --diag_wrap=off --preproc_with_compile --preproc_dependency="gpio.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 -O2 --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.7/include" --include_path="C:/Users/Mishko Lytvyn/Dropbox/WORK/CCSworkspace/motorsPro/head" --define=FLASH --diag_warning=225 --display_error_number --diag_wrap=off --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

memCopy.obj: C:/Users/user/Dropbox/WORK/CCSworkspace/MotorsGit/src/memCopy.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 -O2 --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.7/include" --include_path="C:/Users/Mishko Lytvyn/Dropbox/WORK/CCSworkspace/motorsPro/head" --define=FLASH --diag_warning=225 --display_error_number --diag_wrap=off --preproc_with_compile --preproc_dependency="memCopy.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


