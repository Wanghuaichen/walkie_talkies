################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
CC1310_LAUNCHXL.obj: ../CC1310_LAUNCHXL.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/a0227556/Desktop/Intern DIY Showcase/uartRxTx_CC1350_LAUNCHXL_TI/uartRxTx_CC1350_LAUNCHXL_TI" --include_path="C:/Users/a0227556/Desktop/Intern DIY Showcase/uartRxTx_CC1350_LAUNCHXL_TI/uartRxTx_CC1350_LAUNCHXL_TI/smartrf_settings" --include_path="/products/cc13xxware_2_04_03_17272" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="CC1310_LAUNCHXL.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

RFQueue.obj: ../RFQueue.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/a0227556/Desktop/Intern DIY Showcase/uartRxTx_CC1350_LAUNCHXL_TI/uartRxTx_CC1350_LAUNCHXL_TI" --include_path="C:/Users/a0227556/Desktop/Intern DIY Showcase/uartRxTx_CC1350_LAUNCHXL_TI/uartRxTx_CC1350_LAUNCHXL_TI/smartrf_settings" --include_path="/products/cc13xxware_2_04_03_17272" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="RFQueue.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

ccfg.obj: ../ccfg.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/a0227556/Desktop/Intern DIY Showcase/uartRxTx_CC1350_LAUNCHXL_TI/uartRxTx_CC1350_LAUNCHXL_TI" --include_path="C:/Users/a0227556/Desktop/Intern DIY Showcase/uartRxTx_CC1350_LAUNCHXL_TI/uartRxTx_CC1350_LAUNCHXL_TI/smartrf_settings" --include_path="/products/cc13xxware_2_04_03_17272" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="ccfg.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

build-633368735:
	@$(MAKE) -Onone -f subdir_rules.mk build-633368735-inproc

build-633368735-inproc: ../rfExamples.cfg
	@echo 'Building file: $<'
	@echo 'Invoking: XDCtools'
	"C:/ti/xdctools_3_32_02_25_core/xs" --xdcpath="C:/ti/ccsv7/ccs_base;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC1350F128 -r release -c "C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS" --compileOptions "-mv7M3 --code_state=16 --float_support=vfplib -me --include_path=\"C:/Users/a0227556/Desktop/Intern DIY Showcase/uartRxTx_CC1350_LAUNCHXL_TI/uartRxTx_CC1350_LAUNCHXL_TI\" --include_path=\"C:/Users/a0227556/Desktop/Intern DIY Showcase/uartRxTx_CC1350_LAUNCHXL_TI/uartRxTx_CC1350_LAUNCHXL_TI/smartrf_settings\" --include_path=\"/products/cc13xxware_2_04_03_17272\" --include_path=\"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include\" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi  " "$<"
	@echo 'Finished building: $<'
	@echo ' '

configPkg/linker.cmd: build-633368735 ../rfExamples.cfg
configPkg/compiler.opt: build-633368735
configPkg/: build-633368735

rfPacketRx.obj: ../rfPacketRx.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/a0227556/Desktop/Intern DIY Showcase/uartRxTx_CC1350_LAUNCHXL_TI/uartRxTx_CC1350_LAUNCHXL_TI" --include_path="C:/Users/a0227556/Desktop/Intern DIY Showcase/uartRxTx_CC1350_LAUNCHXL_TI/uartRxTx_CC1350_LAUNCHXL_TI/smartrf_settings" --include_path="/products/cc13xxware_2_04_03_17272" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="rfPacketRx.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


