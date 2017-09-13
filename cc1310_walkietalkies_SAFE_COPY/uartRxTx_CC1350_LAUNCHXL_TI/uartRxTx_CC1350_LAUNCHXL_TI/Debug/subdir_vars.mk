################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CFG_SRCS += \
../rfExamples.cfg 

CMD_SRCS += \
../CC1310_LAUNCHXL_TIRTOS.cmd \
../cc13x0f128.cmd 

C_SRCS += \
../CC1310_LAUNCHXL.c \
../RFQueue.c \
../ccfg.c \
../rfPacketRx.c 

GEN_CMDS += \
./configPkg/linker.cmd 

GEN_FILES += \
./configPkg/linker.cmd \
./configPkg/compiler.opt 

GEN_MISC_DIRS += \
./configPkg/ 

C_DEPS += \
./CC1310_LAUNCHXL.d \
./RFQueue.d \
./ccfg.d \
./rfPacketRx.d 

GEN_OPTS += \
./configPkg/compiler.opt 

OBJS += \
./CC1310_LAUNCHXL.obj \
./RFQueue.obj \
./ccfg.obj \
./rfPacketRx.obj 

GEN_MISC_DIRS__QUOTED += \
"configPkg\" 

OBJS__QUOTED += \
"CC1310_LAUNCHXL.obj" \
"RFQueue.obj" \
"ccfg.obj" \
"rfPacketRx.obj" 

C_DEPS__QUOTED += \
"CC1310_LAUNCHXL.d" \
"RFQueue.d" \
"ccfg.d" \
"rfPacketRx.d" 

GEN_FILES__QUOTED += \
"configPkg\linker.cmd" \
"configPkg\compiler.opt" 

C_SRCS__QUOTED += \
"../CC1310_LAUNCHXL.c" \
"../RFQueue.c" \
"../ccfg.c" \
"../rfPacketRx.c" 


