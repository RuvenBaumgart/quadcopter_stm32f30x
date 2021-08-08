#
#
##############################################################################################
#
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# make flash_stlink = build project files and flash the microcontroller via st-flash (https://github.com/texane/stlink.git)
#
# To rebuild project do "make clean" and "make all".
#
# This makefile offers the possibility to build STM32 projects without any IDE.
# Since the latest release of the GNU ARM Plugin for the eclipse IDE is a crap it gives back the
# possibilty to define own project directory structures.
#
# Included originally in the yagarto projects. Original Author : Michael Fischer
# Modified by Franz Flasch
# Use at your own risk!!!!!
#
##############################################################################################
# Start of default section
#
PROJECT = BRIght_flight_01
CCPREFIX   = arm-none-eabi-
CC         = $(CCPREFIX)gcc
CP         = $(CCPREFIX)objcopy
AS         = $(CCPREFIX)gcc -x assembler-with-cpp
GDBTUI     = $(CCPREFIX)gdbtui
HEX        = $(CP) -O ihex
BIN        = $(CP) -O binary -S
MCU        = cortex-m4
 
# List all C defines here
# DDEFS =

# Define optimisation level here
OPT = -Os

OBJ_PATH = obj

#LINKER DIR AND SCRIPT
LINKER_DIR	=/Users/$(USER)/Binary/Roaming/C-Code/StmProjects/stm32f3x/lib/link
LINKER_SCRIPT	= $(LINKER_DIR)/stm32_flash_f303_128k.ld

#StartupCode
STM32F1_DEV_DIR	= /Users/$(USER)/Binary/Roaming/C-Code/StmProjects/stm32f3x/lib/startup
START_UP_CODE	=startup_stm32f30x_md_gcc.S
START_UP	=$(STM32F1_DEV_DIR)/$(START_UP_CODE)

#SOURCE DIRECTORIES 
APP_INC_DIR	= ./

#INCLUDES
#CMSIS_INC
APP_INC_DIR	= /Users/$(USER)/Binary/Roaming/C-Code/StmProjects/stm32f3x/lib/STM32F30x_DSP_StdPeriph_Lib_V1.2.3/Libraries/STM32F30x_StdPeriph_Driver/inc 
APP_INC_DIR	+= /Users/$(USER)/Binary/Roaming/C-Code/StmProjects/stm32f3x/lib/STM32F30x_DSP_StdPeriph_Lib_V1.2.3/Libraries/CMSIS/Include 
APP_INC_DIR	+= /Users/$(USER)/Binary/Roaming/C-Code/StmProjects/stm32f3x/lib/STM32F30x_DSP_StdPeriph_Lib_V1.2.3/Libraries/CMSIS/Device/ST/STM32F30x/Include 

#SOURCE
#CMSIS_SRC
APP_SRC_DIR	= ./
APP_SRC_DIR	+= /Users/$(USER)/Binary/Roaming/C-Code/StmProjects/stm32f3x/lib/STM32F30x_DSP_StdPeriph_Lib_V1.2.3/Libraries/STM32F30x_StdPeriph_Driver/src 
APP_SRC_DIR	+= /Users/$(USER)/Binary/Roaming/C-Code/StmProjects/stm32f3x/lib/STM32F30x_DSP_StdPeriph_Lib_V1.2.3/Libraries/CMSIS/Device/ST/STM32F30x/Source/Templates 

#STM_SRC
#APP_SRC_DIR	+= /Users/$(USER)/Binary/Roaming/C-Code/StmProjects/stm32f1x/lib/core-common-lib-master/STM32F10x_StdPeriph_Driver/src 


#ASSEMBLER_SRC
ASM_SRC_DIR = $(STM32F1_DEV_DIR)

#Search path for source code (.c .s etc)
VPATH = $(APP_SRC_DIR) \
	$(ASM_SRC_DIR) \

#source code in the main folder 
SRC  = $(wildcard *.c)

#EXTERNAL SRC CODE (Not in main directory)
SRC_EXT = $(APP_SRC_DIR)/stm32f30x_gpio.c
SRC_EXT += $(APP_SRC_DIR)/stm32f30x_rcc.c
SRC_EXT += $(APP_SRC_DIR)/stm32f30x_usart.c
SRC_EXT += $(APP_SRC_DIR)/stm32f30x_misc.c
#SRC_EXT += $(APP_SRC_DIR)/core_cm3.c
SRC_EXT += $(APP_SRC_DIR)/system_stm32f30x.c
SRC_EXT += $(APP_SRC_DIR)/stm32f30x_i2c.c
SRC_EXT += $(APP_SRC_DIR)/stm32f30x_tim.c
SRC_EXT += $(APP_SRC_DIR)/stm32f30x_dma.c
SRC_ECT += $(APP_SRC_DIR)/usart.c
SRC_ECT += $(APP_SRC_DIR)/filter.c
SRC_ECT += $(APP_SRC_DIR)/fc_config.c
SRC_ECT += $(APP_SRC_DIR)/stm32f3x_system.c
SRC_ECT += $(APP_SRC_DIR)/buzzer.c
SRC_ECT += $(APP_SRC_DIR)/scheduler.c


#substitution of the folder of all external object files and sort out the matching case for object files. Other wise the inlcude dirs will be part of the files
OBJ_EXT = $(patsubst %.c, $(OBJ_PATH)%.o, $(SRC_EXT))
OBJ_EXT_CLEAN = $(filter %.o, $(OBJ_EXT))

#EXTERNAL ASM SCR CODE ( in this case the Startupfile)
ASM_EXT = $(START_UP)

OBJ_S_EXT = $(patsubst $(ASM_SRC_DIR)/%.S, $(OBJ_PATH)/%.o,$(ASM_EXT))

INCDIRS = $(APP_INC_DIR)


#++++++++++++++++++ DEFINITION OF THE USED Mirco Controler ++++++++++++++++++++++++++++++
# include submakefiles here
#DDEFS += -DSTM32F10X_MD_VL -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000
DDEFS = -DSTM32F30X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000

#Include directories for header files.
INCDIR  = $(patsubst %,-I%, $(INCDIRS))

## run from Flash
DEFS	= $(DDEFS) -DRUN_FROM_FLASH=1

#PRINT OUT VARIABLES FOR INFO
$(info var is [${LINKER_SCRIPT}])

## Search for srcfiles in the main folder
#SCR	= $(wildcard *.c) 
#CC_SCR	= $(wildcard *.cc) 
#CPP_SCR	= $(wildcard *.cpp) 
#convert the source files to object files
#OBJ_CPP	:= $(CPPSCR:.cpp=.o) 
#OBJ_CC	:= $(CCSCR:.cc=.o) 
OBJ_C	= $(SRC:.c=.o)
#OBJ_EXT_C	= $(SRC_EXT:.c=.o)

#EXT_SCR	= $(SRC_EXT/*.c) 
#EXT_CCSCR	= $(SRC_EXT/*.cc)
#EXT_CPPSCR	= $(SRC_EXT/*.cpp)

#OBJECTS_CPP	:= $(patsubst $(SCR)/%,$(OBJ_PATH)/%,$(EXT_CPPSCR:.cpp=.o)) 
#OBJECTS_CC	:= $(patsubst $(SCR)/%,$(OBJ_PATH)/%,$(EXT_CCSCR:.cc=.o)) 
#OBJ_EXT_C_OUT	= $(patsubst $(APP_SRC_DIR)/%, $(OBJ_PATH)/%, $(OBJ_EXT_C:.c=.o))


MCFLAGS = -mcpu=$(MCU)

ASFLAGS = $(MCFLAGS) -g -gdwarf-2 -mthumb  -Wa,-amhls=$(<:.S=.lst) 
CPFLAGS = $(MCFLAGS) $(OPT) -g -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)

# "-Xlinker --gc-sections" - removes unused code from the output binary - saves memory
LDFLAGS = $(MCFLAGS) -g -gdwarf-2 -mthumb -nostartfiles -Xlinker --gc-sections -T$(LINKER_SCRIPT) -lm -Wl,-Map=$(PROJECT).map,--cref,--no-warn-mismatch $(LIBDIR) $(LIB)

######################## makefile rules 
#### OBJECT FILES #####
all:$(OBJ_C) $(OBJ_EXT_CLEAN) $(OBJ_S_EXT) $(PROJECT).elf $(PROJECT).hex $(PROJECT).bin
	$(CCPREFIX)size $(PROJECT).elf

$(OBJ_PATH)/%.o: %.c
	$(CC) $(CPFLAGS) -I. $(INCDIR) -c $^ -o $@

%.o: %.c
	$(CC) -c $(CPFLAGS) -I. $(INCDIR) $< -o $@

$(OBJ_PATH)/%.o: %.S
	$(AS) -c $(ASFLAGS) $< -o $@

#### ELF FILES ####
%elf: $(OBJ_S_EXT) $(OBJ_C) $(OBJ_EXT_CLEAN)
	$(CC) $(OBJ_S_EXT) $(OBJ_C) $(OBJ_EXT_CLEAN) $(LDFLAGS) $(LIBS) -o $@

%hex: %elf
	@echo "Creating HEX $(PROJECT)" 
	$(HEX) --set-start 0x8000000 $< $@
	
%bin: %elf
	$(BIN)  $< $@

	
flash_openocd: $(PROJECT).bin
	openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "reset halt" -c "sleep 100" -c "wait_halt 2" -c "flash write_image erase $(PROJECT).bin 0x08000000" -c "sleep 100" -c "verify_image $(PROJECT).bin 0x08000000" -c "sleep 100" -c "reset run" -c shutdown

flash_stlink: $(PROJECT).bin
	st-flash write $(PROJECT).bin 0x8000000

erase_openocd:
	openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "reset halt" -c "sleep 100" -c "stm32f1x mass_erase 0" -c "sleep 100" -c shutdown 

erase_stlink:
	st-flash erase

debug_openocd: $(PROJECT).elf flash_openocd
	xterm -e openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f0x_stlink.cfg -c "init" -c "halt" -c "reset halt" &
	$(GDBTUI) --eval-command="target remote localhost:3333" $(PROJECT).elf 

debug_stlink: $(PROJECT).elf
	xterm -e st-util &
	$(GDBTUI) --eval-command="target remote localhost:4242"  $(PROJECT).elf -ex 'load'

stmloader: $(PROJECT).hex
	cp -f $(PROJECT).hex stmloader
	stmloader/./stmloader
clean:
	-rm -rf $(OBJ_C)
	-rm -rf $(PROJECT).elf
	-rm -rf $(PROJECT).map
	-rm -rf $(PROJECT).hex
	-rm -rf $(PROJECT).bin
	-rm -rf $(SRC:.c=.lst)
	-rm -rf $(ASRC:.s=.lst)
# *** EOF ***
