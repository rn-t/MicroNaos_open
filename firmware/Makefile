##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.16.0] date: [Tue Apr 19 17:06:02 JST 2022] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = MicroNaos


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
# Core/Src/main.c 

C_SOURCES = \
USB_DEVICE/App/usb_device.c \
USB_DEVICE/App/usbd_desc.c \
USB_DEVICE/Target/usbd_conf.c \
$(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c) \
$(wildcard Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/*.c) \
$(wildcard Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c) \
$(wildcard lib/SEGGER_RTT_V764b/RTT/*.c) \
$(wildcard lib/SEGGER_RTT_V764b/Syscalls/*.c) \
$(wildcard Core/Src/*.c)

# CPP sources
CPP_SOURCES = \
$(wildcard Core/Src/*.cpp) \
$(wildcard Core/Peripherals/Src/*.cpp) \
$(filter-out lib/MazeLibrary/Src/simulator.cpp, $(wildcard lib/MazeLibrary/Src/*.cpp))

# ASM sources
ASM_SOURCES =  \
startup_stm32f405xx.s


#######################################
# binaries
#######################################
BIN_PATH = /opt/gcc-arm-none-eabi/bin
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.

CC = $(BIN_PATH)/$(PREFIX)g++
AS = $(BIN_PATH)/$(PREFIX)g++ -x assembler-with-cpp
CP = $(BIN_PATH)/$(PREFIX)objcopy
AR = $(BIN_PATH)/$(PREFIX)ar
SZ = $(BIN_PATH)/$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F405xx \
-DARM_MATH_CM4 \
-DUSE_ARM_GCC


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/CMSIS/DSP/Include \
-IUSB_DEVICE/App \
-IUSB_DEVICE/Target \
-ICore/Inc \
-ICore/Peripherals/Inc \
-Ilib/SEGGER_RTT_V764b/RTT \
-Ilib/SEGGER_RTT_V764b/Config \
-Ilib/SEGGER_RTT_V764b/Syscalls \
-Ilib/MazeLibrary/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/HID/Inc \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F405RGTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys -larm_cortexM4lf_math
LIBDIR = -L Drivers/CMSIS/Lib/GCC
LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -u _printf_float

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of C objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of CPP objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(CPP_SOURCES:.cpp=.o))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	mkdir -p $(dir $@);
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(<:.c=.lst) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	mkdir -p $(dir $@);
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(<:.cpp=.lst) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(OBJECTS:%.o=%.d)

# *** EOF ***
