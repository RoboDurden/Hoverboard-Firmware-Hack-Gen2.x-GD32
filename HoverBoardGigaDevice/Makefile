##########################################################################################################################
# Makefile for Hoverboard-Firmware-Hack-Gen2.x-GD32
##########################################################################################################################

# 1. Target-Nummer aus Inc/defines.h auslesen (ohne Raute im Befehl, um Make-Kommentare zu vermeiden)
TARGET_NUM := $(shell grep -E 'define[ \t]+TARGET[ \t]+[0-9]+' Inc/defines.h | awk '{print $$3}' | tr -d '\r')

# 2. Nummer in den passenden Chip-Namen übersetzen
ifeq ($(TARGET_NUM), 1)
  TARGET = GD32F130
else ifeq ($(TARGET_NUM), 3)
  TARGET = GD32E230
else
  # Default (Fallback für 2 oder falls nichts gefunden wird)
  TARGET = GD32F103
endif

# Temporärer Ordner für die Objektdateien (.o)
BUILD_DIR = build/$(TARGET)

ifeq ($(TARGET), GD32E230)
  CPU = -mcpu=cortex-m23
else
  CPU = -mcpu=cortex-m3
endif

MCU = $(CPU) -mthumb

######################################
# Compiler-Flags (C_DEFS) & Framework-Paths
######################################

C_DEFS = \
-D$(TARGET) \
-DUSE_STDPERIPH_DRIVER \
-D__PIO_DONT_SET_CLOCK_SOURCE__ \
-DSEGGER_RTT_MODE_DEFAULT=SEGGER_RTT_MODE_NO_BLOCK_SKIP

ifeq ($(TARGET), GD32F103)
  SPL_DIR = gd32f10x
  C_DEFS += -DTARGET=2 -D__SYSTEM_CLOCK_72M_PLL_IRC8M=72000000 -DGD32F10X_MD
  CFLAGS_EXTRA = -Wno-unused-function
  LDFLAGS_EXTRA = -Wl,--undefined=SystemCoreClock
else ifeq ($(TARGET), GD32F130)
  SPL_DIR = gd32f1x0
  C_DEFS += -DTARGET=1 -D__SYSTEM_CLOCK_72M_PLL_IRC8M_DIV2=72000000UL -DGD32F130_150
  CFLAGS_EXTRA = 
  LDFLAGS_EXTRA = 
else ifeq ($(TARGET), GD32E230)
  SPL_DIR = gd32e23x
  C_DEFS += -DTARGET=3 -D__SYSTEM_CLOCK_72M_PLL_IRC8M_DIV2=72000000UL
  CFLAGS_EXTRA = 
  LDFLAGS_EXTRA = 
endif

######################################
# C-Sources and Includes
######################################
C_SOURCES = \
$(wildcard Src/*.c) \
$(wildcard lib/framework/gd32/spl/variants/$(SPL_DIR)/src/*.c) \
$(wildcard lib/framework/gd32/cmsis/variants/$(SPL_DIR)/*.c)

C_INCLUDES = \
-IInc \
-Ilib/framework/gd32/cmsis/cores/gd32 \
-Ilib/framework/gd32/cmsis/variants/$(SPL_DIR) \
-Ilib/framework/gd32/spl/variants/$(SPL_DIR)/inc

OPT = -O2
#OPT = -Os

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(CFLAGS_EXTRA)

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

######################################
# Linker & Startup
######################################
LDSCRIPT = lib/spl/gd32f1x0/gd32f1x0.ld
ASM_SOURCES = lib/startup_$(TARGET).s

AS_DEFS = 
AS_INCLUDES = 
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

LIBS = -lc -lm -lnosys 
#LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -Wl,-u,_printf_float $(LDFLAGS_EXTRA)
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections $(LDFLAGS_EXTRA)

######################################
# Toolchain
######################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
######################################
# Build Rules
######################################

# .PHONY sagt Make, dass das keine Dateien, sondern echte Befehle sind
.PHONY: all clean patch_framework

all: patch_framework $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@echo "Compiling $< ..."
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "Assembling $< ..."
	@$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile | $(BUILD_DIR)
	@echo "Linking $@ ..."
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@$(SZ) $@
	@cp $@ build/$(TARGET).elf

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(HEX) $< $@
	@cp $@ build/$(TARGET).hex
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(BIN) $< $@	
	@cp $@ build/$(TARGET).bin
	@echo "Success! Binaries copied to build/ directory"

$(BUILD_DIR):
	mkdir -p $@		


# This patch runs always, but only applies '//' if missing
patch_framework:
	@echo "Patching framework clocks to use internal oscillator (IRC8M)..."
	@# GD32F103 Patch
	@if [ -f lib/framework/gd32/cmsis/variants/gd32f10x/system_gd32f10x.c ]; then \
		sed -i 's/^[ \t]*#define __SYSTEM_CLOCK_108M_PLL_HXTAL/\/\/#define __SYSTEM_CLOCK_108M_PLL_HXTAL/g' lib/framework/gd32/cmsis/variants/gd32f10x/system_gd32f10x.c; \
		sed -i 's/^[ \t]*#define __SYSTEM_CLOCK_72M_PLL_HXTAL/\/\/#define __SYSTEM_CLOCK_72M_PLL_HXTAL/g' lib/framework/gd32/cmsis/variants/gd32f10x/system_gd32f10x.c; \
	fi
	@# GD32F130 Patch
	@if [ -f lib/framework/gd32/cmsis/variants/gd32f1x0/system_gd32f1x0.c ]; then \
		sed -i 's/^[ \t]*#define __SYSTEM_CLOCK_72M_PLL_HXTAL/\/\/#define __SYSTEM_CLOCK_72M_PLL_HXTAL/g' lib/framework/gd32/cmsis/variants/gd32f1x0/system_gd32f1x0.c; \
	fi
	@# GD32E230 Patch
	@if [ -f lib/framework/gd32/cmsis/variants/gd32e23x/system_gd32e23x.c ]; then \
		sed -i 's/^[ \t]*#define __SYSTEM_CLOCK_72M_PLL_HXTAL/\/\/#define __SYSTEM_CLOCK_72M_PLL_HXTAL/g' lib/framework/gd32/cmsis/variants/gd32e23x/system_gd32e23x.c; \
	fi

clean:
	-rm -fR build

######################################
# Dependencies
######################################
-include $(wildcard $(BUILD_DIR)/*.d)