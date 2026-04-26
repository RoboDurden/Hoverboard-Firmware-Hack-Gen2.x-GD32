##########################################################################################################################
# Makefile for Hoverboard-Firmware-Hack-Gen2.x-GD32
# Liegt im Root-Verzeichnis. Nutzt einheitlich 8K RAM Linker-Skript.
##########################################################################################################################

TARGET ?= GD32F103

BUILD_DIR = build/$(TARGET)

ifeq ($(TARGET), GD32E230)
  CPU = -mcpu=cortex-m23
else
  CPU = -mcpu=cortex-m3
endif

MCU = $(CPU) -mthumb

######################################
# C-Quellen und Includes
######################################
C_SOURCES = \
$(wildcard Src/*.c) \
$(wildcard lib/CMSIS/*.c) \
$(wildcard lib/GD32F10x_standard_peripheral/Source/*.c)

C_INCLUDES = \
-IInc \
-Ilib/CMSIS \
-Ilib/GD32F10x_standard_peripheral/Include

######################################
# Compiler-Flags (C_DEFS)
######################################
C_DEFS = \
-D$(TARGET) \
-DUSE_STDPERIPH_DRIVER \
-D__PIO_DONT_SET_CLOCK_SOURCE__

ifeq ($(TARGET), GD32F103)
  C_DEFS += -DTARGET=2 -D__SYSTEM_CLOCK_72M_PLL_IRC8M
  CFLAGS_EXTRA = -Wno-unused-function
  LDFLAGS_EXTRA = -Wl,--undefined=SystemCoreClock
else ifeq ($(TARGET), GD32F130)
  C_DEFS += -DTARGET=1 -D__SYSTEM_CLOCK_72M_PLL_IRC8M_DIV2=72000000UL
  CFLAGS_EXTRA = 
  LDFLAGS_EXTRA = 
else ifeq ($(TARGET), GD32E230)
  C_DEFS += -DTARGET=3 -D__SYSTEM_CLOCK_72M_PLL_IRC8M_DIV2=72000000UL
  CFLAGS_EXTRA = 
  LDFLAGS_EXTRA = 
endif

OPT = -Os

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections $(CFLAGS_EXTRA)

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

######################################
# Linker & Startup-Dateien
######################################
# Nutzt einheitlich das 8K RAM Skript für alle Targets
LDSCRIPT = lib/spl/gd32f1x0/gd32f1x0.ld

# Sucht die Startup-Dateien nun im lib/ Ordner
ASM_SOURCES = lib/startup_$(TARGET).s

AS_DEFS = 
AS_INCLUDES = 
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

LIBS = -lc -lm -lnosys 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -Wl,-u,_printf_float $(LDFLAGS_EXTRA)

######################################
# Toolchain (GCC)
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
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@echo "Kompiliere $< ..."
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "Assembliere $< ..."
	@$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo "Linke $@ ..."
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(BIN) $< $@	
	@echo "Fertig! Binaries liegen in $(BUILD_DIR)"

$(BUILD_DIR):
	mkdir $@		

clean:
	-rm -fR build

######################################
# Abhängigkeiten
######################################
-include $(wildcard $(BUILD_DIR)/*.d)