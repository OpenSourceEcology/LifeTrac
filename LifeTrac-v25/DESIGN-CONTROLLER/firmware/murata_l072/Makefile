# Makefile — Murata L072 custom firmware (skeleton)
#
# Single canonical build entry point per DESIGN-LORAFIRMWARE/02 §5 (and
# Claude review §2.4 — drop the second build system to avoid bit-rot).
#
# This Makefile's primary job at skeleton stage is to enforce the memory
# map: it preprocesses include/memory_map.h into the linker script so the
# .ld file never hand-mirrors an address, and it compiles
# include/static_asserts.c so the build fails immediately if the map drifts.
#
# Phase 1 will extend this Makefile with HAL/CMSIS sources and the actual
# arm-none-eabi-* invocation. For now `make check` is the only target that
# runs without a cross toolchain; full `make` requires arm-none-eabi-gcc.

# --------------------------------------------------------------------------
# Toolchain
# --------------------------------------------------------------------------
CROSS    ?= arm-none-eabi-
CC       := $(CROSS)gcc
CPP      := $(CROSS)cpp
LD       := $(CROSS)gcc
OBJCOPY  := $(CROSS)objcopy
SIZE     := $(CROSS)size

TARGET   := firmware
BUILD    := build

# --------------------------------------------------------------------------
# Sources (skeleton stage — extended in Phase 1)
# --------------------------------------------------------------------------
SRCS := \
	startup.c \
    main.c \
    boot/safe_mode.c \
	hal/platform.c \
	host/host_uart.c \
	radio/sx1276.c \
    include/static_asserts.c

OBJS := $(SRCS:%.c=$(BUILD)/%.o)
DEPS := $(OBJS:.o=.d)

# --------------------------------------------------------------------------
# Flags
# --------------------------------------------------------------------------
MCU_FLAGS := \
    -mcpu=cortex-m0plus \
    -mthumb \
    -mfloat-abi=soft

WARN_FLAGS := \
    -Wall -Wextra -Werror \
    -Wno-unused-function \
    -Wno-unused-variable

OPT_FLAGS := -Os -ffunction-sections -fdata-sections

CFLAGS := \
    $(MCU_FLAGS) \
    $(WARN_FLAGS) \
    $(OPT_FLAGS) \
    -std=gnu11 \
    -Iinclude -I. \
    -fno-common \
    -fno-builtin \
    -MMD -MP

# --------------------------------------------------------------------------
# Linker script — preprocessed from ld/stm32l072cz_flash.ld so it can
# #include include/memory_map.h. This is the mechanism that keeps the
# memory map single-sourced (Codex review §E, Claude review §5.2 E).
# --------------------------------------------------------------------------
LD_SRC      := ld/stm32l072cz_flash.ld
LD_GEN      := $(BUILD)/$(TARGET).ld

LDFLAGS := \
    $(MCU_FLAGS) \
    -T$(LD_GEN) \
    -nostartfiles \
    -Wl,--gc-sections \
    -Wl,--no-warn-rwx-segments \
    -Wl,-Map=$(BUILD)/$(TARGET).map \
    -specs=nano.specs \
    -specs=nosys.specs

# --------------------------------------------------------------------------
# Targets
# --------------------------------------------------------------------------
.PHONY: all clean size check check-cpp check-asserts help

all: $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex size

help:
	@echo "Targets:"
	@echo "  all     — full build (requires arm-none-eabi-gcc)"
	@echo "  size    — show region usage against the memory budget"
	@echo "  check   — preprocessor + memory-map static asserts only (no cross toolchain)"
	@echo "  clean   — remove build artifacts"

# Generated linker script — runs the C preprocessor on the .ld template
# so MM_* constants from include/memory_map.h expand inline.
$(LD_GEN): $(LD_SRC) include/memory_map.h | $(BUILD)
	@mkdir -p $(dir $@)
	$(CPP) -P -nostdinc -undef -Iinclude -x c $< -o $@

# Object compilation
$(BUILD)/%.o: %.c | $(BUILD)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# Link
$(BUILD)/$(TARGET).elf: $(OBJS) $(LD_GEN)
	$(LD) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

size: $(BUILD)/$(TARGET).elf
	@echo
	@echo "Section sizes:"
	@$(SIZE) -A -d $<
	@echo
	@echo "Region budgets (from include/memory_map.h):"
	@echo "  BOOT  = 4 KB    APP = 180 KB    CFG = 8 KB    Total = 192 KB"
	@echo "  RAM   = 20 KB    Stack reserved = 2.5 KB"

# --------------------------------------------------------------------------
# `make check` — runs WITHOUT a cross toolchain. Verifies that:
#   1. memory_map.h preprocesses cleanly via the host C preprocessor.
#   2. static_asserts.c compiles via the host C compiler (the asserts are
#      C11 _Static_assert which evaluates at compile time, identical to
#      the cross compile).
# This is what CI runs today, before Phase 1 lands the real toolchain.
# --------------------------------------------------------------------------
HOST_CC ?= cc

check: check-cpp check-asserts
	@echo "[OK] memory map invariants hold"

check-cpp: $(BUILD)
	@echo "[1/2] Preprocessing memory_map.h ..."
	@$(HOST_CC) -E -P -Iinclude -x c include/memory_map.h > $(BUILD)/memory_map.preproc 2>&1
	@echo "      -> $(BUILD)/memory_map.preproc"

check-asserts: $(BUILD)
	@echo "[2/2] Compiling include/static_asserts.c (host) ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude \
	    -c include/static_asserts.c -o $(BUILD)/static_asserts.host.o

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)

-include $(DEPS)
