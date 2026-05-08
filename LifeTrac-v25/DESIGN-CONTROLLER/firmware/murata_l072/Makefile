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

HELLO_TARGET := hello_world
HELLO_BUILD  := build_hello
HELLO_SRCS   := startup.c hal/platform.c hello_world.c include/static_asserts.c

PING_SRCS    := startup.c hal/platform.c lora_ping.c include/static_asserts.c

# EXTRA_CFLAGS: injected by recursive lora_ping_tx / lora_ping_rx targets
EXTRA_CFLAGS ?=

# --------------------------------------------------------------------------
# Host shell portability helpers
# --------------------------------------------------------------------------
ifeq ($(OS),Windows_NT)
path_native = $(subst /,\,$1)
MKDIR_P = if not exist "$(call path_native,$1)" mkdir "$(call path_native,$1)"
RM_RF = if exist "$(call path_native,$1)" rmdir /S /Q "$(call path_native,$1)"
else
MKDIR_P = mkdir -p "$1"
RM_RF = rm -rf "$1"
endif

# --------------------------------------------------------------------------
# Sources (skeleton stage — extended in Phase 1)
# --------------------------------------------------------------------------
SRCS := \
	startup.c \
    main.c \
    boot/safe_mode.c \
	hal/platform.c \
	host/host_cfg.c \
	host/host_cfg_wire.c \
	host/host_cmd.c \
	host/host_stats.c \
	host/host_uart.c \
	radio/sx1276_airtime.c \
	radio/sx1276_cad.c \
	radio/sx1276_lbt.c \
	radio/sx1276_modes.c \
	radio/sx1276_rx.c \
	radio/sx1276_tx.c \
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

# Older arm-none-eabi ld (such as Arduino's 7-2017q4 toolchain) does not
# support --no-warn-rwx-segments. Enable this explicitly only on toolchains
# that support it.
LD_NO_WARN_RWX ?= 0
ifeq ($(LD_NO_WARN_RWX),1)
LDFLAG_NO_WARN_RWX := -Wl,--no-warn-rwx-segments
endif

LDFLAGS := \
	$(MCU_FLAGS) \
	-T$(LD_GEN) \
	-nostartfiles \
	-Wl,--gc-sections \
	$(LDFLAG_NO_WARN_RWX) \
	-Wl,-Map=$(BUILD)/$(TARGET).map \
	-specs=nano.specs \
	-specs=nosys.specs

# --------------------------------------------------------------------------
# Targets
# --------------------------------------------------------------------------
.PHONY: all hello lora_ping_tx lora_ping_rx lora_ping_tx_active lora_ping_rx_active clean size check check-cpp check-asserts check-opmode-owner check-cfg-contract check-cfg-contract-unit check-cfg-contract-wire check-cfg-wire-owner check-rx-counter-owner check-tx-counter-owner check-lbt-owner check-airtime-counter-owner help

all: $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex size

hello:
	$(MAKE) CROSS="$(CROSS)" TARGET=$(HELLO_TARGET) BUILD=$(HELLO_BUILD) SRCS="$(HELLO_SRCS)" all

lora_ping_tx:
	$(MAKE) CROSS="$(CROSS)" TARGET=lora_ping_tx BUILD=build_ping_tx SRCS="$(PING_SRCS)" EXTRA_CFLAGS="-DPING_ROLE=0" all

lora_ping_rx:
	$(MAKE) CROSS="$(CROSS)" TARGET=lora_ping_rx BUILD=build_ping_rx SRCS="$(PING_SRCS)" EXTRA_CFLAGS="-DPING_ROLE=1" all

lora_ping_tx_active:
	$(MAKE) CROSS="$(CROSS)" TARGET=lora_ping_tx_active BUILD=build_ping_tx_active SRCS="$(PING_SRCS)" EXTRA_CFLAGS="-DPING_ROLE=0 -DRADIO_ACTIVE_TEST_DEFAULT=1" all

lora_ping_rx_active:
	$(MAKE) CROSS="$(CROSS)" TARGET=lora_ping_rx_active BUILD=build_ping_rx_active SRCS="$(PING_SRCS)" EXTRA_CFLAGS="-DPING_ROLE=1 -DRADIO_ACTIVE_TEST_DEFAULT=1" all

help:
	@echo "Targets:"
	@echo "  all          — full build (requires arm-none-eabi-gcc)"
	@echo "  hello        — build minimal hello-world image for L072 bring-up"
	@echo "  lora_ping_tx — build LoRa TX pinger with minimized-on-boot RF default"
	@echo "  lora_ping_rx — build LoRa RX ponger with minimized-on-boot RF default"
	@echo "  lora_ping_tx_active — build opt-in active RF TX pinger for bench OTA tests"
	@echo "  lora_ping_rx_active — build opt-in active RF RX ponger for bench OTA tests"
	@echo "  size         — show region usage against the memory budget"
	@echo "  check        — preprocessor + memory-map static asserts only (no cross toolchain)"
	@echo "  clean        — remove build artifacts"

# Generated linker script — runs the C preprocessor on the .ld template
# so MM_* constants from include/memory_map.h expand inline.
$(LD_GEN): $(LD_SRC) include/memory_map.h | $(BUILD)
	@$(call MKDIR_P,$(dir $@))
	$(CPP) -P -nostdinc -undef -Iinclude -x c $< -o $@

# Object compilation
$(BUILD)/%.o: %.c | $(BUILD)
	@$(call MKDIR_P,$(dir $@))
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) -c $< -o $@

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
ifeq ($(OS),Windows_NT)
HOST_CC ?= gcc
else
HOST_CC ?= cc
endif
CFG_CONTRACT_BIN := $(BUILD)/cfg_contract
CFG_WIRE_BIN     := $(BUILD)/cfg_wire_vectors

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

# Optional CI guard: ensure RegOpMode ownership stays in radio/sx1276_modes.c.
ifeq ($(OS),Windows_NT)
check-opmode-owner:
	@echo "[CI] Checking OpMode ownership ..."
	@powershell -NoProfile -Command "$$hits = Select-String -Path 'radio/*.c' -Pattern 'SX1276_REG_OP_MODE' | Where-Object { ($$_.Path -replace '\\','/') -notlike '*/radio/sx1276_modes.c' }; if ($$hits) { Write-Output '[FAIL] Unexpected RegOpMode access outside radio/sx1276_modes.c'; $$hits | ForEach-Object { Write-Output $$_.ToString() }; exit 1 }; Write-Output '[OK] OpMode ownership check passed'"

check-cfg-wire-owner:
	@echo "[CI] Checking CFG URC payload ownership ..."
	@powershell -NoProfile -Command "$$hits = Select-String -Path 'host/*.c' -Pattern 'HOST_TYPE_CFG_OK_URC|HOST_TYPE_CFG_DATA_URC' | Where-Object { ($$_.Path -replace '\\','/') -notlike '*/host/host_cfg_wire.c' -and ($$_.Path -replace '\\','/') -notlike '*/host/host_cmd.c' }; if ($$hits) { Write-Output '[FAIL] Unexpected CFG URC emitters outside host_cfg_wire/host_cmd'; $$hits | ForEach-Object { Write-Output $$_.ToString() }; exit 1 }; Write-Output '[OK] CFG URC ownership check passed'"

check-rx-counter-owner:
	@echo "[CI] Checking RX counter ownership ..."
	@powershell -NoProfile -Command "$$hits = Select-String -Path 'radio/*.c','host/*.c','main.c' -Pattern 'host_stats_radio_rx_ok|host_stats_radio_crc_err' | Where-Object { ($$_.Path -replace '\\','/') -notlike '*/radio/sx1276_rx.c' -and ($$_.Path -replace '\\','/') -notlike '*/host/host_stats.c' }; if ($$hits) { Write-Output '[FAIL] RX counters must only be incremented in radio/sx1276_rx.c'; $$hits | ForEach-Object { Write-Output $$_.ToString() }; exit 1 }; Write-Output '[OK] RX counter ownership check passed'"

check-tx-counter-owner:
	@echo "[CI] Checking TX counter ownership ..."
	@powershell -NoProfile -Command "$$hits = Select-String -Path 'radio/*.c','host/*.c','main.c' -Pattern 'host_stats_radio_tx_ok' | Where-Object { ($$_.Path -replace '\\','/') -notlike '*/radio/sx1276_tx.c' -and ($$_.Path -replace '\\','/') -notlike '*/host/host_stats.c' }; if ($$hits) { Write-Output '[FAIL] TX counters must only be incremented in radio/sx1276_tx.c'; $$hits | ForEach-Object { Write-Output $$_.ToString() }; exit 1 }; Write-Output '[OK] TX counter ownership check passed'"

check-lbt-owner:
	@echo "[CI] Checking LBT abort counter ownership ..."
	@powershell -NoProfile -Command "$$hits = Select-String -Path 'radio/*.c','host/*.c','main.c' -Pattern 'host_stats_radio_tx_abort_lbt' | Where-Object { ($$_.Path -replace '\\','/') -notlike '*/radio/sx1276_lbt.c' -and ($$_.Path -replace '\\','/') -notlike '*/host/host_stats.c' }; if ($$hits) { Write-Output '[FAIL] LBT abort counter must only be incremented in radio/sx1276_lbt.c'; $$hits | ForEach-Object { Write-Output $$_.ToString() }; exit 1 }; Write-Output '[OK] LBT abort ownership check passed'"

check-airtime-counter-owner:
	@echo "[CI] Checking airtime abort counter ownership ..."
	@powershell -NoProfile -Command "$$hits = Select-String -Path 'radio/*.c','host/*.c','main.c' -Pattern 'host_stats_radio_tx_abort_airtime' | Where-Object { ($$_.Path -replace '\\','/') -notlike '*/radio/sx1276_tx.c' -and ($$_.Path -replace '\\','/') -notlike '*/host/host_stats.c' }; if ($$hits) { Write-Output '[FAIL] Airtime abort counter must only be incremented in radio/sx1276_tx.c'; $$hits | ForEach-Object { Write-Output $$_.ToString() }; exit 1 }; Write-Output '[OK] Airtime abort ownership check passed'"
else
check-opmode-owner:
	@echo "[CI] Checking OpMode ownership ..."
	@hits=$$(grep -RIn "SX1276_REG_OP_MODE" radio/*.c | grep -v "radio/sx1276_modes.c" || true); \
	if [ -n "$$hits" ]; then \
		echo "[FAIL] Unexpected RegOpMode access outside radio/sx1276_modes.c"; \
		echo "$$hits"; \
		exit 1; \
	fi; \
	echo "[OK] OpMode ownership check passed"

check-cfg-wire-owner:
	@echo "[CI] Checking CFG URC payload ownership ..."
	@hits=$$(grep -RIn "HOST_TYPE_CFG_OK_URC\|HOST_TYPE_CFG_DATA_URC" host/*.c | grep -v "host/host_cfg_wire.c\|host/host_cmd.c" || true); \
	if [ -n "$$hits" ]; then \
		echo "[FAIL] Unexpected CFG URC emitters outside host_cfg_wire/host_cmd"; \
		echo "$$hits"; \
		exit 1; \
	fi; \
	echo "[OK] CFG URC ownership check passed"

check-rx-counter-owner:
	@echo "[CI] Checking RX counter ownership ..."
	@hits=$$(grep -RIn "host_stats_radio_rx_ok\|host_stats_radio_crc_err" radio/ host/ main.c | grep -v "radio/sx1276_rx.c\|host/host_stats.c" || true); \
	if [ -n "$$hits" ]; then \
		echo "[FAIL] RX counters must only be incremented in radio/sx1276_rx.c"; \
		echo "$$hits"; \
		exit 1; \
	fi; \
	echo "[OK] RX counter ownership check passed"

check-tx-counter-owner:
	@echo "[CI] Checking TX counter ownership ..."
	@hits=$$(grep -RIn "host_stats_radio_tx_ok" radio/ host/ main.c | grep -v "radio/sx1276_tx.c\|host/host_stats.c" || true); \
	if [ -n "$$hits" ]; then \
		echo "[FAIL] TX counters must only be incremented in radio/sx1276_tx.c"; \
		echo "$$hits"; \
		exit 1; \
	fi; \
	echo "[OK] TX counter ownership check passed"

check-lbt-owner:
	@echo "[CI] Checking LBT abort counter ownership ..."
	@hits=$$(grep -RIn "host_stats_radio_tx_abort_lbt" radio/ host/ main.c | grep -v "radio/sx1276_lbt.c\|host/host_stats.c" || true); \
	if [ -n "$$hits" ]; then \
		echo "[FAIL] LBT abort counter must only be incremented in radio/sx1276_lbt.c"; \
		echo "$$hits"; \
		exit 1; \
	fi; \
	echo "[OK] LBT abort ownership check passed"

check-airtime-counter-owner:
	@echo "[CI] Checking airtime abort counter ownership ..."
	@hits=$$(grep -RIn "host_stats_radio_tx_abort_airtime" radio/ host/ main.c | grep -v "radio/sx1276_tx.c\|host/host_stats.c" || true); \
	if [ -n "$$hits" ]; then \
		echo "[FAIL] Airtime abort counter must only be incremented in radio/sx1276_tx.c"; \
		echo "$$hits"; \
		exit 1; \
	fi; \
	echo "[OK] Airtime abort ownership check passed"
endif

check-cfg-contract: check-cfg-contract-unit check-cfg-contract-wire

check-cfg-contract-unit: $(BUILD)
	@echo "[CFG] Building host contract test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/cfg_contract.c \
	    bench/host_proto/sx1276_stub.c \
	    host/host_cfg.c \
	    -o $(CFG_CONTRACT_BIN)
	@echo "[CFG] Running host contract test ..."
	@$(CFG_CONTRACT_BIN)

check-cfg-contract-wire: $(BUILD)
	@echo "[CFG] Building wire-vector test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/cfg_wire_vectors.c \
	    bench/host_proto/sx1276_stub.c \
	    host/host_cfg.c \
	    host/host_cfg_wire.c \
	    -o $(CFG_WIRE_BIN)
	@echo "[CFG] Running wire-vector test ..."
	@$(CFG_WIRE_BIN)

$(BUILD):
	@$(call MKDIR_P,$(BUILD))

clean:
	@$(call RM_RF,$(BUILD))

-include $(DEPS)
