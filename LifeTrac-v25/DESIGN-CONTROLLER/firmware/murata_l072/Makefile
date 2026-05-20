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
	host/host_cfg_profile.c \
	host/host_cfg_wire.c \
	host/host_cmd.c \
	host/host_rfco.c \
	host/host_rfco_summary.c \
	host/host_rfco_summary_emit.c \
	host/host_stats.c \
	host/host_uart.c \
	radio/sx1276_airtime.c \
	radio/sx1276_cad.c \
	radio/lora_pkt_hdr.c \
	radio/sx1276_fhss_chantab.c \
	radio/sx1276_fhss.c \
	radio/sx1276_lbt.c \
	radio/sx1276_legal_dwell.c \
	radio/sx1276_modes.c \
	radio/sx1276_rx.c \
	radio/sx1276_rx_counters.c \
	radio/sx1276_rx_retune_policy.c \
	radio/sx1276_rx_retune_counters.c \
	radio/sx1276_rx_scan_policy.c \
	radio/sx1276_rx_scan_counters.c \
	radio/sx1276_rx_scan_fail.c \
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
    -DLIFETRAC_FHSS_TX_ROUTED=1 \
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
.PHONY: all hello lora_ping_tx lora_ping_rx lora_ping_tx_active lora_ping_rx_active clean size check check-cpp check-asserts check-opmode-owner check-cfg-contract check-cfg-contract-unit check-cfg-contract-wire check-cfg-wire-owner check-rx-counter-owner check-tx-counter-owner check-lbt-owner check-airtime-counter-owner check-airtime-invariant check-fhss-scheduler check-legal-dwell check-rfco-pertx check-rfco-summary-counters check-rfco-summary check-rfco-summary-emit check-rfco-summary-integration check-cfg-profile check-lora-pkt-hdr check-rx-counters check-rx-retune-policy check-rx-retune-counters check-rx-scan-policy check-rx-scan-counters help

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
CFG_CONTRACT_BIN  := $(BUILD)/cfg_contract
CFG_WIRE_BIN      := $(BUILD)/cfg_wire_vectors
FHSS_CHANTAB_BIN   := $(BUILD)/fhss_chantab_vectors
FHSS_SCHEDULER_BIN := $(BUILD)/fhss_scheduler_vectors
LEGAL_DWELL_BIN    := $(BUILD)/legal_dwell
RFCO_PERTX_BIN     := $(BUILD)/rfco_pertx
RFCO_SUMMARY_COUNTERS_BIN := $(BUILD)/rfco_summary_counters
RFCO_SUMMARY_BIN   := $(BUILD)/rfco_summary
RFCO_SUMMARY_EMIT_BIN := $(BUILD)/rfco_summary_emit
RFCO_SUMMARY_INTEGRATION_BIN := $(BUILD)/rfco_summary_integration
CFG_PROFILE_BIN    := $(BUILD)/cfg_profile
LORA_PKT_HDR_BIN   := $(BUILD)/lora_pkt_hdr
RX_COUNTERS_BIN    := $(BUILD)/rx_counters
RX_RETUNE_POLICY_BIN := $(BUILD)/rx_retune_policy
RX_RETUNE_COUNTERS_BIN := $(BUILD)/rx_retune_counters
RX_SCAN_POLICY_BIN := $(BUILD)/rx_scan_policy
RX_SCAN_COUNTERS_BIN := $(BUILD)/rx_scan_counters
RX_SCAN_FAIL_BIN := $(BUILD)/rx_scan_fail
AIRTIME_INVARIANT_BIN := $(BUILD)/airtime_invariant

check: check-cpp check-asserts check-fhss-chantab check-fhss-scheduler check-airtime-invariant check-legal-dwell check-rfco-pertx check-rfco-summary-counters check-rfco-summary check-rfco-summary-emit check-rfco-summary-integration check-cfg-profile check-lora-pkt-hdr check-rx-counters check-rx-retune-policy check-rx-retune-counters check-rx-scan-policy check-rx-scan-counters check-rx-scan-fail
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

# FCC-FHSS-CHANTAB (implementation plan §14 delta #6).
# Compiles the firmware table source against a host toolchain so the
# _Static_asserts in radio/sx1276_fhss_chantab.c fire on bad edits, then
# runs golden vectors that are mirrored byte-for-byte in
# bench/host_proto/fhss_chantab.py.
check-fhss-chantab: $(BUILD)
	@echo "[FHSS] Building chantab golden-vector test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/fhss_chantab_vectors.c \
	    radio/sx1276_fhss_chantab.c \
	    -o $(FHSS_CHANTAB_BIN)
	@echo "[FHSS] Running chantab golden-vector test ..."
	@$(FHSS_CHANTAB_BIN)

# FCC-A1a (plan §14.2 step 3): config-time airtime invariant.
# Builds the pure ToA helper + invariant check from radio/sx1276_airtime.c
# against the host toolchain and runs golden vectors that prove
#   (a) SF12/BW125/255B is rejected (ToA > 380 ms),
#   (b) SF7/BW250/255B is accepted (ToA = 199.808 ms),
#   (c) the pure ToA helper matches the Semtech datasheet formula.
check-airtime-invariant: $(BUILD)
	@echo "[AIRTIME] Building airtime-invariant test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/airtime_invariant.c \
	    radio/sx1276_airtime.c \
	    -o $(AIRTIME_INVARIANT_BIN)
	@echo "[AIRTIME] Running airtime-invariant test ..."
	@$(AIRTIME_INVARIANT_BIN)

# FCC-A3 (plan §14.2 step 5): FHSS hop scheduler.
# Compiles radio/sx1276_fhss.c + radio/sx1276_fhss_chantab.c against the
# host toolchain and runs golden vectors that prove:
#   - FNV-1a 32-bit seed matches off-tree reference values,
#   - Fisher-Yates / xorshift32 permutation matches reference vectors,
#   - permutation is a bijection of 0..49,
#   - one full epoch via next_channel() visits every channel exactly once,
#   - blacklist is refused during cold-start warm-up (50 hops) and after
#     warm-up the 50-channel legal floor still refuses it,
#   - LBT block accounting is decoupled from the active set,
#   - APIs fail closed (NOT_INIT / NULL_ARG / BAD_IDX) without mutating state.
check-fhss-scheduler: $(BUILD)
	@echo "[FHSS] Building scheduler golden-vector test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/fhss_scheduler_vectors.c \
	    radio/sx1276_fhss.c \
	    radio/sx1276_fhss_chantab.c \
	    -o $(FHSS_SCHEDULER_BIN)
	@echo "[FHSS] Running scheduler golden-vector test ..."
	@$(FHSS_SCHEDULER_BIN)

# FCC-A2 (plan §14.2 step 6): legal-dwell accountant.
# Compiles radio/sx1276_legal_dwell.c against the host toolchain and
# runs property tests covering: half-open window inclusion, pessimistic
# reserve + monotonic-downward reconcile, no-rollback-on-NACK contract,
# per-channel isolation across 64 channels, 10 s vs 20 s window split,
# over-budget refusal, ring-full eviction path, stale-handle safety,
# and fail-closed bad-input rejection.
check-legal-dwell: $(BUILD)
	@echo "[DWELL] Building legal-dwell test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/legal_dwell.c \
	    radio/sx1276_legal_dwell.c \
	    -o $(LEGAL_DWELL_BIN)
	@echo "[DWELL] Running legal-dwell test ..."
	@$(LEGAL_DWELL_BIN)

# FCC-B1-PERTX (plan §14.2 step 7): per-TX RFCO URC packer + emit().
# Compiles host/host_rfco.c against the host toolchain with a stub for
# host_uart_send_urc(), and runs golden-vector + property tests that
# pin the on-wire payload layout (21 bytes, little-endian), the
# schema_ver byte (= 1), the URC type (= 0xC3), the status enum
# sentinels, and that emit() forwards the correct (type, payload, len)
# triple. Bench post-processing depends on these constants.
check-rfco-pertx: $(BUILD)
	@echo "[RFCO ] Building per-TX RFCO test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rfco_pertx.c \
	    host/host_rfco.c \
	    radio/sx1276_fhss.c \
	    radio/sx1276_fhss_chantab.c \
	    -o $(RFCO_PERTX_BIN)
	@echo "[RFCO ] Running per-TX RFCO test ..."
	@$(RFCO_PERTX_BIN)

# FCC-B1-SUMMARY-b-1 (plan §14.2 step 11, sub-step 1 of 3): sidecar
# counter TUs that feed the per-minute RFCO_SUMMARY URC payload.
# Compiles three counter-bearing modules (host_rfco, sx1276_fhss,
# sx1276_legal_dwell) against the host toolchain and runs property
# tests covering: saturating increment, snapshot-and-clear semantics,
# NULL-safe out, out-of-range guards, per-channel/per-slot independence,
# OK-not-recorded, INTERNAL->slot 7 mapping. Pins behavior that the
# SUMMARY pack helper (b-2) and call-site wiring (b-3) will rely on.
check-rfco-summary-counters: $(BUILD)
	@echo "[SUMC ] Building RFCO summary counters test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rfco_summary_counters.c \
	    host/host_rfco.c \
	    radio/sx1276_fhss.c \
	    radio/sx1276_fhss_chantab.c \
	    radio/sx1276_legal_dwell.c \
	    -o $(RFCO_SUMMARY_COUNTERS_BIN)
	@echo "[SUMC ] Running RFCO summary counters test ..."
	@$(RFCO_SUMMARY_COUNTERS_BIN)

# FCC-B1-SUMMARY-b-2 (plan §14.2 step 11, sub-step 2 of 3): pure pack
# helper for the per-minute RFCO_SUMMARY URC (191-byte LE payload +
# trailing CRC-16/CCITT-FALSE). Compiles host/host_rfco_summary.c
# against the host toolchain and runs byte-by-byte golden vectors that
# pin: (a) the 191-byte payload length, (b) the schema byte hard-pin
# at offset 0, (c) the _reserved_align hard-zero at offsets 6..7, (d)
# LE encoding of every u16/u32 field, (e) per-channel/per-slot array
# layout (50 hop + 50 dwell + 8 blocked), (f) CRC-16/CCITT-FALSE
# correctness (KAT + sensitivity + LE byte order). NO call-site wiring
# — that lands as B1-SUMMARY-b-3.
check-rfco-summary: $(BUILD)
	@echo "[SUMP ] Building RFCO summary pack test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rfco_summary.c \
	    host/host_rfco_summary.c \
	    -o $(RFCO_SUMMARY_BIN)
	@echo "[SUMP ] Running RFCO summary pack test ..."
	@$(RFCO_SUMMARY_BIN)

# FCC-B1-SUMMARY-c-1 (plan §14.2 step 11, sub-step c-1 of c-1/c-2):
# emit wrapper + cadence helper + per-window sidecars (no main-loop
# wiring — that lands as c-2). Compiles host/host_rfco_summary.c
# alongside the sidecar sources (host_rfco, sx1276_fhss,
# sx1276_legal_dwell) against the host toolchain with a stub for
# host_uart_send_urc(), and runs property tests covering:
# (a) emit() forwards (type=0xC4, seq lower 16, flags, payload, len=191)
# to send_urc; (b) payload matches an independent re-pack via
# host_rfco_summary_pack(); (c) FIRST_SINCE_BOOT bit set on first call,
# cleared on the second; (d) summary_emit_count increments 0→1→2;
# (e) all sidecars (hop_count, dwell_peak, blocked_attempts,
# pertx_count_in_window, last_clamp_reason) are drained by emit();
# (f) pertx_count_in_window reflects intervening host_rfco_pertx_emit()
# calls and saturates at 0xFF; (g) last_clamp_reason captures the
# most-recent non-OK status, stays 0xFF when only OKs; (h) µs→ms
# ceiling-div conversion (0→0, 1µs→1ms, 1500µs→2ms, UINT32_MAX→0xFFFF);
# (i) should_emit cadence at exactly 60000 ms boundary + u32 wraparound.
check-rfco-summary-emit: $(BUILD)
	@echo "[SUME ] Building RFCO summary emit test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rfco_summary_emit.c \
	    host/host_rfco_summary.c \
	    host/host_rfco_summary_emit.c \
	    host/host_rfco.c \
	    radio/sx1276_fhss.c \
	    radio/sx1276_fhss_chantab.c \
	    radio/sx1276_legal_dwell.c \
	    -o $(RFCO_SUMMARY_EMIT_BIN)
	@echo "[SUME ] Running RFCO summary emit test ..."
	@$(RFCO_SUMMARY_EMIT_BIN)

# FCC-B1-SUMMARY-c-2 (plan §14.2 step 11, sub-step c-2 of c-1/c-2):
# main-loop integration of the summary emit. Compiles
# bench/host_proto/rfco_summary_integration.c against the same emit
# wrapper + sidecar TUs as c-1 (now exercising the new
# host_rfco_summary_tick() helper that owns last_emit_ms + seq) and
# drives a simulated clock to verify:
# (a) tick() emits nothing before now_ms reaches the first 60 000 ms
# boundary; (b) at exactly 60 000 ms the first emit fires, carries
# FIRST_SINCE_BOOT, summary_seq == 0, window_elapsed_ms == 60000;
# (c) tick() is idempotent within a window (no double-emit at the
# boundary); (d) successive windows emit at 120 000, 180 000, ... with
# monotonically advancing summary_seq and window_elapsed_ms reflecting
# the actual delta (e.g. late tick at 121 500 reports window=61 500);
# (e) FIRST_SINCE_BOOT flag is cleared on the second emit;
# (f) reset_wrapper_state() restores the 60 s startup delay and zeros
# both seq and last_emit_ms; (g) profile_id arg threads through to the
# SUMMARY payload byte 2 unchanged; (h) cadence survives a u32
# platform_now_ms() wraparound near 0xFFFFFFFF.
check-rfco-summary-integration: $(BUILD)
	@echo "[SUMI ] Building RFCO summary integration test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rfco_summary_integration.c \
	    host/host_rfco_summary.c \
	    host/host_rfco_summary_emit.c \
	    host/host_rfco.c \
	    radio/sx1276_fhss.c \
	    radio/sx1276_fhss_chantab.c \
	    radio/sx1276_legal_dwell.c \
	    -o $(RFCO_SUMMARY_INTEGRATION_BIN)
	@echo "[SUMI ] Running RFCO summary integration test ..."
	@$(RFCO_SUMMARY_INTEGRATION_BIN)

# FCC-A5 (plan §14.2 step 8): regulatory-profile validation + two-phase
# commit. Compiles host/host_cfg_profile.c against the host toolchain
# and runs 25 property tests covering: validator (profile id, FHSS mask
# popcount + table-membership, BW match, DTS, antenna range, unrouted
# gate, NULL arg), power-clamp arithmetic (gain<=6 no-op, gain>6 ERP
# reduction, hw-ceiling clamp, negative gain, no-headroom floor), tier
# ceiling lookup, and the stage/activate state machine (happy path,
# activate-without-stage, failed-stage clears in-flight, cancel, double
# activate, unrouted-leaves-active-alone).
check-cfg-profile: $(BUILD)
	@echo "[CFGP ] Building cfg-profile test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/cfg_profile.c \
	    host/host_cfg_profile.c \
	    -o $(CFG_PROFILE_BIN)
	@echo "[CFGP ] Running cfg-profile test ..."
	@$(CFG_PROFILE_BIN)

# FCC-A6a (plan §14.2 step 10, delta #13): LoRa link-layer packet header
# pack/unpack contract. Compiles radio/lora_pkt_hdr.c against the host
# toolchain and runs golden-vector + property tests pinning the on-wire
# layout (8 bytes, little-endian epoch), the schema_ver byte (= 1), the
# header length constant, NULL/short/bad-schema rejection paths, the
# reserved-byte ignore rule (additive-evolution friendly per FCC notes
# §17.3 #4), and pack/unpack round-trip integrity. Required so both
# TX (sx1276_tx.c) and RX (sx1276_rx.c) agree on hop-sync header
# layout before A6b (RX retune) and A6c (Scanning state) land.
check-lora-pkt-hdr: $(BUILD)
	@echo "[HDR  ] Building lora-pkt-hdr test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/lora_pkt_hdr.c \
	    radio/lora_pkt_hdr.c \
	    -o $(LORA_PKT_HDR_BIN)
	@echo "[HDR  ] Running lora-pkt-hdr test ..."
	@$(LORA_PKT_HDR_BIN)

# FCC-A6b-2-ii-β (plan §14.2 step 10): per-decision counters that
# sx1276_rx_service() updates via sx1276_rx_counter_record() after each
# successful A6a header parse. Compiles radio/sx1276_rx_counters.c
# (HW-free TU) against the host toolchain and verifies initial-zero,
# per-slot increment isolation, out-of-range no-op, snapshot
# idempotency, reset round-trip, NULL-arg safety, no-narrow-type-wrap,
# interleaved-accumulation, and the DIM-matches-enum invariant. Drop
# visibility for FCC-B1-SUMMARY rides on these counters — no
# host_stats field added (would bump the host↔X8 wire layout).
check-rx-counters: $(BUILD)
	@echo "[RXCT ] Building rx-counters test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rx_counters.c \
	    radio/sx1276_rx_counters.c \
	    -o $(RX_COUNTERS_BIN)
	@echo "[RXCT ] Running rx-counters test ..."
	@$(RX_COUNTERS_BIN)

# FCC-A6b-2-ii-γ-1 (plan §14.2 step 10): pure-function RX retune
# decision. Verifies priority ordering (NOT_INIT > BUSY > timing),
# strict-< period boundary, wrap-safe int32 elapsed math (incl. real
# uint32 tick wrap), DO_WRAP self-heal branch, DIM = max-enum + 1, and
# enum numeric stability (FCC-B1-SUMMARY's additive URC will index
# per-outcome counters by these values — reordering would silently
# mis-route histograms). γ-2 wires this policy + a HW-free counter TU
# into sx1276_rx_tick(); γ-3 replaces the placeholder period with the
# Q2 epoch-driven boundary.
check-rx-retune-policy: $(BUILD)
	@echo "[RXRT ] Building rx-retune-policy test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rx_retune_policy.c \
	    radio/sx1276_rx_retune_policy.c \
	    -o $(RX_RETUNE_POLICY_BIN)
	@echo "[RXRT ] Running rx-retune-policy test ..."
	@$(RX_RETUNE_POLICY_BIN)

# FCC-A6b-2-ii-γ-2: per-outcome counters for the γ-1 retune policy.
# HW-free TU (radio/sx1276_rx_retune_counters.c) — host links and runs
# without modem/platform deps. The 10-case test verifies: zeroed
# initial state, per-slot isolation, out-of-range no-op, idempotent
# snapshot, reset + reuse, NULL-safe getter, no narrow-type wrap @
# 4096 records, interleaved accumulation, DIM = max-enum + 1, and
# round-trip after reset. FCC-B1-SUMMARY will read these counters
# additively into the per-minute URC.
check-rx-retune-counters: $(BUILD)
	@echo "[RXRTC] Building rx-retune-counters test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rx_retune_counters.c \
	    radio/sx1276_rx_retune_counters.c \
	    -o $(RX_RETUNE_COUNTERS_BIN)
	@echo "[RXRTC] Running rx-retune-counters test ..."
	@$(RX_RETUNE_COUNTERS_BIN)

# FCC-A6c-1: RX cold-start Scanning state-machine pure-function
# policy (radio/sx1276_rx_scan_policy.c). 20-case host golden vector
# verifies: NULL-safe default, BOOT-collapses-all-events to
# BEGIN_SCAN, SCANNING dwell-boundary + redesign-boundary precedence
# (FAIL wins over ADVANCE), FRAME_VALID-locks-only-with-header-ok
# defensive demotion, LOCKED+FAILED absorbing, uint32 ms tick wrap
# on either anchor -> REANCHOR, DIM = max-enum + 1 for
# state/event/action, and enum numeric stability (FCC-B1-SUMMARY
# URC will index per-state / per-action counters by these values --
# reordering would silently mis-route histograms). A6c-2 wires this
# policy + a HW-free counter TU into the RX loop; A6c-3 surfaces
# 5 s acceptance-target overruns to D-Gate.
check-rx-scan-policy: $(BUILD)
	@echo "[RXSCN] Building rx-scan-policy test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rx_scan_policy.c \
	    radio/sx1276_rx_scan_policy.c \
	    -o $(RX_SCAN_POLICY_BIN)
	@echo "[RXSCN] Running rx-scan-policy test ..."
	@$(RX_SCAN_POLICY_BIN)

# FCC-A6c-2 (counters TU): per-state + per-action histograms for
# the A6c-1 Scanning SM. HW-free TU (radio/sx1276_rx_scan_counters.c)
# mirrors the γ-2 retune-counters pattern but spans two axes
# (state DIM=4 + action DIM=6). The 14-case test verifies: zeroed
# initial state, per-axis slot isolation in lockstep with the
# orthogonal axis, OOR-state drops state but keeps action, OOR-
# action drops action but keeps state, both-OOR total no-op,
# idempotent snapshots, reset zeros BOTH arrays, NULL-safe getters
# on either axis, no narrow-type wrap @ 4096 records, interleaved
# accumulation, STATE_DIM/ACTION_DIM = max-enum + 1, and round-
# trip after reset. FCC-B1-SUMMARY will read both histograms
# additively into the per-minute URC.
check-rx-scan-counters: $(BUILD)
	@echo "[RXSCC] Building rx-scan-counters test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rx_scan_counters.c \
	    radio/sx1276_rx_scan_counters.c \
	    -o $(RX_SCAN_COUNTERS_BIN)
	@echo "[RXSCC] Running rx-scan-counters test ..."
	@$(RX_SCAN_COUNTERS_BIN)

# FCC-A6c-3-c: pure helper that builds the HOST_FAULT_CODE_RX_SCAN_
# FAILED sub-byte + the retry-vs-final gate from the A6c-3-b file-
# static snapshot. HW-free TU (radio/sx1276_rx_scan_fail.c) — host
# links and runs without modem/platform/host_cmd deps. Test pins
# the §13.1 #2 bit layout (final=bit7, warm=bit6, hw_suspect=bit5,
# crc_seen=bit4, attempt=bits3..0), the retry boundary
# (retries==max ⇒ final), the attempt-nibble saturation, the
# hw_suspect derivation (NOT got_irq AND NOT crc_seen), the
# warm-bit passthrough, the NULL-input defensive contract
# (sub=0x80, final=1), and the constant SX1276_RX_SCAN_MAX_RETRIES
# = 3U from v5.0 §13.1.
check-rx-scan-fail: $(BUILD)
	@echo "[RXSCF] Building rx-scan-fail test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/rx_scan_fail.c \
	    radio/sx1276_rx_scan_fail.c \
	    -o $(RX_SCAN_FAIL_BIN)
	@echo "[RXSCF] Running rx-scan-fail test ..."
	@$(RX_SCAN_FAIL_BIN)

$(BUILD):
	@$(call MKDIR_P,$(BUILD))

clean:
	@$(call RM_RF,$(BUILD))

-include $(DEPS)
