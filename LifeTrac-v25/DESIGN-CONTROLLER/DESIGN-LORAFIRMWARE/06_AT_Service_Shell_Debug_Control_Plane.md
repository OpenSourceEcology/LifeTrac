# Method G AT Service Shell Debug Control Plane

Date: 2026-05-06
Status: Accepted for bench/debug implementation

## 1. Decision

Method G keeps the COBS-framed binary host protocol as the primary runtime interface, and adds a small AT-style ASCII service shell on the same host UART as a bench, diagnostic, and recovery aid.

The service shell is not the production control protocol. It is a human-readable secondary surface that proves the custom Murata/L072 firmware is alive and can report useful state before the full binary host stack or X8-side tooling is trusted.

## 2. Goals

The AT shell exists to answer simple bench questions quickly:

- Is the L072 custom firmware booted?
- Which firmware version and git placeholder are running?
- Did the SX1276 initialize, and what version byte was observed?
- Are host UART and radio counters moving?
- Can the host deliberately return to the binary Method G protocol?

## 3. Non-goals

The AT shell does not replace Method G binary framing for runtime traffic.

It does not carry normal control frames, telemetry streams, crypto payloads, or high-rate LoRa packets. Those remain owned by the COBS-framed binary protocol because it provides compact packets, CRC coverage, unambiguous payload length, and deterministic parsing.

## 4. Wire behavior

The host UART parser recognizes AT lines opportunistically at a binary frame boundary:

1. Binary Method G frames remain COBS-delimited with `0x00` delimiters.
2. ASCII service commands are newline-terminated text beginning with `AT`.
3. If an apparent AT candidate turns out not to be an ASCII line, the bytes are returned to the binary parser.
4. AT responses are plain ASCII lines ending in `\r\n`.

This keeps binary mode always available. `AT+BIN` is therefore a confirmation command rather than a persistent mode switch.

## 5. Initial command set

| Command | Response | Purpose |
| --- | --- | --- |
| `AT` | `OK` | Liveness check |
| `ATI` | firmware identity, then `OK` | Human-readable identity |
| `AT+VER?` | firmware/protocol/capability details, then `OK` | Version audit |
| `AT+RADIO?` | radio init/version/state, then `OK` | SX1276 bring-up proof |
| `AT+STAT?` | host/radio counters, then `OK` | Bench diagnostics |
| `AT+HELP` / `AT+HELP?` | command list, then `OK` | Discoverability |
| `AT+BIN` | `OK BIN` | Explicit return to binary host tooling |

Unknown or malformed AT commands return `ERROR`.

## 6. Build control

The shell is compile-gated with `HOST_AT_SHELL_ENABLE` in `firmware/murata_l072/config.h`.

Bench and early Method G bring-up builds enable it by default. Hardened production builds may disable it without changing the binary protocol.

## 7. Safety rules

The first implementation is read-mostly. It intentionally excludes reset, register write, and transmit commands so the AT shell cannot accidentally alter RF state during early bench testing.

Future additions may include guarded commands such as `AT+RESET`, `AT+TXHEX=...`, or `AT+REG?`, but those should be separate decisions with explicit production-disable behavior.

## 8. Bench usage

Example interaction:

```text
AT
OK

ATI
OSE-LifeTracLORA-MurataFW 0.0.0 dev
OK

AT+RADIO?
RADIO=OK VERSION=0x12 STATE=1
OK

AT+STAT?
HOST_DROPPED=0 HOST_ERRORS=0 HOST_QUEUE_FULL=0
RADIO_RX_OK=0 RADIO_TX_OK=0 RADIO_CRC_ERR=0 STATE=1
OK
```

## 9. Validation plan

Firmware validation should include:

1. Cross-build the L072 firmware with `HOST_AT_SHELL_ENABLE=1`.
2. Confirm no size-budget regression beyond acceptable launch headroom.
3. Bench-send `AT`, `ATI`, `AT+VER?`, `AT+RADIO?`, `AT+STAT?`, `AT+HELP`, and `AT+BIN` over the selected host UART path.
4. Confirm binary COBS frames still parse after AT commands.
5. Confirm production builds can compile with `HOST_AT_SHELL_ENABLE=0`.