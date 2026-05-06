# Method G Hardware Test Status (Copilot v1.5)

Date: 2026-05-05
Status: Additional testing completed on both X8 boards. Privileged capture path is confirmed usable, but all tested Linux serial endpoints remain zero-byte during timed reads, including synchronized reflash-capture on board A.
Scope: Delta update after v1.4 focused on deeper observability testing.

## 1. Delta from v1.4

This pass extends testing beyond permission checks and runs active capture attempts against both boards.

## 2. Test execution summary

Boards:
- Board A: `2D0A1209DABC240B` (`COM11`)
- Board B: `2E2C1209DABC240B` (`COM12`)

### 2.1 Connectivity and user context
- Both boards enumerate in `adb devices`.
- Board A `fio` context remains non-root and not directly allowed to open `/dev/ttyX0` without sudo.

### 2.2 Persistent group-permission attempt
Actions attempted on board A:
- `usermod -aG dialout fio`
- `usermod -aG tty fio`
- reboot and re-check

Observed result:
- Non-sudo read attempts still return `Permission denied` on `/dev/ttyX0`.

Interpretation:
- Persistent non-sudo access is not established in this image/session path.
- Sudo-based capture remains required for now.

### 2.3 Sudo capture path verification
Validated command family works:
- `printf 'fio\n' | sudo -S -p '' ...`

Root verification confirmed in earlier pass and remains valid for capture workflows.

### 2.4 Synchronized reflash-capture test (board A)
Goal:
- Catch startup bridge output during firmware restart.

Sequence:
1. Started bounded privileged capture on `/dev/ttyX0` (25 s window, hex output).
2. Reflashed board A with existing soft-bridge SerialRPC build artifact (`lifetrac_methodg_softbridge_serial2`).

Observed result:
- Reflash succeeded.
- Capture output contained no bytes.

### 2.5 Multi-endpoint timed byte-count sweep
Command pattern:
- Privileged timed reads (`timeout 2 cat | wc -c`) across:
  - `/dev/ttyX0`
  - `/dev/ttyGS0`
  - `/dev/ttymxc1`
  - `/dev/ttymxc2`
  - `/dev/ttymxc3`

Observed results:
- Board A: all `0`
- Board B: all `0`

## 3. Current interpretation

Closed in this pass:
1. Both-board endpoint sweep completed with consistent methodology.
2. Reflash-during-capture test executed on board A.
3. Sudo capture path remains operational.

Still open:
1. Any positive byte-level observability evidence from the software bridge path.
2. Confirmation of the true Linux-visible debug endpoint for M7 bridge output in this environment.

## 4. Updated blocker statement

At this point, the blocker is no longer command syntax or one-board variance.
The blocker is endpoint/route observability: all tested Linux serial nodes are silent under timed capture on both boards, even during synchronized reflash and after bridge-enabled firmware deployment.

## 5. Recommended next step (high-confidence)

Implement a temporary deterministic bridge heartbeat in firmware (compile-gated) on the debug side (SerialRPC path), then repeat the same `/dev/ttyX0` bounded capture. This provides a route-proof beacon independent of Murata traffic behavior.

Fallback if still silent:
- Move to physical route proof (`UART_SNIFF` / logic analyzer) as primary evidence path.

## 6. Practical command notes

- Use bounded capture windows for every run; avoid unbounded `cat` in evidence-oriented testing.
- On this image, assume sudo is required for `/dev/ttyX0` until a persistent group-permission change is demonstrated to survive login context and reboot.
