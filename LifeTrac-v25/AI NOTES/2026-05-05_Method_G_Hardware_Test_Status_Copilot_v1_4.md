# Method G Hardware Test Status (Copilot v1.4)

Date: 2026-05-05
Status: Linux permission blocker for `/dev/ttyX0` is now resolved (sudo path confirmed). UART observability is still not proven in a timed sample window (0 bytes captured in 3 seconds), so proof-of-traffic remains open.
Scope: Delta update after v1.3 focused on the ttyX0 permission fix and revised capture procedure.

## 1. Delta from v1.3

This update records execution of the privilege-elevation path and updates the error-fix guidance to separate:
1. Access failure (now fixed), from
2. Actual traffic evidence (still pending in this pass).

## 2. Permission-fix execution and results

### 2.1 User-provided command path
Command executed:
- `adb -s 2D0A1209DABC240B shell "echo fio | sudo -S stty -F /dev/ttyX0 115200 raw -echo && echo fio | sudo -S cat /dev/ttyX0"`

Behavior:
- Password prompt is expected on first sudo use.
- `cat /dev/ttyX0` is a continuous stream reader and will remain open until interrupted.

### 2.2 Validation commands run in this pass
Credential and privilege verification:
- `adb -s 2D0A1209DABC240B shell "echo fio | sudo -S id; echo fio | sudo -S ls -l /dev/ttyX0"`

Observed result:
- `uid=0(root) gid=0(root)`
- `/dev/ttyX0` confirmed as `root:dialout` ownership.

Timed capture check:
- `adb -s 2D0A1209DABC240B shell "echo fio | sudo -S sh -lc 'stty -F /dev/ttyX0 115200 raw -echo; timeout 3 cat /dev/ttyX0 | wc -c'"`

Observed result:
- `0`

Interpretation:
- The permission blocker is closed.
- Traffic evidence is not yet captured in the timed window.

## 3. Architecture interpretation (updated)

1. Host-side COM silence remains expected in this setup when observing Linux-routed paths:
- The USB-C host link is on the i.MX8 side, not a direct M7 USB path.
2. `ttyX0` remains the correct Linux-side endpoint to check when bridge output is routed through `SerialRPC`.
3. A permission fix alone does not guarantee bytes; it only enables the capture path.

## 4. Updated error-fix playbook

### 4.1 One-time privileged capture (recommended)
Use bounded reads to avoid indefinite blocking and to produce auditable output:

```bash
adb -s 2D0A1209DABC240B shell "echo fio | sudo -S sh -lc 'stty -F /dev/ttyX0 115200 raw -echo; timeout 5 cat /dev/ttyX0 | xxd -g 1 | head -n 40'"
```

### 4.2 Persistent permission fix (optional)

```bash
adb -s 2D0A1209DABC240B shell "echo fio | sudo -S usermod -aG dialout fio"
adb -s 2D0A1209DABC240B shell "echo fio | sudo -S usermod -aG tty fio"
adb -s 2D0A1209DABC240B shell "echo fio | sudo -S reboot"
```

After reboot, re-check with:

```bash
adb -s 2D0A1209DABC240B shell "id"
```

## 5. What is closed vs. open

Closed:
1. Software bridge implementation and build validation (from v1.3).
2. Flash path to board `2D0A1209DABC240B`.
3. Linux permission-access blocker for `/dev/ttyX0` (sudo path proven).

Open:
1. Positive byte-level observability evidence on `ttyX0` during an active traffic event.
2. Final route-proof artifact package for gate closure.

## 6. Immediate next actions

1. Ensure the actively flashed image uses the intended bridge debug route (`SerialRPC` path when validating `ttyX0`).
2. Generate a deterministic UART stimulus while capture is running (marker payload), then archive hex output.
3. If repeated bounded captures remain zero, switch to physical `UART_SNIFF`/logic analyzer capture for definitive route proof.

## 7. Practical notes

- `cat /dev/ttyX0` alone is not a pass/fail test; it is an open stream endpoint.
- Use bounded commands (`timeout`, byte counts, or hex dumps) so each run produces evidence suitable for the status log.
- The default sudo credential used in this session is `fio` for user `fio`.
