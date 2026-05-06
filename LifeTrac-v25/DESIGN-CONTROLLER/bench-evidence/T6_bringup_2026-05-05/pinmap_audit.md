# T6 Pinmap Audit

Date: 2026-05-05
Scope: Gate 3 pin conflict audit for Method G UART selection

## Inputs

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino`
- `C:/Users/dorkm/AppData/Local/Arduino15/packages/arduino/hardware/mbed_portenta/4.5.0/variants/PORTENTA_H7_M7/pins_arduino.h`
- `C:/Users/dorkm/AppData/Local/Arduino15/packages/arduino/hardware/mbed_portenta/4.5.0/variants/PORTENTA_X8/pins_arduino.h`
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md`

## Runtime pin use observed in `tractor_h7.ino`

- `Serial1.begin(921600)` used for X8<->H747 link in legacy path.
- RS-485 configured with `RS485.setPins(PA_9, PA_10, PB_3)`.
- Method G runtime path is compile-time exclusive when enabled.

## Candidate audit table

| Candidate | Core mapping visibility | Known conflicts | Result |
|---|---|---|---|
| `Serial1` | Present in H7/X8 variants | Explicitly reserved for X8<->H747 link | Rejected |
| `Serial2` | Present in H7 variant (`PA_15/PF_6/PF_8/PF_9`) | No direct conflict with RS-485 pins (`PA_9/PA_10/PB_3`) in current sketch | Preferred desk candidate |
| `Serial3` | Present in H7 variant (`PJ_8/PJ_9`) | Reserved by Max Carrier cellular modem path per runbook | Rejected |
| `Serial4` | Not present in local stock X8/H7 variant headers | Cannot verify from installed core pin headers | Pending patched-core/board trace evidence |
| `Serial5` | Not present in local stock X8/H7 variant headers | Cannot verify from installed core pin headers | Pending patched-core/board trace evidence |

## Conclusion

- `Serial2` is the best available candidate from local pinmap data and current sketch usage.
- `Serial4`/`Serial5` remain unresolved in this workstation's installed stock core metadata.

## Residual risk

- Production X8 build may require a patched core package that exposes the selected serial route consistently with Max Carrier wiring.
- Final closure requires physical board trace/sniff evidence and successful bench traffic on the chosen route.
