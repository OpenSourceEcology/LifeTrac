# Development Defaults - Radio Minimizer and Power-Cycle Avoidance

**Date:** 2026-05-08
**Author:** GitHub Copilot
**Version:** 1.0
**Status:** Proposed development policy for all pre-release controller software and firmware

## 1. Purpose

This document defines the minimum default features that shall be present in all LifeTrac controller software, firmware, bench tools, and flashing workflows used during development, bring-up, testing, and field experiments before final release.

The policy goal is simple:

1. Minimize unnecessary LoRa transmissions and RF emissions whenever the system is powered but not actively testing or communicating.
2. Avoid manual power-cycles as a normal development or recovery step.

These protections shall remain enabled in every build and toolchain path used on real hardware until an explicit final-release decision disables them.

## 2. Policy Statement

For all non-final builds, the following two defaults are mandatory:

1. **LoRa radiation minimizer enabled by default**
2. **Power-cycle avoidance routine enabled by default**

"Non-final builds" includes:

1. Bring-up builds
2. Bench-test builds
3. Diagnostic builds
4. Development snapshots
5. HIL and integration builds
6. Release candidates that are still under validation

Only the final production release may disable these defaults, and only after an intentional sign-off step.

## 3. Mandatory Default Feature A - LoRa Radiation Minimizer

### 3.1 Required behavior

All pre-release firmware that can control the Murata CMWX1ZZABZ radio path shall boot into a low-emission state.

That low-emission state shall mean:

1. No periodic beaconing or background ping traffic
2. No continuous RX unless a test explicitly requires it
3. No TX unless a command, bench procedure, or test case explicitly enables it
4. Automatic return to the minimized state after the active test window ends

### 3.2 Required radio state

The Murata module's SX1276 supports reversible low-power mode transitions. In practice, the firmware shall use:

1. **Sleep** as the default idle state for maximum emission minimization
2. **Standby** as the pre-armed state when quick wake-up is needed
3. **RX/TX** only during explicitly enabled test activity

This means the answer to the design question is yes: the Murata radio path does have a sleep mode that can be toggled on and off through the SX1276 operating mode state machine.

### 3.3 Minimum implementation requirements

Every development firmware image that owns the radio shall provide all of the following:

1. A build-time default variable such as `RADIO_MINIMIZER_DEFAULT=1`
2. A runtime software control such as `radio_enable()` / `radio_disable()` or an equivalent state/policy setter
3. An inactivity timeout that returns the radio to Sleep or Standby after test traffic stops
4. A visible status indicator over UART, host protocol, logs, or stats so the operator can tell whether the radio is minimized or active
5. A default boot policy of `MINIMIZED`, not `ACTIVE`

### 3.4 Minimum prohibited behaviors in pre-release builds

The following behaviors shall not be present by default in development builds:

1. Automatic broadcast on boot
2. Continuous RX on boot without an explicit operator action
3. Background keepalive packets that exist only for convenience
4. Hidden test transmissions with no operator-visible indication

## 4. Mandatory Default Feature B - Power-Cycle Avoidance

### 4.1 Required behavior

All software and tooling used to flash, recover, test, or reset the controller stack shall default to a workflow that avoids manual unplug/replug cycles whenever software recovery is possible.

The current validated Method G workflow already provides the baseline:

1. `wdt_pet.sh start` for long operations that outlive the native 60-second watchdog window
2. `prep_bridge.sh` before H7 halt/openocd operations
3. `full_flash_pipeline.sh` as the standard wrapped flow
4. Automatic wait for reboot and ADB reattach instead of assuming human power removal

### 4.2 Minimum implementation requirements

All pre-release flashing and recovery paths shall preserve these protections:

1. Watchdog-pet support must remain available and enabled where required
2. Bridge-preparation steps must remain part of the normal flashing path
3. Recovery code shall prefer software reset, orderly restart, or timed reattach over manual power removal
4. New tools shall not introduce a workflow that depends on "just power-cycle it" as the first-line recovery path

### 4.3 Design intent

The power-cycle avoidance routine is not just a convenience feature. It is a development requirement because repeated manual power-cycling:

1. Slows test iteration
2. Increases operator error
3. Masks software recovery defects
4. Makes bench procedures harder to reproduce

## 5. Required Build Profiles

The project should standardize three policy-aware build profiles.

### 5.1 Development and bench profiles

`DEVELOPMENT`, `BENCH`, and `RELEASE_CANDIDATE` profiles shall all set:

1. `RADIO_MINIMIZER_DEFAULT=1`
2. `POWER_CYCLE_AVOIDANCE_DEFAULT=1`
3. `FINAL_RELEASE_BUILD=0`

### 5.2 Final release profile

`FINAL_RELEASE` may set:

1. `RADIO_MINIMIZER_DEFAULT=0` only if approved
2. `POWER_CYCLE_AVOIDANCE_DEFAULT=0` only if approved
3. `FINAL_RELEASE_BUILD=1`

The key rule is that disabling the guards must be explicit, intentional, and reviewable. It must never happen silently as a side effect of optimization or cleanup.

## 6. Minimum Feature Matrix by Software Layer

| Layer | Minimum required default features before final release |
| --- | --- |
| L072 radio firmware | Boot minimized, explicit enable/disable control, idle timeout back to Sleep or Standby, visible radio state reporting |
| H7 host firmware | Must not auto-command RF activity on boot, must expose operator control to arm/disarm radio tests |
| X8 helper scripts | Must use watchdog-aware and bridge-safe flashing flow, must prefer orderly restart over manual power-cycle |
| Bench tools and diagnostics | Must default to non-transmitting mode until the operator explicitly starts a test |
| Recovery tooling | Must treat power-cycling as fallback, not the normal first step |

## 7. Acceptance Gates for Future Software Sent to Hardware

Before any new software or firmware is loaded onto real controller hardware during development, it should satisfy this minimum checklist:

1. On boot, the radio is minimized by default
2. The operator can explicitly enable and disable RF activity
3. The radio returns to minimized state when idle or after timeout
4. The flashing or recovery path does not remove the watchdog-pet or bridge-prep safeguards
5. The build configuration clearly reports whether the two protections are enabled

## 8. Conditions Required Before Disabling the Defaults in Final Release

The two protections may be disabled only after an explicit release review confirms all of the following:

1. The radio's final always-on behavior is intentional and documented
2. Bench and field validation show that non-minimized default RF behavior is required
3. Recovery and flashing workflows are stable enough that removing the avoidance routines is acceptable
4. The release notes clearly document that the protections changed state from enabled to disabled

## 9. Recommended Naming Convention for Code

To keep the policy consistent across repositories, use names close to the following:

1. `RADIO_MINIMIZER_DEFAULT`
2. `POWER_CYCLE_AVOIDANCE_DEFAULT`
3. `FINAL_RELEASE_BUILD`
4. `radio_enable()`
5. `radio_disable()`
6. `radio_policy_set()`

The exact spelling can vary by component, but the intent must remain obvious in review.

## 10. Immediate Next Implementation Steps

This document implies the following near-term engineering work:

1. Add an explicit radio policy enum and default-minimized boot behavior in the L072 LoRa firmware
2. Add an inactivity timeout that forces the SX1276 back to Sleep or Standby
3. Keep the Method G watchdog-pet and bridge-prep flow as the standard flashing path
4. Add a release gate so the two defaults can only be disabled in a documented final-release build

## 11. Summary

Until final release, every software and firmware artifact sent to LifeTrac controller hardware should assume two things:

1. The radio should be quiet unless a test explicitly needs RF
2. The system should recover without requiring routine manual power-cycles

Those two defaults should be treated as baseline engineering discipline, not optional convenience features.