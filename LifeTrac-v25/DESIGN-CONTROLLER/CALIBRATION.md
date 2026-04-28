# Calibration

> Per [TODO.md § Operations](TODO.md). Procedures for calibrating every analog and positional input on the LifeTrac v25 stack. All steps assume a freshly flashed firmware image, the machine on jacks (wheels off the ground for the loader/auger steps), and the operator using the handheld in **bench mode** (E-stop wired but engine **off** until called for).

Each procedure produces values that get written to `config/calibration.toml` on the X8 and pushed to the H747 via `params_service.py`. Re-running a procedure is non-destructive — the file is keyed by sub-system.

## 1 — Joystick deadband + endpoints (handheld)

**When:** first power-up of a new handheld; whenever the operator notices drift while neutral or non-linearity at the rails.

**Tools:** USB-CDC console (`screen /dev/ttyACM0 115200` or VS Code serial monitor); the handheld must be powered from USB so the OLED is visible.

**Procedure:**
1. Hold MENU for 3 s; release. The OLED prints `CAL JOY 1/4 — release`. Both sticks must be **untouched** for ≥2 s while the firmware samples 256 readings.
2. The OLED prompts `CAL JOY 2/4 — LH up`. Push the left stick **fully up** and hold for 1 s.
3. Repeat for `LH down`, `LH left`, `LH right`, then the same four for RH.
4. Released-state mean ± 2σ becomes the deadband; min/max become the endpoints. The handheld writes them to its own EEPROM and sends a `params_set` command to the base over USB-CDC pairing channel — the base persists them in `config/calibration.toml [joystick.<serial>]`.

**Acceptance:** with both sticks at rest the per-axis report on the base UI must be 0 ±1 LSB for ≥30 s of observation.

## 2 — Flow-valve 0–10 V → GPM curve (tractor)

**When:** new Bürkert valve installed; new pressure transducer; suspected fluid contamination.

**Tools:** flow meter inline on the boom return (≥30 GPM scale), engine running at governor (typically 2200 rpm), reservoir at operating temperature (~50 °C).

**Procedure:**
1. From the base UI, open `Bench → Flow sweep`. The browser disables operator commands and the X8 takes ownership of the boom solenoid.
2. The X8 ramps the valve command from 0 → 100 % in 5 % steps, holding 10 s per step. At each step the operator records `(commanded %, measured GPM, line pressure)`.
3. Repeat for each axis (loader lift, loader curl, boom lift, boom curl, auxiliary).
4. Save the table in `config/calibration.toml [valves.<axis>]`. The runtime interpolates between calibration points.

**Acceptance:** measured GPM at any commanded % is within ±5 % of the linear-fit residual; if not, the valve has hysteresis and needs flushing or replacement.

## 3 — Pressure-sensor zero (tractor)

**When:** every 100 hours of operation; after any sensor swap.

**Tools:** none — the loader and boom must be **fully retracted with the engine off** so the lines are at relief pressure (typically 50–100 PSI of static load).

**Procedure:**
1. Engine off, key on. Open `Bench → Pressure zero` on the base UI.
2. The X8 samples the loader-pressure and boom-pressure channels for 10 s and writes the offset into `[pressure.<channel>] zero_psi` in `config/calibration.toml`.

**Acceptance:** with no flow commanded, the runtime-displayed pressure must read within ±25 PSI of zero on both channels.

## 4 — GPS antenna offset (tractor)

**When:** new GPS antenna mounted; antenna physically relocated.

**Tools:** measuring tape, plumb line.

**Procedure:**
1. Park the tractor on a known-flat surface. Measure the antenna's position relative to the **loader pin** (the front-most fixed point on the chassis):
   - `dx_m` — distance forward of the pin (positive = ahead of pin)
   - `dy_m` — distance left of pin (positive = to driver's left)
   - `dz_m` — distance above ground
2. Enter the three values in `config/calibration.toml [gps] antenna_offset_m`.
3. The autopilot uses these to translate antenna fixes to "loader-pin position" before computing path geometry.

**Acceptance:** at autopilot path-line follow speed, the loader pin must track the planned line within ±10 cm. If not, re-measure (most common error: forgetting to flip a sign).

## 5 — IMU bias (tractor)

**When:** every firmware update; suspected drift on the heading display.

**Procedure:** automatic — the H747 fuses gyro bias over the first 60 s of "tractor stationary" detected at boot. Operator just needs to leave the machine **completely still** for 60 s after key-on. The OLED prints `IMU CAL — wait` then `IMU CAL — OK`.

## See also

- [TRACTOR_NODE.md § Sensors](TRACTOR_NODE.md)
- [params_service.py](firmware/tractor_x8/params_service.py)
- [FIELD_SERVICE.md](FIELD_SERVICE.md) — what to do when calibration won't converge
