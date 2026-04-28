# Operations Manual

> Operator-facing procedures for daily LifeTrac v25 use. **Not** an engineer's reference — see [BASE_STATION.md](BASE_STATION.md), [TRACTOR_NODE.md](TRACTOR_NODE.md), [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md) for that. This manual assumes the machine is already field-installed and calibrated.

## 1 — Power-on (cold start)

1. Open the **base-station enclosure**, flip the breaker. Wait ~45 s for the X8 Yocto image to boot. The base-station Wi-Fi SSID `lifetrac-<serial>` becomes visible when boot completes.
2. Walk to the **tractor**, key-on. The tractor controller boots in ~10 s; the cab status LED goes solid green when the LoRa link to base comes up.
3. **Do not** turn on the handheld yet — pairing happens before take-control.

## 2 — Pairing the handheld (first use only)

> Engineering safety: pairing **must** happen with the handheld physically connected to the base via USB-C. There is no over-the-air pairing path.

1. Plug the handheld into the base-station laptop with a USB-C cable.
2. The handheld OLED prints `PAIR READY — press MENU on base UI`.
3. Open `https://<base-ip>/admin/pair` on the base console. Enter the operator name (this gets logged with every command for the life of the pairing).
4. Press the highlighted **PAIR NOW** button. Watch the OLED — it counts down `PAIR 5… 4…`. While the countdown is running, **press and hold** the E-stop on the handheld. This proves physical possession.
5. After ~5 s the OLED prints `PAIRED ✓ <serial>`. The base UI confirms with a toast `Handheld paired`.
6. Unplug the USB cable. The handheld is now keyed; it will only talk to this base station until you re-pair.

Re-pair (operator change, lost handheld):

- Plug back in, click `Re-pair` (instead of `PAIR NOW`) on the admin page.
- The previous key is wiped from the handheld's secure element; the new key is generated and burned in. The audit log records both events.

## 3 — Take-control

1. Walk to the tractor with the handheld powered on. Stand at least 5 m clear of the implement path.
2. Press TAKE-CONTROL (the green button below the right joystick).
3. The handheld OLED briefly shows `REQ TAKE…` then `IN CONTROL` when the base has logged the takeover. The tractor cab LED turns from solid green → flashing green so anyone nearby can see remote control is active.
4. From this moment the handheld's heartbeat must arrive at the base every 200 ms. If three consecutive heartbeats are missed (≥600 ms gap), the tractor goes to a **safe stop** automatically.

## 4 — Operating

- **Left joystick** — drive (forward/back, left/right).
- **Right joystick** — loader (up/down, dump/curl).
- **Top-left button** — toggle aux circuit (e.g. snow blower PTO).
- **Top-right button** — toggle work-light.
- **Bottom button (red)** — E-stop. Latches; pull to release.

The base UI mirrors everything in real time: live camera tile, telemetry, status pill colour. **The UI is for situational awareness; it is not a control surface.** Never command motion from the browser.

## 5 — Returning control / shut-down

1. Park the tractor in a safe spot, loader on the ground, transmission to neutral.
2. Press TAKE-CONTROL again to release. OLED shows `RELEASED`. Cab LED → solid green.
3. Key-off the tractor.
4. Power off the handheld with a long-press of MENU.
5. The base station can stay up between sessions; only flip its breaker if you'll be away >24 hours.

## 6 — E-stop behaviour

| Trigger                              | What happens immediately                              | Recovery                                  |
|--------------------------------------|------------------------------------------------------|-------------------------------------------|
| Handheld E-stop pressed              | Tractor closes all valves, sets engine to idle       | Pull E-stop out → press CLEAR-ESTOP on base UI |
| Heartbeat lost ≥600 ms               | Same as above (this **is** an E-stop)                | Re-establish link, then CLEAR-ESTOP        |
| Base UI E-stop button                | Same as above                                         | CLEAR-ESTOP on base UI                     |
| Tractor-side fault (overspeed, etc.) | Same as above + audit-log entry with fault code      | Field-service required (see [FIELD_SERVICE.md](FIELD_SERVICE.md)) |

CLEAR-ESTOP **only** clears software latches; it does not bypass any pull-to-release physical E-stop on the handheld.

## 7 — Charging the handheld

- USB-C, any 5 V / 1 A or higher charger.
- Charge LED on TP4056: red = charging, blue = full.
- Full charge ~3 h. Run-time ~12 h continuous, ~36 h with screen-blanking on idle.
- Don't store the handheld on the charger long-term — pull when full.

## 8 — Daily safety check

Before first take-control of the day:

- [ ] Handheld E-stop physically tested (press, confirm tractor stops, release, confirm CLEAR-ESTOP works)
- [ ] Cab LED green
- [ ] Base UI status pill green
- [ ] Audit log time-stamped with today's date (`https://<base-ip>/audit` shows the most recent entry < 24 h old; if not, the X8 clock has drifted — call engineering)
- [ ] Loader hydraulics: command full lift then full lower at idle — no abnormal noise, no leak

## See also

- [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md) — engineering detail on the handheld
- [FIELD_SERVICE.md](FIELD_SERVICE.md) — when something is wrong
- [FIRMWARE_UPDATES.md](FIRMWARE_UPDATES.md) — when an update notice arrives
