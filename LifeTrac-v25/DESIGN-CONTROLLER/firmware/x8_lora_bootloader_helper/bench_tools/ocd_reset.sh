#!/bin/sh
# ocd_reset.sh -- runs ON THE TRACTOR, as root (it drives the i.MX GPIOs and
# openocd). The leg-start SWD reset of the tractor's L072: drop BOOT0 and pulse
# NRST through the H7 with openocd and 08_boot_user_app.cfg, in the background.
# run_live_radio_monitor.ps1 pushes it (LF-only) to /tmp/lifetrac_p0c next to
# the cfg and runs it once per leg; it is not meant to be run by hand.
#
#   sh ocd_reset.sh OCD_LEG=<UTC stamp>
#        export gpio8/10/15, drive gpio10 (H7 NRST) high, unexport them (a
#        freshly flashed X8 boots gpio10 LOW; FLASH_RUNBOOK), then start
#        openocd in the background and print "swd_reset_spawned" at once.
#
# Log /tmp/lifetrac_p0c/ocd_reset.log (truncated by every run; the harness
# reads it back at leg end and judges it, Get-TractorResetVerdict):
#   OCD_LEG=<UTC stamp>      first line, the tag the harness passed
#   <openocd output>         banner, Phase A-D lines of 08_boot_user_app.cfg
#   OCD_EXIT=<rc>            openocd's exit status; 128+N = killed by signal N
#
# Why a pushed script (bench kit, 2026-10-19): the reset used to be inline in
# the harness's adb/sudo `sh -c '...'`, as `( trap : HUP; ...; nohup openocd
# ...; echo OCD_EXIT=$? ) &`. A CAUGHT trap is reset to the default action in
# every exec'd child, so the adb/sudo session hangup that lands between the
# fork and nohup's own signal(SIGHUP, SIG_IGN) killed nohup: 2026-10-19 leg
# A2c2 logged only its tag and OCD_EXIT=129, no openocd banner. Here HUP is
# IGNORED (trap '' HUP) before anything is forked; an ignored disposition is
# inherited through fork and kept across exec, so the subshell and nohup
# start with HUP already ignored, and so does openocd until its own
# server_preinit installs a SIGHUP handler (after its banner). The window
# A2c2 died in, from the fork to nohup's signal(SIGHUP, SIG_IGN), is closed.
# `trap '' HUP` cannot be written inside the harness's single-quoted sh -c,
# hence this file.
#
# Test overrides (environment; sudo resets them on the board): OCD_DIR,
# OCD_GPIO, OCD_IFACE_CFG, OPENOCD, PKILL.
set -u

TAG=${1:-}
case $TAG in
  OCD_LEG=*) ;;
  *) echo "usage: sh $0 OCD_LEG=<UTC stamp>" >&2; exit 2 ;;
esac
DIR=${OCD_DIR:-/tmp/lifetrac_p0c}
GPIO=${OCD_GPIO:-/sys/class/gpio}
IFACE_CFG=${OCD_IFACE_CFG:-/usr/arduino/extra/openocd_script-imx_gpio.cfg}
OPENOCD=${OPENOCD:-openocd}
PKILL=${PKILL:-pkill}

# First, before any fork: ignored, not caught (see the header).
trap '' HUP

mkdir -p "$DIR"
# An openocd left by an earlier leg (an RS-5.2 hang) holds the SWD pins.
$PKILL -9 openocd 2>/dev/null
sleep 0.5
for g in 8 10 15; do
  [ -d "$GPIO/gpio$g" ] || { echo $g > "$GPIO/export"; } 2>/dev/null
done
{ echo out > "$GPIO/gpio10/direction"; } 2>/dev/null
{ echo 1 > "$GPIO/gpio10/value"; } 2>/dev/null
sleep 1
for g in 8 10 15; do
  { echo $g > "$GPIO/unexport"; } 2>/dev/null
done

( echo "$TAG"
  nohup "$OPENOCD" -f "$IFACE_CFG" -f "$DIR/08_boot_user_app.cfg"
  echo "OCD_EXIT=$?"
) > "$DIR/ocd_reset.log" 2>&1 < /dev/null &
echo swd_reset_spawned
