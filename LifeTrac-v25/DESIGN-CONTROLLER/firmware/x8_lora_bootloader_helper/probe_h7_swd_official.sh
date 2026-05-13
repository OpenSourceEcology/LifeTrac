#!/bin/sh
# Run the OFFICIAL Arduino openocd cfg in attach-only mode, verbose,
# to capture DPIDR read attempt and full openocd init output.
echo fio | sudo -S -p '' /usr/bin/openocd -d3 \
  -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
  -c "init; halt; reg pc; shutdown" 2>&1
