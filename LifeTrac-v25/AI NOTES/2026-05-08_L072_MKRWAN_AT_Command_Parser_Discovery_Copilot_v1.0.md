# L072 MKRWAN AT Command Handshake & Parser Discovery
**Date:** 2026-05-08
**Author:** GitHub Copilot
**Version:** 1.0

## Overview
This document covers the breakthrough in reverse-engineering the ST `mkrwan1300-fw` AT Command parser on the Portenta X8 (i.MX8 <-> M7 <-> L072). Previously, any attempt to communicate with the newly flashed MKRWAN reference firmware resulted in `+ERR_RX` and `Error when receiving` messages. This analysis explains the root causes and provides the script sequence to establish flawless logical UART handshaking.

## Root Cause Analysis

### 1. Two Stop Bits Required (19200 8N2)
By cloning and analyzing the Arduino/ST MKRWAN AT_Slave firmware repository (`mkrwan1300-fw/Projects/Multi/Applications/LoRa/AT_Slave/src/vcom.c`), it was discovered that the ST firmware explicitly hardcodes its `LPUART1` user application UART using `LL_LPUART_STOPBITS_2`. 
- **Standard Serial (8N1)** fails because the STM32L072 expects a secondary stop bit. When absent, it throws a hardware framing error.
- **Fix:** Pass `cstopb` (without a minus sign) to `stty` on the host side, locking the Linux UART to 2 stop bits.

### 2. Proprietary "+ERR_RX" Injection
When a hardware framing error or noise spike is detected upon UART interrupt, the L072 firmware forcefully queues a proprietary byte `AT_ERROR_RX_CHAR` (0x01) into its internal receive buffer. When the command parser evaluates the buffer, the presence of `0x01` trips `com_error(AT_RX_ERROR)`, which triggers the output strings `Error when receiving` and subsequently `+ERR_RX\r`. 

### 3. File Descriptor Toggling Glitches
Using standard bash redirection (`echo "AT\r" > /dev/ttymxc3`) causes the kernel to open, flush, and close the file descriptor around every transmission. This drops and restores the hardware link dynamically, causing electrical transients that the L072 interpreted as UART noise/framing errors.
- **Fix:** Keep the file descriptor persistently open using bash's `exec 3<> /dev/ttymxc3`.

## The Working Integration Sequence

The following bash sequence establishes a stable, continuous loop for sending AT commands and receiving successful `+OK\r` responses directly from the Linux shell, completely bypassing the M7 core (while it is halted via OpenOCD).

```bash
# 1. Configure the serial port for 19200 baud, 8 Data Bits, No Parity, TWO Stop Bits (cstopb)
# Hardware flow control MUST be disabled (-crtscts).
stty -F /dev/ttymxc3 19200 cs8 -parenb cstopb raw -echo -ixoff -crtscts -icanon -isig min 0 time 10

# 2. Halt the H7 core to prevent its UART TX from colliding on the shared PA_11/PA_9 bus
# (Requires the custom OpenOCD halt script detailed in previous notes)
nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f /home/fio/lifetrac_p0c/09_boot_user_app_hold.cfg > /home/fio/lifetrac_p0c/openocd_at.log 2>&1 &
OCD_PID=$!
sleep 2

# 3. Open File Descriptor 3 persistently to the UART to avoid close/re-open glitches
exec 3<> /dev/ttymxc3

# 4. Attach a background listener to FD 3
cat <&3 > /home/fio/lifetrac_p0c/at_rx.bin &
RX_PID=$!

# 5. Send an AT command through the persistent FD
echo -n -e "AT\r" >&3
sleep 1

# 6. Check outcomes and gracefully close
kill -9 $RX_PID 2>/dev/null
kill -9 $OCD_PID 2>/dev/null
xxd /home/fio/lifetrac_p0c/at_rx.bin
```

When properly framed using `cstopb` and the persistent file descriptor, the L072 flawlessly responds with the expected `+OK\r` immediately upon receiving an `AT\r` payload.
