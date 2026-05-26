# LifeTrac Tractor X8 Camera/Image Pipeline Deploy

## Purpose
Runs the camera encoder pipeline on the tractor X8, publishing TileDeltaFrames to a local MQTT broker for the M7 firmware to pick up and send over LoRa to the base.

## Steps (bench, production path)

1. **Build the image**
   ```sh
   cd LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8
   docker build -t lifetrac-tractor-x8:latest .
   ```
2. **Deploy to tractor X8**
   - Tar up the tractor_x8 directory (excluding __pycache__, .logs, etc.)
   - `adb push` to `/tmp/lifetrac-tractor_x8.tgz`
   - On tractor:
     ```sh
     mkdir -p /opt/lifetrac/compose-apps/lifetrac-camera
     tar -xzf /tmp/lifetrac-tractor_x8.tgz -C /opt/lifetrac/compose-apps/lifetrac-camera --strip-components=1
     chown -R fio:fio /opt/lifetrac/compose-apps/lifetrac-camera
     ```
3. **Restart the systemd unit**
   ```sh
   sudo systemctl restart lifetrac-camera
   sudo systemctl status lifetrac-camera
   ```
4. **Verify**
   - `docker ps` should show `tractor-camera` and `tractor-mosquitto` running.
   - `docker logs tractor-camera` should show frame publish logs.
   - M7 firmware must be running and subscribed to `lifetrac/v25/cmd/image_frame`.
   - Base should start receiving image fragments (U_image > 0%).

## Notes
- `/dev/video1` is the Kurokesu C2 USB camera (default).
- `/dev/ttymxc3` is the UART to the M7 co-MCU.
- No LAN ports are exposed; all MQTT/image traffic is internal and routed through the M7/LoRa only.
- If you need to debug, you can override `LIFETRAC_MQTT_HOST` to point to the base for bench testing, but **production path is LoRa only**.
