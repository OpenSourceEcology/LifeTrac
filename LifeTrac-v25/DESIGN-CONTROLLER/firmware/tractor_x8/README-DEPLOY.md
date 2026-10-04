# LifeTrac Tractor X8 Camera/Image Pipeline Deploy

## Purpose
Runs the camera encoder pipeline on the tractor X8, publishing TileDeltaFrames to a local MQTT broker for the M7 firmware to pick up and send over LoRa to the base.

## Steps (bench, production path)

1. **Build the image — for linux/arm64 — and get it onto the tractor**
   ```sh
   cd LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8
   # on an amd64 PC with Docker (buildx):
   docker buildx build --platform linux/arm64 -t lifetrac-tractor-x8:latest --load .
   # or natively on an aarch64 board with internet (for example the base X8):
   docker build -t lifetrac-tractor-x8:latest .
   docker save lifetrac-tractor-x8:latest | gzip > lifetrac-tractor-x8.tgz
   ```
   The compose file (`docker-compose.yml`) runs `image: lifetrac-tractor-x8:latest`
   and has no `build:`, so step 3 never builds: the image must already be on the
   tractor. A tractor without internet cannot build it (pip needs the numpy and
   OpenCV wheels), so copy the tarball to its disk (`/home/fio`, not the tmpfs
   `/tmp`), keep the old image (`docker tag lifetrac-tractor-x8:latest
   lifetrac-tractor-x8:pre-<change>`) and load the new one with
   `sudo docker load -i /home/fio/lifetrac-tractor-x8.tgz` (`-i`, not stdin:
   `sudo -S` would eat it).
2. **Deploy to tractor X8**
   - Tar up the tractor_x8 directory (excluding __pycache__, .logs, etc.)
   - `adb push` to `/tmp/lifetrac-tractor_x8.tgz`
   - On tractor:
     ```sh
     mkdir -p /opt/lifetrac/compose-apps/lifetrac-camera
     tar -xzf /tmp/lifetrac-tractor_x8.tgz -C /opt/lifetrac/compose-apps/lifetrac-camera --strip-components=1
     chown -R fio:fio /opt/lifetrac/compose-apps/lifetrac-camera
     ```
3. **Restart the systemd unit** (production only — on the radio bench
   `/dev/ttymxc3` is the L072 radio UART, and the compose file maps it as the
   M7 port; the bench harness stops this unit, see
   `x8_lora_bootloader_helper/bench_tools/RS13_VECTOR_LEG.md` step 0)
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
- VECTOR mode (encode mode 9, `x8_image_pipeline/encode_vector.py`) runs on numpy and `opencv-python-headless`, both in `requirements.txt` since Phase 1, so the image build pulls them (aarch64 wheels) and the deploy steps above are unchanged; the codec it needs is the in-tree mirror `x8_image_pipeline/vs1_codec.py`, not the base tree.
