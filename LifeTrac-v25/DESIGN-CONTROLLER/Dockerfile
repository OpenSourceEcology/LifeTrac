# LifeTrac v25 base-station container.
#
# Builds a single image that can be launched with multiple commands
# (lora_bridge, web_ui, audit-tail) via docker-compose. We ship one image
# instead of three because the Python deps overlap heavily and the X8's
# eMMC has limited headroom.
#
# Tractor X8 also reuses this image for its own systemd-managed services
# (time/params/logger) — the firmware/tractor_x8 sources are mounted in
# from the host so the image stays node-agnostic.

# PYTHON_IMAGE is overridable only to pin the flown base image by digest
# (see base_station/requirements.lock.txt). The default is unchanged.
ARG PYTHON_IMAGE=python:3.11-slim
FROM ${PYTHON_IMAGE}

# Mosquitto client + a few diagnostic tools so an operator shell-in is
# useful (mosquitto_pub/sub for poking topics by hand).
RUN apt-get update && apt-get install -y --no-install-recommends \
        mosquitto-clients \
        ca-certificates \
        libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Layer the deps separately so source edits do not invalidate the pip cache.
#
# Optional LOCKED=1 (`docker build --build-arg LOCKED=1 ...`) installs the
# wheel versions of the flown bench image: base_station/requirements.lock.txt
# is applied as a pip constraints file on top of requirements.txt. The
# default (LOCKED unset or 0) is the unpinned resolve this file always did.
# The bench deploy script passes LOCKED=1
# (firmware/x8_lora_bootloader_helper/bench_tools/deploy_base.sh).
ARG LOCKED=0
COPY base_station/requirements.txt base_station/requirements.lock.txt /app/base_station/
RUN if [ "$LOCKED" = "1" ]; then \
        pip install --no-cache-dir -r /app/base_station/requirements.txt \
            -c /app/base_station/requirements.lock.txt; \
    else \
        pip install --no-cache-dir -r /app/base_station/requirements.txt; \
    fi

# Application code. firmware/common/lora_proto/ is also copied so
# lora_bridge.py can import the shared protocol header constants in the
# Python wrapper.
COPY base_station/  /app/base_station/
COPY firmware/      /app/firmware/

ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/app/base_station:/app

# Default command is the operator UI; docker-compose overrides for the
# bridge + audit-tail containers.
CMD ["uvicorn", "web_ui:app", "--host", "0.0.0.0", "--port", "8080"]
