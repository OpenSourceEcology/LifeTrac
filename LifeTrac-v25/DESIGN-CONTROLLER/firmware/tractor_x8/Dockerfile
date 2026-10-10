# LifeTrac tractor X8 camera/image pipeline Dockerfile
# PYTHON_IMAGE is overridable only to pin the flown base image by digest
# (see requirements.lock.txt). The default is unchanged.
ARG PYTHON_IMAGE=python:3.11-slim-bookworm
FROM ${PYTHON_IMAGE}

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    mosquitto-clients \
    libusb-1.0-0 \
    ffmpeg \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies.
# Optional LOCKED=1 (`docker build --build-arg LOCKED=1 ...`) installs the
# wheel versions of the flown bench image: requirements.lock.txt is applied
# as a pip constraints file on top of requirements.txt. The default (LOCKED
# unset or 0) is the unpinned resolve this file always did. The bench build
# script passes LOCKED=1
# (firmware/x8_lora_bootloader_helper/bench_tools/build_tractor_image.sh).
ARG LOCKED=0
COPY requirements.txt requirements.lock.txt /app/
WORKDIR /app
RUN if [ "$LOCKED" = "1" ]; then \
        pip install --no-cache-dir -r requirements.txt -c requirements.lock.txt; \
    else \
        pip install --no-cache-dir -r requirements.txt; \
    fi

# Copy camera/image pipeline code
COPY . /app

# Default: run camera_service.py (can override with CMD)
CMD ["python", "camera_service.py"]
