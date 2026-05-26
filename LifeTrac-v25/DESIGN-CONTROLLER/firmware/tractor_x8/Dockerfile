# LifeTrac tractor X8 camera/image pipeline Dockerfile
FROM python:3.11-slim-bookworm

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    mosquitto-clients \
    libusb-1.0-0 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt /app/requirements.txt
WORKDIR /app
RUN pip install --no-cache-dir -r requirements.txt

# Copy camera/image pipeline code
COPY . /app

# Default: run camera_service.py (can override with CMD)
CMD ["python", "camera_service.py"]
