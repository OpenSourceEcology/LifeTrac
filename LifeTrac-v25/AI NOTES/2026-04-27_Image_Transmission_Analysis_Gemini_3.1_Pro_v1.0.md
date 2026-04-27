# LifeTrac v25 Image Transmission & Edge AI Analysis

**Reviewer:** Gemini 3.1 Pro (Preview)
**Date:** 2026-04-27
**Document Version:** v1.0

This document explores extreme, out-of-the-box methodologies for transmitting visual data over the severely constrained LoRa link. It shifts the focus from traditional video encoding (which fails organically on LoRa) to maximizing the untapped compute potential of the Portenta X8's quad-core Cortex-A53 Linux environment on both the Tractor and the Base Station, and leveraging the operator's web UI hardware.

---

## 1. The Compute Budget: What can the Portenta X8 actually run?

The Portenta X8 features an NXP i.MX 8M Mini (Quad-core Cortex-A53 @ 1.8 GHz) and 2GB of LPDDR4. While it lacks a dedicated high-power NPU/TPU (Neural Processing Unit), the quad-A53 is fully capable of running optimized AI models via **TensorFlow Lite (TFLite)**, **ONNX Runtime**, or **NCNN**. 

*   **Tractor X8 (Encoder):** Can comfortably process incoming UVC camera frames at 5-10 FPS if the models are quantized (INT8) and lightweight. It can run edge detection, semantic segmentation, and latent-space encoding.
*   **Base Station X8 (Decoder/Bridge):** Can run matching decoding models, but **crucially**, the Base Station is connected to the operator's laptop/tablet via Wi-Fi/Ethernet. *We should pass the heaviest AI reconstruction workloads from the Base X8 directly to the operator's Web Browser using WebAssembly/WebGPU.*

---

## 2. Targeted Image Updates: The "Smart Block" Priority Pipeline

Standard video codecs (H.264) generate I-frames and P-frames. P-frames track changes, but their size is unpredictable and often exceeds the MTU of a LoRa packet (~200 bytes max). We need a deterministic, packet-sized chunking algorithm tailored for LoRa.

### Concept: Saliency-Weighted Block Deltas
Instead of full frames, the Tractor maintains a grid of the camera view (e.g., 10x10 blocks). 
1.  **Diff Calculation:** The Tractor computes the Mean Absolute Error (MAE) between the current camera block and the last *acknowledged* block sent to the Base.
2.  **Saliency multiplier:** Standard algorithms waste bandwidth on clouds moving or dust blowing. We apply a lightweight masking algorithm (or central crop) to weight blocks containing the bucket, ground, or hydraulic implements heavily, and weight the sky/horizon at zero.
3.  **The Priority Queue:** Blocks are pushed to a priority queue based on `MAE * Saliency`. 
4.  **LoRa Drip-Feed:** Every time the MAC layer allows a `TelemetryFrame`, the Tractor pops the highest-priority block off the queue. It compresses just that 32x32 pixel block (e.g., a 100-byte WebP chunk) and transmits it with its grid coordinate `[X,Y]`.
5.  **Base Station Canvas:** The Base Station or Web UI maintains a persistent HTML5 Canvas. As `[X,Y]` chunks arrive, it patches them into the canvas. 

**Result:** The bucket and implements animate at maybe 1-2 FPS smoothly, while the sky and distant trees update only once every 30 seconds. This is perfectly predictable and never clogs the LoRa channel.

---

## 3. "Outside the Box" Open Source AI Pipelines

To shrink imagery to fit into < 100-byte payloads, we must stop sending pixels and start sending *semantics* or *latent space vectors*.

### 3.1. Vectorized Scene Drawing (The Wireframe Approach)
*   **Concept:** Pixels are heavy; math is light. 
*   **Execution:** The Tractor X8 runs a highly optimized edge-detection algorithm (Canny Edge Detection, or a lightweight AI like **HED / DexiNed** converted to TFLite).
*   **Transmission:** The Tractor converts the edges into vector lines (e.g., `Start_X, Start_Y, End_X, End_Y`). Translating the immediate off-road terrain and tractor bucket into 20-30 vector lines takes **under 100 bytes**.
*   **Base Station:** The Web UI renders these vectors on a black canvas. The operator sees a high-framerate, Tron-style wireframe representation of their environment and implements in real-time, giving spatial awareness without a single pixel of video data.

### 3.2. Agricultural Latent Space (The Autoencoder)
*   **Concept:** Autoencoders compress an image into a tiny array of numbers (the latent vector), discovering the core "concepts" of the image.
*   **Execution:** 
    *   We train a tiny Variational Autoencoder (VAE) specifically on LifeTrac datasets (dirt, grass, metal bucket, hydraulics, sky).
    *   **Tractor:** The Tractor scales the camera to 64x64 and runs the Encoder. It yields a vector of just 32 floats (converted to 32 bytes via INT8 quantization).
    *   **Transmission:** 32 bytes fits easily inside a standard LoRa telemetry payload.
    *   **Base Station:** The Base Station (or Web UI) runs the Decoder. It takes those 32 bytes and "hallucinates" a 64x64 agricultural image that perfectly matches the mathematical concepts the Tractor saw.
*   **Models:** Custom-trained PyTorch/TFLite Autoencoders. Size on disk: < 2MB.

### 3.3. Semantic Map Construction (Fast-SCNN)
*   **Concept:** Transmit "what" is there, not what it looks like.
*   **Execution:** The Tractor X8 runs **Fast-SCNN** (Fast Semantic Segmentation Network) or **MobileNetV3-Seg**. These run very well on quad-A53 chips. It classifies the image into a few colors: Green (vegetation), Brown (dirt/path), Blue (sky), Gray (tractor metal), Red (obstacle).
*   **Transmission:** Send a downscaled 20x15 grid of class IDs (1 byte per block = 300 bytes total, highly compressible using Run-Length Encoding to < 40 bytes). 
*   **Base Station:** The Web UI generates a top-down or synthetic 3D view using textured 3D assets to represent the dirt, grass, and tractor. 

### 3.4. AI-Enhanced Bounding Boxes (YOLOv8 Nano)
*   **Concept:** If the goal is obstacle avoidance and implement tracking, human vision isn't strictly necessary.
*   **Execution:** Tractor runs **YOLOv8n** (Nano, highly optimized for edge).
*   **Transmission:** It detects the bucket and any humans/animals/rocks. It transmits JSON or binary data: `[Class=Bucket, X=10, Y=20, W=40, H=20]`. Total payload: 5-10 bytes per object.
*   **Base Station:** The browser overlays these bounding boxes on the last known full-resolution background image, or on a synthetic 3D digital twin of the tractor.

---

## 4. Summary & Recommendation for v25

If you want visual feedback over the 915MHz LoRa link, pure video compression like MJPEG or H.264 is a dead end. 

**Recommended Action Plan:**
1.  **Short Term (Bench/Initial Field Test):** Implement the **Targeted Image Updates (Smart Block/Priority Queue)**. It requires no AI training, just basic array diffing in Python/OpenCV on the Tractor, and it allows deterministic control of the LoRa MTU.
2.  **Mid Term (Operator UI):** Move the heavy lifting (upscaling, interpolation, canvas patching) off the Base Station X8 and into the browser. Use `web_ui.py` merely as a websocket pass-through. The operator's laptop GPU should assemble the UI.
3.  **Long Term (Offroad Innovation):** Train a custom Autoencoder or use YOLOv8 Nano. Sending pure metadata (bounding boxes or 32-byte latent vectors) and rebuilding the visual context synthetically in the browser is the ultimate solution for extreme low-bandwidth robotics.