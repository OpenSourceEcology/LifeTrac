# LifeTrac v25 Controller Code Review Report

*Model: Gemini 3.1 Pro (Preview)*
*Date: April 28, 2026*

## 1. Architectural & Protocol Logic Issues

### 1.1 Sequence Number Mismatch between `web_ui.py` and `lora_bridge.py`
**Severity: High**
* **Location:** `base_station/web_ui.py` (websocket `ws_control` handler) & `base_station/lora_bridge.py` (MQTT `_on_mqtt_message` handler).
* **Issue:** When the operator interacts with the web UI joystick, `web_ui.py` pulls a value from its own `seq` counter, manually embeds it into a packed binary `ControlFrame`, and publishes that payload to the `lifetrac/v25/cmd/control` MQTT topic.
However, in `lora_bridge.py`, the `cmd/control` handler receives this payload and passes it straight into `self._tx(SRC_BASE, msg.payload)` **without parsing or providing the `nonce_seq` parameter**. As a result, the bridge allocates a *brand new sequence number* via `self._reserve_tx_seq()` and injects it into the AES-GCM nonce on transmission.
* **Consequence:** The sequence number embedded in the cleartext frame header will not match the sequence number of the radio nonce. Worse, the `LoraReplayWindow` mechanism on the tractor (the receiver) evaluates sequence numbers contained in the *inner frame header*, meaning your security audit trails logging the *bridge outer nonce* will disagree with the real protocol frame ID. If the bridge resets but the web UI doesn't (or vice versa), the desync persists.
* **Fix:** When `lora_bridge.py` intercepts `cmd/control` payloads, it should unpack the packet header, extract the nested sequence number, and pass it directly to `_tx`:
  ```python
  from lora_proto import parse_header

  # Inside _on_mqtt_message for cmd/control
  if not verify_crc(msg.payload): return
  hdr = parse_header(msg.payload)
  if hdr:
      self._tx(SRC_BASE, msg.payload, nonce_seq=hdr.sequence_num)
  ```

### 1.2 Multi-Device Lockout on Failed Logins
**Severity: Medium**
* **Location:** `base_station/web_ui.py` (Rate limiting & Lockout rules).
* **Issue:** `_fail_counts[ip]` blocks clients entirely if they exceed 5 failed PIN attempts. The `_client_ip()` lookup uses `request.client.host`.
* **Consequence:** If the base station runs inside a docker environment, behind NGINX, or a reverse proxy handling the HTTPS endpoint, `request.client.host` may resolve to `127.0.0.1` or the container bridge IP for *all users*. One unauthorized actor continually scraping the login page will aggressively lock out the legitimate operator from using the Web UI.
* **Fix:** Implement `X-Forwarded-For` parsing inside `_client_ip()` (or utilize FastAPI's `TrustedHostMiddleware`/`ProxyHeadersMiddleware`).

---

## 2. Firmware C Implementation Issues

### 2.1 Inefficient Buffer Bounds Checks in `lp_kiss_encode` 
**Severity: Medium**
* **Location:** `firmware/common/lora_proto/lora_proto.c` (`lp_kiss_encode`).
* **Issue:** Inside the `lp_kiss_encode` loop, it strictly asserts `if (o + 2 >= out_max) return 0;` before pulling the next byte. 
* **Consequence:** If you are encoding a 1-byte non-escaped payload into a tightly-bound `out_max` buffer (e.g. `out_max = 3`), `o=1`. Your check evaluates `1 + 2 >= 3` (`3 >= 3`), triggering a false failure and discarding perfectly valid 2-byte allocations. It treats every single byte homogeneously as an escaped doublet that requires 2 slots, drastically limiting maximum throughput if buffer pooling limits are slightly misaligned.
* **Fix:** Check space linearly dependent on characters that demand escape prefixes.
  ```c
  for (size_t i = 0; i < in_len; i++) {
        uint8_t b = in[i];
        if (b == KISS_FEND || b == KISS_FESC) {
            if (o + 3 >= out_max) return 0; // Space for esc doublet + trailing flag 
        } else {
            if (o + 2 >= out_max) return 0; // Space for byte + trailing flag 
        }
  }
  ```

### 2.2 `CommandFrame` Variable Length Pointer Overwrite Risk
**Severity: Medium**
* **Location:** `firmware/common/lora_proto/lora_proto.c` (`lp_make_command`).
* **Issue:** In the C prototype, `CommandFrame` contains `uint8_t arg[8];` immediately followed by `uint16_t crc16;`. 
When `lp_make_command` structures variable-length commands, it explicitly overwrites random offset locations in memory using `raw[prefix] = crc & 0xFF`, bypassing positional struct assignments:
  ```c
  size_t prefix = sizeof(LoraHeader) + 1 + arg_len;
  raw[prefix] = (uint8_t)(crc & 0xFF);
  ```
* **Consequence:** It overwrites the middle of the `arg[8]` array inside the struct! If other C code naturally accesses `f->crc16` under the assumption that `CommandFrame` holds standard offset rules, they will read garbage values positioned `sizeof(f)` away from the dynamically injected CRC.
* **Fix:** Convert `CommandFrame` to utilize C99 Flexible Array Members (e.g. `uint8_t arg[]`), or distinctly split the serialization buffer from the `struct` definition if variable transmission lengths are intended.

---

## 3. Improvements & Optimizations

### 3.1 Unified Auditing Rules
**Location:** `web_ui.py` authentication rejections (`_record_failure()`).
* **Improvement:** While MQTT operations and LoRa `AESGCM` rejections are cleanly dispatched to the permanent `AuditLog` format, UI login failures are emitted using standard `logging.warning()`. Authentication errors are highly critical base-station security events and should be funneled systematically into the `AuditLog` instance so they survive `systemd` / container reloads organically, rendering them visible in offline diagnostic extracts (`/audit`).

### 3.2 Over-The-Air AES-GCM Nonce Bleeding
**Location:** `lora_proto.py` (`encrypt_frame()`) & `lp_crypto_real.cpp`.
* **Improvement:** A full 12-byte nonce (containing the sequence number, `now_s` UNIX timestamps, and random 5-byte tails) is prepended raw to each transmission. Given that `LoraHeader`'s cleartext layer essentially exposes `source_id`, `version`, and `sequence_num` independently at the front of each `decrypt_frame()`, you could heavily compress the 12-byte payload padding cost by deterministically re-inferring repetitive parameters and truncating the over-the-air randomness, lowering bandwidth costs for critical LoRa P0 allocations.

### 3.3 `asyncio` WebSocket Handling 
**Location:** `ws_control` inside `web_ui.py`.
* **Improvement:** Inside the `ws_control` WebSocket handler, you poll time `now = asyncio.get_event_loop().time()` on each packet ingress, checking `now - last_tx < 0.05` to constrain inputs to a conservative 20Hz. If a rogue/stuck client begins blasting inputs to the socket faster than the event loop yields context, FastAPI might suffer thread-starvation on `receive_text`. It is advisable to use `asyncio.sleep` to cleanly drain buffer boundaries or properly backpressure inputs via asynchronous queues.