# LifeTrac v25 ROS2 Architecture

## System Overview

The LifeTrac v25 ROS2 integration provides a bridge between ROS2 control systems (running on BeagleBone or similar devices) and the existing MQTT-based control infrastructure.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           ROS2 Control Layer                             │
│                         (BeagleBone / ROS2 Device)                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  ┌──────────────────┐         ┌────────────────────┐                    │
│  │  User ROS2 Node  │         │  Test Publisher    │                    │
│  │  (Autonomous)    │         │  (Development)     │                    │
│  └────────┬─────────┘         └──────────┬─────────┘                    │
│           │                              │                               │
│           │  Publishes                   │  Publishes                    │
│           └──────────────┬───────────────┘                               │
│                          │                                               │
│                          ▼                                               │
│              ┌────────────────────────┐                                  │
│              │  ROS2 Topic            │                                  │
│              │  /lifetrac/control_cmd │                                  │
│              │  (ControlCommand msg)  │                                  │
│              └───────────┬────────────┘                                  │
│                          │                                               │
│           ┌──────────────┴──────────────┐                                │
│           │                             │                                │
│           ▼                             ▼                                │
│  ┌────────────────┐         ┌──────────────────────┐                    │
│  │ mqtt_client    │         │ lifetrac_mqtt_bridge │                    │
│  │ (ika-rwth)     │         │ (validation/logging) │                    │
│  └────────┬───────┘         └──────────┬───────────┘                    │
│           │                            │                                 │
└───────────┼────────────────────────────┼─────────────────────────────────┘
            │                            │
            │  MQTT Bridge               │  Monitoring
            │                            │
            ▼                            ▼
┌───────────────────────────────────────────────────────────────────────┐
│                        MQTT Communication Layer                        │
│                          (WiFi Network)                                │
├───────────────────────────────────────────────────────────────────────┤
│                                                                         │
│                    ┌────────────────────────┐                          │
│                    │   MQTT Broker          │                          │
│                    │   (Mosquitto)          │                          │
│                    │   Raspberry Pi         │                          │
│                    │   192.168.1.100:1883   │                          │
│                    └──────┬─────────────┬───┘                          │
│                           │             │                               │
│              ┌────────────┘             └───────────┐                   │
│              │                                      │                   │
└──────────────┼──────────────────────────────────────┼───────────────────┘
               │                                      │
               ▼                                      ▼
    ┌──────────────────────┐              ┌────────────────────┐
    │  Arduino Opta        │              │  ESP32 Remote      │
    │  Controller          │              │  Control           │
    │  (Hydraulic Control) │              │  (Manual Control)  │
    └──────────┬───────────┘              └────────────────────┘
               │
               │ Controls
               │
               ▼
    ┌──────────────────────┐
    │  LifeTrac v25        │
    │  Hydraulic System    │
    │  - Tracks            │
    │  - Arms              │
    │  - Bucket            │
    └──────────────────────┘
```

## Data Flow

### Control Command Flow (ROS2 → LifeTrac)

```
1. User Code/Autonomous System
   ↓
2. Publishes ControlCommand to /lifetrac/control_cmd
   ↓
3. mqtt_client node receives message
   ↓
4. Converts ROS2 msg to MQTT JSON payload
   ↓
5. Publishes to MQTT topic: lifetrac/v25/control
   ↓
6. MQTT Broker (Mosquitto) routes message
   ↓
7. Arduino Opta subscribes and receives
   ↓
8. Processes joystick values
   ↓
9. Controls hydraulic valves
   ↓
10. Physical movement of LifeTrac
```

### Status Flow (LifeTrac → ROS2)

```
1. Arduino Opta reads sensor states
   ↓
2. Publishes status to MQTT: lifetrac/v25/status
   ↓
3. MQTT Broker routes message
   ↓
4. mqtt_client node receives from MQTT
   ↓
5. Converts to ROS2 String message
   ↓
6. Publishes to /lifetrac/status
   ↓
7. ROS2 nodes can subscribe for monitoring
```

## Message Formats

### ROS2 ControlCommand Message

```
# lifetrac_msgs/msg/ControlCommand.msg
int32 left_x      # -100 to 100 (turning)
int32 left_y      # -100 to 100 (forward/backward)
int32 right_x     # -100 to 100 (bucket)
int32 right_y     # -100 to 100 (arms)
uint64 timestamp  # milliseconds
```

### MQTT JSON Payload

```json
{
  "left_x": 0,
  "left_y": 50,
  "right_x": 0,
  "right_y": 0,
  "timestamp": 1234567890
}
```

## Component Responsibilities

### 1. User ROS2 Nodes
- **Purpose**: Implement autonomous control logic
- **Input**: Sensors, navigation goals, task plans
- **Output**: ControlCommand messages
- **Examples**: 
  - Autonomous navigation
  - Obstacle avoidance
  - Task execution (digging, moving)

### 2. mqtt_client (ika-rwth-aachen)
- **Purpose**: Bridge ROS2 topics to MQTT broker
- **Function**: Protocol translation
- **Configuration**: Bridge mapping in params.yaml
- **Features**: 
  - Bidirectional communication
  - QoS handling
  - Automatic reconnection

### 3. lifetrac_mqtt_bridge Node
- **Purpose**: Validation and monitoring
- **Functions**:
  - Input range validation (-100 to 100)
  - Command logging
  - Status monitoring
  - Error reporting

### 4. MQTT Broker (Mosquitto)
- **Purpose**: Message routing hub
- **Topics**:
  - `lifetrac/v25/control` - Control commands
  - `lifetrac/v25/status` - System status
  - `lifetrac/v25/remote_status` - Remote status
- **Security**: Username/password authentication

### 5. Arduino Opta Controller
- **Purpose**: Direct hydraulic control
- **Functions**:
  - MQTT subscription
  - Safety timeout (1 second)
  - Valve control
  - Status publishing
  - Flow control via Burkert 8605

### 6. ESP32 Remote Control (Optional)
- **Purpose**: Manual override/testing
- **Features**:
  - Dual joystick input
  - Emergency stop
  - Direct MQTT publishing
  - Battery monitoring

## Network Topology

```
WiFi Network: 192.168.1.0/24
├── BeagleBone (ROS2)        : 192.168.1.50 (example)
├── Raspberry Pi (MQTT)      : 192.168.1.100 (default)
├── Arduino Opta             : DHCP or static
└── ESP32 Remote             : DHCP or static

Ports:
- MQTT: 1883 (TCP)
- MQTT WebSocket: 9001 (TCP)
```

## Communication Protocols

### ROS2 DDS
- **Transport**: UDP multicast (default) or TCP
- **QoS**: Configurable reliability and durability
- **Discovery**: Automatic node discovery

### MQTT
- **Version**: MQTT v5.0 (protocol level 5)
- **Transport**: TCP
- **QoS**: Level 1 (at least once delivery)
- **Keep-alive**: 60 seconds

## Latency Considerations

Typical end-to-end latencies:

```
ROS2 Node → MQTT → Arduino Opta → Valve Response
    2ms       10ms        50ms         100-500ms
    
Total latency: ~160-560ms
```

Factors affecting latency:
- WiFi signal strength
- Network congestion
- MQTT broker load
- Hydraulic response time

For real-time control:
- Publish at 10-20 Hz minimum
- Monitor round-trip time
- Implement predictive control if needed

## Failure Modes and Recovery

### Network Loss
```
BeagleBone ─X─ MQTT Broker
    ↓
mqtt_client detects disconnection
    ↓
Attempts automatic reconnection
    ↓
Success → Resume operation
Failure → Log error, keep trying
```

### Command Timeout (Arduino Opta)
```
Last command received
    ↓
> 1 second elapsed
    ↓
Safety timeout triggered
    ↓
All valves → Neutral position
    ↓
Movement stops
    ↓
Resume when commands received
```

### Validation Failures
```
Invalid ControlCommand received
(values out of -100 to 100 range)
    ↓
lifetrac_mqtt_bridge rejects
    ↓
Log warning
    ↓
Do not forward to MQTT
```

## Scalability

The architecture supports:
- **Multiple controllers**: Multiple BeagleBones can control the same LifeTrac
- **Multiple LifeTracs**: One BeagleBone can control multiple LifeTracs
- **Sensor integration**: Additional ROS2 nodes for cameras, GPS, IMU, etc.
- **Cloud connectivity**: Bridge ROS2 to cloud services for telemetry

Example multi-LifeTrac setup:
```
BeagleBone (ROS2)
    ├── mqtt_client → lifetrac/v25_unit1/control
    └── mqtt_client → lifetrac/v25_unit2/control
```

## Security Considerations

### Current Implementation
- MQTT authentication (username/password)
- WiFi encryption (WPA2)
- Local network only (no internet exposure)

### Recommended Enhancements
- SSL/TLS for MQTT (port 8883)
- Certificate-based authentication
- Network segmentation (VLAN)
- Rate limiting on MQTT broker
- Command signing/verification

## Performance Optimization

### For Low-Power Devices (BeagleBone Black)
- Reduce publishing rate (5-10 Hz)
- Disable debug logging
- Use MQTT QoS 0 for lower overhead
- Optimize node computational load

### For High-Performance (BeagleBone AI-64)
- Increase publishing rate (20-50 Hz)
- Enable full logging
- Run additional sensor processing
- Support multiple simultaneous tasks

## Future Enhancements

Potential improvements:
1. **Sensor feedback loop**: Add position/force feedback from LifeTrac
2. **Path planning**: Integration with Nav2 for autonomous navigation
3. **Computer vision**: Object detection for autonomous operation
4. **Fleet management**: Coordinate multiple LifeTracs
5. **Cloud telemetry**: Real-time monitoring dashboard
6. **Machine learning**: Adaptive control based on terrain

## References

- [ROS2 Architecture](https://docs.ros.org/en/humble/Concepts.html)
- [MQTT Protocol Specification](https://mqtt.org/mqtt-specification/)
- [mqtt_client Package](https://github.com/ika-rwth-aachen/mqtt_client)
