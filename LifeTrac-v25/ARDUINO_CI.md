# Arduino CI for LifeTrac v25

This directory contains Arduino sketches for the LifeTrac v25 control system, which are automatically tested using GitHub Actions continuous integration (CI).

## CI Workflow

The Arduino CI workflow automatically compiles both Arduino sketches whenever changes are made to:
- Arduino Opta Controller code (`arduino_opta_controller/`)
- ESP32 Remote Control code (`esp32_remote_control/`)
- Library requirements (`arduino_libraries.txt`)
- CI workflow configuration (`.github/workflows/arduino-ci.yml`)

## What Gets Tested

### Arduino Opta Controller
- **Board**: Arduino Opta WiFi (arduino:mbed_opta:opta)
- **Sketch**: `arduino_opta_controller/lifetrac_v25_controller.ino`
- **Libraries**: OptaController, PubSubClient, ArduinoJson, ArduinoBLE

### ESP32 Remote Control
- **Board**: SparkFun Thing Plus ESP32 WROOM (esp32:esp32:esp32thing_plus)
- **Sketch**: `esp32_remote_control/lifetrac_v25_remote.ino`
- **Libraries**: PubSubClient, ArduinoJson, SparkFun Qwiic Joystick Arduino Library

## Workflow Status

You can view the status of CI runs at:
https://github.com/OpenSourceEcology/LifeTrac/actions/workflows/arduino-ci.yml

## Compilation Reports

The workflow generates compilation reports showing:
- Whether the sketches compile successfully
- Binary size (flash and RAM usage)
- Size deltas compared to previous builds

These reports are uploaded as artifacts and retained for 30 days.

## Running CI Manually

The workflow can be triggered manually:
1. Go to the [Actions tab](https://github.com/OpenSourceEcology/LifeTrac/actions/workflows/arduino-ci.yml)
2. Click "Run workflow"
3. Select the branch and click "Run workflow"

## Local Testing

To test Arduino sketches locally before pushing:

### Using Arduino IDE
1. Install required libraries (see `arduino_libraries.txt`)
2. Select the appropriate board:
   - For Opta: Tools → Board → Arduino Mbed OS Opta Boards → Arduino Opta
   - For ESP32: Tools → Board → ESP32 Arduino → SparkFun ESP32 Thing Plus
3. Open the sketch and click "Verify" (checkmark icon)

### Using Arduino CLI
```bash
# Install Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Install board support
arduino-cli core install arduino:mbed_opta
arduino-cli core install esp32:esp32 --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# Install libraries
arduino-cli lib install OptaController
arduino-cli lib install PubSubClient
arduino-cli lib install ArduinoJson
arduino-cli lib install ArduinoBLE
arduino-cli lib install "SparkFun Qwiic Joystick Arduino Library"

# Compile sketches
arduino-cli compile --fqbn arduino:mbed_opta:opta LifeTrac-v25/arduino_opta_controller
arduino-cli compile --fqbn esp32:esp32:esp32thing_plus LifeTrac-v25/esp32_remote_control
```

## Troubleshooting

### Workflow Not Running
- New workflows require approval from a repository maintainer
- Check the Actions tab for pending approval requests

### Compilation Failures
- Check the workflow logs for specific error messages
- Verify all required libraries are listed in `arduino_libraries.txt`
- Ensure sketch code is compatible with the target board

### Adding New Libraries
If you add a new library dependency:
1. Update `arduino_libraries.txt` with the library name and version
2. Add the library to the workflow configuration in `.github/workflows/arduino-ci.yml`

## Contributing

When making changes to Arduino code:
1. Test compilation locally first
2. Create a pull request
3. Wait for CI to pass before merging
4. Fix any compilation errors reported by CI
