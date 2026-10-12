# ESP32 Image-to-Command Controller

A real-time image-to-command decoder on a Seeed XIAO ESP32-S3 Sense: the onboard OV2640 camera continuously scans for QR codes, decodes them on a dedicated FreeRTOS task, and triggers an LED "command executed" signal — the same pattern a real embedded vision command interface would use, just with an LED standing in for whatever actuator/action the decoded payload would trigger.

## Problem

Turning a camera frame into an action on a microcontroller means solving three things at once: keeping the camera pipeline running without blocking, decoding image content (here, QR payloads) fast enough to react in real time, and doing it reliably enough to survive continuous operation, not just a one-off demo.

## Approach

Built in three stages:

1. **`prototype/`** — a minimal standalone sketch: camera init, QR decoding on its own FreeRTOS task (`onQrCodeTask`, pinned to core 1) so decoding never blocks the main loop, and an LED pulse on every valid decode. This proved out the core camera → decode → signal pipeline before adding anything else.
2. **`camera-web-server/`** — the same QR detection integrated into a full live MJPEG streaming server (extending Espressif's official `CameraWebServer` example), so the camera feed can be watched live over Wi-Fi while QR commands are decoded in the background.

3. **`blink-fft/`**: a second, frequency-domain command channel. An LED blinking at 2 / 4 / 6 Hz in front of the camera is decoded as LEFT / RIGHT / STOP. Each frame's centre-region brightness becomes a time series sampled at the camera frame rate. A DSP task on the other core runs a Hann-windowed 64-point radix-2 FFT, measures the real frame rate from timestamps, rejects weak or off-target peaks (peak-to-noise gate), and debounces before acting. A self-test mode drives its own LED through known frequencies and logs accuracy and false triggers. The DSP module is portable C99 with **7/7 host unit tests** (see [blink-fft/README.md](blink-fft/README.md)).

QR decoding uses the [ESP32QRCodeReader](https://github.com/alvarowolfx/ESP32QRCodeReader) library (built on the `quirc` decoder), which handles frame capture and decode; the project's own logic is the task architecture, LED signaling, and the web-server integration.

## Results

(As reported in the course final report/proposal for this project.)

- 95.3% functional accuracy decoding QR payloads under test.
- Root-caused bus/GPIO faults during bring-up, cutting driver init failures by ~90%.
- Validated with a 48.7-hour burn-in test plus a 500-cycle stress test — zero critical failures.

## Tech Stack

C++ (Arduino/ESP32 framework), C99 DSP (radix-2 FFT, Hann window), Unity host tests, FreeRTOS, ESP32-S3 (Seeed XIAO ESP32-S3 Sense), OV2640 camera, `ESP32QRCodeReader` (quirc-based QR decoding), ESP32 Arduino camera web server stack.

## How to Run

1. Install the ESP32 Arduino core and the `ESP32QRCodeReader` library via the Arduino Library Manager.
2. Open either `prototype/prototype.ino` (minimal QR-to-LED demo) or `camera-web-server/CameraWebServer.ino` (adds the live MJPEG stream) in the Arduino IDE.
3. Select the XIAO_ESP32S3 camera model (already set in `board_config.h`) and a partition scheme with enough app space (`camera-web-server` needs the custom scheme in `partitions.csv`).
4. Flash to the XIAO ESP32-S3 Sense; open the Serial Monitor at 115200 baud to see decode output (and, for the web server variant, the stream URL).

## What's Next

- Move the "LED pulse" stand-in to an actual downstream action (e.g. publish the decoded command over Wi-Fi/MQTT) so this becomes a genuine image-to-actuator pipeline instead of a local-only demo.
- Add automated accuracy/latency logging instead of relying on manual test runs for the reported metrics.

## Credits

`camera-web-server/app_httpd.cpp`, `camera_index.h`, `camera_pins.h`, and the overall `CameraWebServer` structure are from Espressif's official [arduino-esp32 CameraWebServer example](https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer); this project's contribution is the QR-decode task, LED signaling, and integrating the two together. QR decoding via [ESP32QRCodeReader](https://github.com/alvarowolfx/ESP32QRCodeReader) (quirc-based).
