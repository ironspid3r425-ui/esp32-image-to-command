# blink-fft: frequency-domain optical commands (frame rate and rolling shutter)

A second command channel for the image-to-command controller. An LED blinking at a set frequency in front of the OV2640 is decoded with an FFT and turned into a command. Two modes, selected with `MODE` in the sketch:

| Command | FRAME mode (~25 Hz series) | ROWS mode (rolling shutter, kHz series) |
|---|---|---|
| LEFT | 2 Hz | 200 Hz |
| RIGHT | 4 Hz | 400 Hz |
| STOP | 6 Hz | 600 Hz |
| NONE | none / other | none / other |

## ROWS mode: sampling faster than the frame rate

The OV2640 has a rolling shutter: it reads a frame out one row at a time. Averaging each row of a single frame therefore gives a time series sampled at the **row rate** (a few kHz), not the frame rate. An LED blinking at hundreds of hertz, invisible to the eye, appears as horizontal stripes, and an FFT across the rows finds its frequency.

1. **Row means.** `camTask` averages each of the 120 rows of a 160x120 grayscale frame and queues the vector. It never blocks: frames are dropped and counted if the DSP falls behind.
2. **Background removal.** The image itself also varies from row to row (scene structure). A running per-row average (`alpha = 0.1`) is the static scene. The blink phase is different in every frame, so the blink averages out of that background, and subtracting it leaves only the flicker.
3. **Row-rate calibration.** The true row period depends on the sensor's internal readout and blanking, so it is measured, not assumed. At boot the self-test LED runs at a known 400 Hz via LEDC PWM, the dominant frequency in cycles per row is measured on 30 frames, and `row_rate = 400 Hz / median(cycles per row)`. Without self-test, a default of 3 kHz is used, so calibrate with any known-frequency blinker.
4. **FFT + decision.** The 120 rows are mean-removed, Hann-windowed, zero-padded to 128 and transformed. Peaks below 80 Hz (scene residue) are ignored, and the same peak-to-noise gate and two-frame debounce as FRAME mode decide the command.
5. **Exposure.** Exposure is locked short (`aec_value 20`). Each row integrates light for the exposure time, so if that approaches the 2.5 ms blink period the stripes wash out. Diffuse the LED (tissue paper right at the lens) so it lights the whole frame.

## FRAME mode: how it works

1. **Sensor time series.** `camTask` (core 1) grabs 160x120 grayscale frames and averages a 40x40 centre region, giving one brightness sample per frame plus a timestamp. Auto-exposure and auto-gain are locked so the camera doesn't "chase" the blink.
2. **DSP.** `dspTask` (core 0) takes the last 64 samples every 8 frames, measures the **actual** frame rate from the timestamps, removes the mean, applies a Hann window and runs a radix-2 FFT (`blink_dsp.c`). At ~25 fps the resolution is ~0.4 Hz per bin; the peak frequency is refined with parabolic interpolation.
3. **Decision.** The peak must be at least 6x the median noise power and within 0.6 Hz of a command frequency. A command is executed only after it is decoded on two consecutive windows (debounce).
4. **Self-test.** With `SELFTEST 1`, a third task drives an external LED on D1 through NONE / 2 / 4 / 6 Hz in 8-second phases. Every decision is scored against that known ground truth, and the serial log prints running accuracy, false triggers and DSP time.

Capture and DSP run on separate cores, so frame capture never waits on the FFT.

## Verified so far

- `blink_dsp.c` host tests: **11/11 pass** under `-std=c99 -Wall -Wextra -Werror`:
  - FRAME mode (7): FFT vs a naive DFT, decoding all three commands, rejecting noise / constant scenes / off-target frequencies, tolerating 20-30 fps frame-rate drift.
  - ROWS mode (4): decoding 200 / 400 / 600 Hz from simulated rolling-shutter frames with a textured scene and random blink phase; a static scene gives NONE; background removal cancels the scene exactly; calibration recovers a 3 kHz row rate within 3%.

```bash
cd test && make test
```

## On the board (fill in after running)

| Metric | Value |
|---|---|
| Measured frame rate | _ fps |
| Calibrated row rate (ROWS mode) | _ Hz |
| Self-test accuracy (per command, 10+ min) | _ |
| False triggers with no blink | _ |
| DSP time per window | _ µs |

## Hardware

Seeed XIAO ESP32-S3 Sense with OV2640, PSRAM enabled. For the self-test: an LED and a 220 Ω resistor from D1 (GPIO2) to GND, pointed at the lens from about 10 cm.
