# blink-fft: frequency-domain optical commands

A second command channel for the image-to-command controller. An LED blinking at a set frequency in front of the OV2640 is decoded with an FFT and turned into a command:

| Blink | Command |
|---|---|
| 2 Hz | LEFT |
| 4 Hz | RIGHT |
| 6 Hz | STOP |
| none / other | NONE |

## How it works

1. **Sensor time series.** `camTask` (core 1) grabs 160x120 grayscale frames and averages a 40x40 centre region, giving one brightness sample per frame plus a timestamp. Auto-exposure and auto-gain are locked so the camera doesn't "chase" the blink.
2. **DSP.** `dspTask` (core 0) takes the last 64 samples every 8 frames, measures the **actual** frame rate from the timestamps, removes the mean, applies a Hann window and runs a radix-2 FFT (`blink_dsp.c`). At ~25 fps the resolution is ~0.4 Hz per bin; the peak frequency is refined with parabolic interpolation.
3. **Decision.** The peak must be at least 6x the median noise power and within 0.6 Hz of a command frequency. A command is executed only after it is decoded on two consecutive windows (debounce).
4. **Self-test.** With `SELFTEST 1`, a third task drives an external LED on D1 through NONE / 2 / 4 / 6 Hz in 8-second phases. Every decision is scored against that known ground truth, and the serial log prints running accuracy, false triggers and DSP time.

Capture and DSP run on separate cores, so frame capture never waits on the FFT.

## Verified so far

- `blink_dsp.c` host tests: **7/7 pass** under `-std=c99 -Wall -Wextra -Werror`: FFT vs a naive DFT, decoding all three commands, rejecting noise / constant scenes / off-target frequencies, and tolerating 20-30 fps frame-rate drift.

```bash
cd test && make test
```

## On the board (fill in after running)

| Metric | Value |
|---|---|
| Measured frame rate | _ fps |
| Self-test accuracy (per command, 10+ min) | _ |
| False triggers with no blink | _ |
| DSP time per window | _ µs |

## Hardware

Seeed XIAO ESP32-S3 Sense with OV2640, PSRAM enabled. For the self-test: an LED and a 220 Ω resistor from D1 (GPIO2) to GND, pointed at the lens from about 10 cm.
