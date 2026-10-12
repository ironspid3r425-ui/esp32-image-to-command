/*
 * blink_dsp.h - frequency-domain command decoder for the optical command channel.
 *
 * Input:  BD_N brightness samples (one per camera frame) and the measured frame rate.
 * Output: the dominant blink frequency, its peak-to-noise ratio, and a command.
 *
 * Portable C99, no ESP-IDF / Arduino calls, no malloc, so it is unit-tested on the
 * host (test/) and compiled unchanged into the sketch.
 */
#ifndef BLINK_DSP_H
#define BLINK_DSP_H

#ifdef __cplusplus
extern "C" {
#endif

#define BD_N            64      /* FFT length (power of two) */
#define BD_MIN_HZ       1.0f    /* ignore drift / DC leakage below this */
#define BD_TOL_HZ       0.6f    /* how close the peak must be to a command frequency */
#define BD_MIN_SNR      6.0f    /* peak power / median noise power needed to act */

typedef enum { BD_CMD_NONE = 0, BD_CMD_LEFT, BD_CMD_RIGHT, BD_CMD_STOP } bd_cmd_t;

typedef struct {
    float    peak_hz;   /* interpolated dominant frequency (0 if none) */
    float    snr;       /* peak power / median power of the other bins */
    bd_cmd_t cmd;       /* decoded command, NONE if weak or off-target */
} bd_result_t;

/* Build the Hann window table. Call once at startup. */
void bd_init(void);

/* In-place iterative radix-2 FFT. n must be a power of two and <= BD_N. */
void bd_fft(float *re, float *im, int n);

/* Analyse one window: remove mean, Hann, FFT, pick peak, map to a command.
 * fs is the measured sample rate in Hz (camera frame rate). */
bd_result_t bd_analyze(const float samples[BD_N], float fs);

/* Command frequencies: LEFT = 2 Hz, RIGHT = 4 Hz, STOP = 6 Hz. */
const char *bd_cmd_name(bd_cmd_t c);

#ifdef __cplusplus
}
#endif
#endif
