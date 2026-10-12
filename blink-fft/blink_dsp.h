/*
 * blink_dsp.h - frequency-domain command decoder for the optical command channel.
 *
 * Two modes, same DSP core (mean removal, Hann window, zero-padded radix-2 FFT,
 * peak-to-noise gate, parabolic peak interpolation):
 *
 *   FRAME mode: one brightness sample per camera frame (~25 Hz time series).
 *               2 / 4 / 6 Hz LED blinks -> LEFT / RIGHT / STOP.
 *
 *   ROWS mode:  rolling shutter. The sensor reads a frame out row by row, so the
 *               row averages of ONE frame form a time series sampled at the row
 *               rate (kHz). 200 / 400 / 600 Hz blinks -> LEFT / RIGHT / STOP.
 *               A running background per row removes the static scene, and the
 *               row rate is calibrated against an LED of known frequency.
 *
 * Portable C99, no ESP-IDF / Arduino calls, no malloc: unit-tested on the host
 * (test/) and compiled unchanged into the sketch.
 */
#ifndef BLINK_DSP_H
#define BLINK_DSP_H

#ifdef __cplusplus
extern "C" {
#endif

#define BD_NFFT_MAX     128     /* largest FFT used (ROWS mode) */

/* FRAME mode */
#define BD_N            64      /* frames per window */
#define BD_MIN_HZ       1.0f    /* ignore drift / DC leakage below this */
#define BD_TOL_HZ       0.6f
#define BD_MIN_SNR      6.0f

/* ROWS mode */
#define BD_ROWS_MAX     128     /* max rows per frame (QQVGA = 120) */
#define BD_ROWS_MIN_HZ  80.0f   /* below this is scene structure / slow lighting */
#define BD_ROWS_TOL_HZ  40.0f
#define BD_ROWS_MIN_SNR 6.0f

typedef enum { BD_CMD_NONE = 0, BD_CMD_LEFT, BD_CMD_RIGHT, BD_CMD_STOP } bd_cmd_t;

typedef struct {
    float    peak_hz;   /* interpolated dominant frequency (0 if none) */
    float    snr;       /* peak power / median power of the other searched bins */
    bd_cmd_t cmd;       /* decoded command, NONE if weak or off-target */
} bd_result_t;

/* Build the FRAME-mode Hann table. Call once at startup. */
void bd_init(void);

/* In-place iterative radix-2 FFT. n must be a power of two and <= BD_NFFT_MAX. */
void bd_fft(float *re, float *im, int n);

/* FRAME mode: analyse BD_N per-frame brightness samples at measured frame rate fs. */
bd_result_t bd_analyze(const float samples[BD_N], float fs);

/* ROWS mode, step 1: subtract the static scene. bg holds a running per-row
 * average (alpha 0..1); out = rows - bg, then bg is updated. On the first call
 * pass first=1 to seed bg. */
void bd_rows_remove_background(const float *rows, float *bg, float *out, int nrows, float alpha, int first);

/* ROWS mode, step 2: analyse one frame's background-removed row means at row_rate Hz. */
bd_result_t bd_analyze_rows(const float *rows, int nrows, float row_rate);

/* ROWS mode calibration: dominant frequency in cycles per row (0..0.5) of one
 * frame's background-removed rows. With an LED at a known f_led Hz:
 *     row_rate = f_led / cycles_per_row. Returns 0 if no clear peak. */
float bd_cycles_per_row(const float *rows, int nrows);

const char *bd_cmd_name(bd_cmd_t c);

#ifdef __cplusplus
}
#endif
#endif
