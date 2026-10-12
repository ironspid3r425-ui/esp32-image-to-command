/* blink_dsp.c - see blink_dsp.h. */
#include <math.h>
#include "blink_dsp.h"

#define BD_PI 3.14159265358979f

static float hann_frame[BD_N];
static const float FRAME_CMD_HZ[3] = { 2.0f, 4.0f, 6.0f };       /* LEFT, RIGHT, STOP */
static const float ROWS_CMD_HZ[3]  = { 200.0f, 400.0f, 600.0f }; /* LEFT, RIGHT, STOP */

void bd_init(void)
{
    for (int i = 0; i < BD_N; i++) {
        hann_frame[i] = 0.5f - 0.5f * cosf(2.0f * BD_PI * (float)i / (float)BD_N);
    }
}

void bd_fft(float *re, float *im, int n)
{
    /* bit-reversal permutation */
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    /* butterflies */
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * BD_PI / (float)len;
        float wr = cosf(ang), wi = sinf(ang);
        for (int i = 0; i < n; i += len) {
            float cr = 1.0f, ci = 0.0f;
            for (int k = 0; k < len / 2; k++) {
                int a = i + k, b = i + k + len / 2;
                float xr = re[b] * cr - im[b] * ci;
                float xi = re[b] * ci + im[b] * cr;
                re[b] = re[a] - xr; im[b] = im[a] - xi;
                re[a] += xr;        im[a] += xi;
                float ncr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = ncr;
            }
        }
    }
}

/* median of a small array (insertion sort into a local copy) */
static float median(const float *v, int n)
{
    float tmp[BD_NFFT_MAX / 2 + 1];
    for (int i = 0; i < n; i++) {
        float x = v[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > x) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = x;
    }
    return (n % 2) ? tmp[n / 2] : 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
}

/*
 * Shared core: remove mean, Hann over the n real samples, zero-pad to nfft, FFT,
 * find the strongest bin above fmin, noise floor = median of the other searched bins,
 * parabolic interpolation, then match against the command table.
 * fs = 1 gives frequencies in cycles per sample.
 */
static bd_result_t analyze_core(const float *x, int n, int nfft, float fs, float fmin,
                                const float *cmd_hz, float tol, float min_snr, const float *window)
{
    bd_result_t r = { 0.0f, 0.0f, BD_CMD_NONE };
    float re[BD_NFFT_MAX], im[BD_NFFT_MAX], pw[BD_NFFT_MAX / 2 + 1];

    if (fs <= 0.0f || n < 8 || n > nfft || nfft > BD_NFFT_MAX) return r;

    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += x[i];
    mean /= (float)n;
    for (int i = 0; i < nfft; i++) {
        float w = window ? window[i < n ? i : 0]
                         : 0.5f - 0.5f * cosf(2.0f * BD_PI * (float)i / (float)n);
        re[i] = (i < n) ? (x[i] - mean) * w : 0.0f;
        im[i] = 0.0f;
    }

    bd_fft(re, im, nfft);
    for (int k = 0; k <= nfft / 2; k++) pw[k] = re[k] * re[k] + im[k] * im[k];

    float df = fs / (float)nfft;
    int kmin = (int)ceilf(fmin / df);
    if (kmin < 1) kmin = 1;
    if (kmin >= nfft / 2) return r;

    int kpk = kmin;
    for (int k = kmin; k < nfft / 2; k++) if (pw[k] > pw[kpk]) kpk = k;

    float noise[BD_NFFT_MAX / 2]; int nn = 0;
    for (int k = kmin; k < nfft / 2; k++) if (k < kpk - 1 || k > kpk + 1) noise[nn++] = pw[k];
    float floor_pw = (nn > 0) ? median(noise, nn) : 0.0f;
    r.snr = (floor_pw > 1e-12f) ? pw[kpk] / floor_pw : (pw[kpk] > 1e-12f ? 1e6f : 0.0f);

    float delta = 0.0f;
    if (kpk > 0 && kpk < nfft / 2) {
        float a = logf(pw[kpk - 1] + 1e-12f), b = logf(pw[kpk] + 1e-12f), c = logf(pw[kpk + 1] + 1e-12f);
        float den = a - 2.0f * b + c;
        if (fabsf(den) > 1e-9f) delta = 0.5f * (a - c) / den;
    }
    r.peak_hz = ((float)kpk + delta) * df;

    if (cmd_hz && r.snr >= min_snr) {
        for (int c = 0; c < 3; c++) {
            if (fabsf(r.peak_hz - cmd_hz[c]) <= tol) { r.cmd = (bd_cmd_t)(c + 1); break; }
        }
    }
    return r;
}

bd_result_t bd_analyze(const float samples[BD_N], float fs)
{
    return analyze_core(samples, BD_N, BD_N, fs, BD_MIN_HZ, FRAME_CMD_HZ, BD_TOL_HZ, BD_MIN_SNR, hann_frame);
}

void bd_rows_remove_background(const float *rows, float *bg, float *out, int nrows, float alpha, int first)
{
    for (int i = 0; i < nrows; i++) {
        if (first) bg[i] = rows[i];
        out[i] = rows[i] - bg[i];
        bg[i] += alpha * (rows[i] - bg[i]);
    }
}

bd_result_t bd_analyze_rows(const float *rows, int nrows, float row_rate)
{
    return analyze_core(rows, nrows, BD_NFFT_MAX, row_rate, BD_ROWS_MIN_HZ,
                        ROWS_CMD_HZ, BD_ROWS_TOL_HZ, BD_ROWS_MIN_SNR, 0);
}

float bd_cycles_per_row(const float *rows, int nrows)
{
    /* fs = 1 -> frequency in cycles per row; skip the lowest few bins (scene residue) */
    bd_result_t r = analyze_core(rows, nrows, BD_NFFT_MAX, 1.0f, 3.0f / BD_NFFT_MAX, 0, 0.0f, 0.0f, 0);
    return (r.snr >= BD_ROWS_MIN_SNR) ? r.peak_hz : 0.0f;
}

const char *bd_cmd_name(bd_cmd_t c)
{
    switch (c) {
        case BD_CMD_LEFT:  return "LEFT";
        case BD_CMD_RIGHT: return "RIGHT";
        case BD_CMD_STOP:  return "STOP";
        default:           return "NONE";
    }
}
