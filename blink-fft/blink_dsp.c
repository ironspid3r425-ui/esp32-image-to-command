/* blink_dsp.c - see blink_dsp.h. */
#include <math.h>
#include "blink_dsp.h"

#define BD_PI 3.14159265358979f

static float hann[BD_N];
static const float CMD_HZ[3] = { 2.0f, 4.0f, 6.0f };   /* LEFT, RIGHT, STOP */

void bd_init(void)
{
    for (int i = 0; i < BD_N; i++) {
        hann[i] = 0.5f - 0.5f * cosf(2.0f * BD_PI * (float)i / (float)BD_N);
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

/* median of a small array (insertion sort on a copy) */
static float median(const float *v, int n)
{
    float tmp[BD_N / 2 + 1];
    for (int i = 0; i < n; i++) {
        float x = v[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > x) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = x;
    }
    return (n % 2) ? tmp[n / 2] : 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
}

bd_result_t bd_analyze(const float samples[BD_N], float fs)
{
    bd_result_t r = { 0.0f, 0.0f, BD_CMD_NONE };
    float re[BD_N], im[BD_N], pw[BD_N / 2 + 1];

    if (fs <= 0.0f) return r;

    float mean = 0.0f;
    for (int i = 0; i < BD_N; i++) mean += samples[i];
    mean /= (float)BD_N;
    for (int i = 0; i < BD_N; i++) { re[i] = (samples[i] - mean) * hann[i]; im[i] = 0.0f; }

    bd_fft(re, im, BD_N);
    for (int k = 0; k <= BD_N / 2; k++) pw[k] = re[k] * re[k] + im[k] * im[k];

    float df = fs / (float)BD_N;
    int kmin = (int)ceilf(BD_MIN_HZ / df);
    if (kmin < 1) kmin = 1;

    int kpk = kmin;
    for (int k = kmin; k < BD_N / 2; k++) if (pw[k] > pw[kpk]) kpk = k;

    /* noise floor: median power of the searched band, excluding the peak and its neighbours */
    float noise[BD_N / 2]; int nn = 0;
    for (int k = kmin; k < BD_N / 2; k++) if (k < kpk - 1 || k > kpk + 1) noise[nn++] = pw[k];
    float floor_pw = (nn > 0) ? median(noise, nn) : 0.0f;
    r.snr = (floor_pw > 1e-12f) ? pw[kpk] / floor_pw : (pw[kpk] > 1e-12f ? 1e6f : 0.0f);

    /* parabolic interpolation on log power for a sub-bin frequency estimate */
    float delta = 0.0f;
    if (kpk > 0 && kpk < BD_N / 2) {
        float a = logf(pw[kpk - 1] + 1e-12f), b = logf(pw[kpk] + 1e-12f), c = logf(pw[kpk + 1] + 1e-12f);
        float den = a - 2.0f * b + c;
        if (fabsf(den) > 1e-9f) delta = 0.5f * (a - c) / den;
    }
    r.peak_hz = ((float)kpk + delta) * df;

    if (r.snr >= BD_MIN_SNR) {
        for (int c = 0; c < 3; c++) {
            if (fabsf(r.peak_hz - CMD_HZ[c]) <= BD_TOL_HZ) { r.cmd = (bd_cmd_t)(c + 1); break; }
        }
    }
    return r;
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
