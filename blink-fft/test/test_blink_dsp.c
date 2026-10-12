/* Host tests for blink_dsp.c: FFT correctness, command decoding, noise rejection, frame-rate jitter. */
#include <math.h>
#include <stdlib.h>
#include "unity.h"
#include "blink_dsp.h"

#define PI_F 3.14159265358979f

void setUp(void) { bd_init(); srand(1234); }
void tearDown(void) {}

static float noise(float amp) { return amp * ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f; }

/* LED blink seen by the camera: square wave in brightness, plus sensor noise. */
static void square_blink(float *s, float f, float fs, float amp, float noise_amp)
{
    for (int i = 0; i < BD_N; i++) {
        float ph = sinf(2.0f * PI_F * f * (float)i / fs);
        s[i] = 100.0f + (ph >= 0.0f ? amp : -amp) + noise(noise_amp);
    }
}

void test_fft_matches_naive_dft(void)
{
    float re[16], im[16], x[16];
    for (int i = 0; i < 16; i++) { x[i] = sinf(0.7f * i) + 0.3f * cosf(2.1f * i); re[i] = x[i]; im[i] = 0.0f; }
    bd_fft(re, im, 16);
    for (int k = 0; k < 16; k++) {
        float dr = 0.0f, di = 0.0f;
        for (int n = 0; n < 16; n++) { dr += x[n] * cosf(2.0f * PI_F * k * n / 16.0f); di -= x[n] * sinf(2.0f * PI_F * k * n / 16.0f); }
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, dr, re[k]);
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, di, im[k]);
    }
}

void test_decodes_left_right_stop(void)
{
    float s[BD_N];
    const float hz[3] = { 2.0f, 4.0f, 6.0f };
    const bd_cmd_t want[3] = { BD_CMD_LEFT, BD_CMD_RIGHT, BD_CMD_STOP };
    for (int c = 0; c < 3; c++) {
        square_blink(s, hz[c], 25.0f, 20.0f, 3.0f);
        bd_result_t r = bd_analyze(s, 25.0f);
        TEST_ASSERT_EQUAL_INT(want[c], r.cmd);
        TEST_ASSERT_FLOAT_WITHIN(0.3f, hz[c], r.peak_hz);
    }
}

void test_no_blink_is_none(void)
{
    float s[BD_N];
    for (int i = 0; i < BD_N; i++) s[i] = 100.0f + noise(3.0f);
    TEST_ASSERT_EQUAL_INT(BD_CMD_NONE, bd_analyze(s, 25.0f).cmd);
}

void test_constant_scene_is_none(void)
{
    float s[BD_N];
    for (int i = 0; i < BD_N; i++) s[i] = 80.0f;
    TEST_ASSERT_EQUAL_INT(BD_CMD_NONE, bd_analyze(s, 25.0f).cmd);
}

void test_off_target_frequency_is_none(void)
{
    float s[BD_N];
    square_blink(s, 3.0f, 25.0f, 20.0f, 1.0f);       /* 3 Hz is between LEFT and RIGHT */
    TEST_ASSERT_EQUAL_INT(BD_CMD_NONE, bd_analyze(s, 25.0f).cmd);
}

void test_tolerates_frame_rate_variation(void)
{
    /* the camera's real rate drifts; the decoder uses the measured fs, not a constant */
    float s[BD_N];
    const float rates[3] = { 20.0f, 25.0f, 30.0f };
    for (int i = 0; i < 3; i++) {
        square_blink(s, 4.0f, rates[i], 20.0f, 3.0f);
        TEST_ASSERT_EQUAL_INT(BD_CMD_RIGHT, bd_analyze(s, rates[i]).cmd);
    }
}

void test_invalid_rate_is_none(void)
{
    float s[BD_N];
    square_blink(s, 2.0f, 25.0f, 20.0f, 1.0f);
    TEST_ASSERT_EQUAL_INT(BD_CMD_NONE, bd_analyze(s, 0.0f).cmd);
}

/* ---------------- ROWS mode (rolling shutter) ---------------- */
#define NROWS 120
#define ROW_RATE 3000.0f

/* One frame: static scene (gradient + texture) + LED square wave sampled row by row,
 * with a random blink phase per frame (the blink is not synchronised to the camera). */
static void rows_frame(float *rows, float f_led, float amp, float noise_amp)
{
    float phase = (float)rand() / (float)RAND_MAX;            /* 0..1 cycle */
    for (int i = 0; i < NROWS; i++) {
        float scene = 60.0f + 0.4f * (float)i + 20.0f * sinf(0.3f * (float)i);
        float blink = 0.0f;
        if (f_led > 0.0f) {
            float cyc = f_led * (float)i / ROW_RATE + phase;
            blink = (cyc - floorf(cyc) < 0.5f) ? amp : -amp;
        }
        rows[i] = scene + blink + noise(noise_amp);
    }
}

/* Run the background for a while, then decode the next frame. */
static bd_result_t rows_decode(float f_led)
{
    float rows[NROWS], bg[NROWS], out[NROWS];
    for (int fr = 0; fr < 40; fr++) {
        rows_frame(rows, f_led, 15.0f, 2.0f);
        bd_rows_remove_background(rows, bg, out, NROWS, 0.1f, fr == 0);
    }
    rows_frame(rows, f_led, 15.0f, 2.0f);
    bd_rows_remove_background(rows, bg, out, NROWS, 0.1f, 0);
    return bd_analyze_rows(out, NROWS, ROW_RATE);
}

void test_rows_decodes_200_400_600_hz(void)
{
    const float hz[3] = { 200.0f, 400.0f, 600.0f };
    const bd_cmd_t want[3] = { BD_CMD_LEFT, BD_CMD_RIGHT, BD_CMD_STOP };
    for (int c = 0; c < 3; c++) {
        bd_result_t r = rows_decode(hz[c]);
        TEST_ASSERT_EQUAL_INT(want[c], r.cmd);
        TEST_ASSERT_FLOAT_WITHIN(25.0f, hz[c], r.peak_hz);
    }
}

void test_rows_static_scene_is_none(void)
{
    TEST_ASSERT_EQUAL_INT(BD_CMD_NONE, rows_decode(0.0f).cmd);
}

void test_rows_background_removes_scene(void)
{
    float rows[NROWS], bg[NROWS], out[NROWS];
    for (int fr = 0; fr < 40; fr++) {
        rows_frame(rows, 0.0f, 0.0f, 0.0f);
        bd_rows_remove_background(rows, bg, out, NROWS, 0.1f, fr == 0);
    }
    for (int i = 0; i < NROWS; i++) TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, out[i]);
}

void test_rows_calibration_recovers_row_rate(void)
{
    float rows[NROWS], bg[NROWS], out[NROWS];
    for (int fr = 0; fr < 40; fr++) {
        rows_frame(rows, 400.0f, 15.0f, 2.0f);
        bd_rows_remove_background(rows, bg, out, NROWS, 0.1f, fr == 0);
    }
    float cpr = bd_cycles_per_row(out, NROWS);
    TEST_ASSERT_TRUE(cpr > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.03f * ROW_RATE, ROW_RATE, 400.0f / cpr);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_fft_matches_naive_dft);
    RUN_TEST(test_decodes_left_right_stop);
    RUN_TEST(test_no_blink_is_none);
    RUN_TEST(test_constant_scene_is_none);
    RUN_TEST(test_off_target_frequency_is_none);
    RUN_TEST(test_tolerates_frame_rate_variation);
    RUN_TEST(test_invalid_rate_is_none);
    RUN_TEST(test_rows_decodes_200_400_600_hz);
    RUN_TEST(test_rows_static_scene_is_none);
    RUN_TEST(test_rows_background_removes_scene);
    RUN_TEST(test_rows_calibration_recovers_row_rate);
    return UNITY_END();
}
