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
    return UNITY_END();
}
