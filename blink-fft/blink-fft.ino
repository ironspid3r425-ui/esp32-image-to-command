/*
 * blink-fft.ino - optical command channel for the image-to-command controller.
 *
 * An LED blinking at 2 / 4 / 6 Hz in front of the OV2640 becomes LEFT / RIGHT / STOP.
 *   camTask (core 1): grab grayscale QQVGA frames, average a centre region of interest
 *                     -> one brightness sample + timestamp per frame into a ring buffer.
 *   dspTask (core 0): every HOP new samples, take the last 64, measure the real frame rate
 *                     from timestamps, Hann + radix-2 FFT (blink_dsp.c), pick the peak,
 *                     and act only after the same command is decoded twice in a row.
 *
 * SELFTEST: GPIO SELFTEST_PIN drives an external LED placed in front of the lens, cycling
 * through the three command frequencies, so accuracy is logged against known ground truth.
 *
 * Board: Seeed XIAO ESP32-S3 Sense (PSRAM enabled). Serial 115200.
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "board_config.h"
#include "blink_dsp.h"

#define LED_PIN        21      /* onboard user LED: "command executed" */
#define SELFTEST       1       /* 1 = drive our own blinking LED for automatic accuracy logging */
#define SELFTEST_PIN   2       /* D1 on the XIAO: LED + 220 ohm resistor to GND, aimed at the lens */
#define ROI            40      /* centre ROI is ROI x ROI pixels */
#define HOP            8       /* analyse every 8 new frames (~0.3 s at 25 fps) */

/* ---- ring buffer shared by the two tasks ---- */
#define RB 128
static float    rb_val[RB];
static uint32_t rb_ms[RB];
static volatile uint32_t rb_count = 0;            /* total samples written */
static portMUX_TYPE rb_mux = portMUX_INITIALIZER_UNLOCKED;

/* ---- ground truth + stats (self-test) ---- */
static volatile bd_cmd_t truth = BD_CMD_NONE;
static uint32_t n_eval = 0, n_ok = 0, n_false = 0;

static bool camera_init(void)
{
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0;  c.ledc_timer = LEDC_TIMER_0;
    c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
    c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
    c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM;
    c.pin_href = HREF_GPIO_NUM; c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
    c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
    c.xclk_freq_hz = 20000000;
    c.pixel_format = PIXFORMAT_GRAYSCALE;          /* 1 byte per pixel: brightness only */
    c.frame_size = FRAMESIZE_QQVGA;                /* 160 x 120: small = fast */
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;              /* never process a stale frame */
    if (esp_camera_init(&c) != ESP_OK) return false;

    /* Auto-exposure / auto-gain would "chase" the blink and flatten it - lock them. */
    sensor_t *s = esp_camera_sensor_get();
    s->set_exposure_ctrl(s, 0);
    s->set_aec_value(s, 200);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, 0);
    return true;
}

static void camTask(void *)
{
    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;
        uint32_t t = millis();
        const int w = fb->width, h = fb->height;
        const int x0 = (w - ROI) / 2, y0 = (h - ROI) / 2;
        uint32_t sum = 0;
        for (int y = y0; y < y0 + ROI; y++) {
            const uint8_t *row = fb->buf + y * w + x0;
            for (int x = 0; x < ROI; x++) sum += row[x];
        }
        esp_camera_fb_return(fb);

        portENTER_CRITICAL(&rb_mux);
        uint32_t i = rb_count % RB;
        rb_val[i] = (float)sum / (float)(ROI * ROI);
        rb_ms[i]  = t;
        rb_count++;
        portEXIT_CRITICAL(&rb_mux);
    }
}

static void execute(bd_cmd_t cmd)
{
    /* stand-in actuator: blink the onboard LED 1/2/3 times for LEFT/RIGHT/STOP */
    for (int k = 0; k < (int)cmd; k++) {
        digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(60));
        digitalWrite(LED_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(60));
    }
}

static void dspTask(void *)
{
    static float win[BD_N];
    uint32_t last = 0;
    bd_cmd_t prev = BD_CMD_NONE, active = BD_CMD_NONE;

    for (;;) {
        uint32_t n = rb_count;
        if (n < BD_N || n - last < HOP) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        last = n;

        uint32_t t_first, t_last;
        portENTER_CRITICAL(&rb_mux);
        for (int k = 0; k < BD_N; k++) win[k] = rb_val[(n - BD_N + k) % RB];
        t_first = rb_ms[(n - BD_N) % RB];
        t_last  = rb_ms[(n - 1) % RB];
        portEXIT_CRITICAL(&rb_mux);

        /* real frame rate over this window, not the nominal one */
        float fs = (t_last > t_first) ? (float)(BD_N - 1) * 1000.0f / (float)(t_last - t_first) : 0.0f;

        uint32_t c0 = ESP.getCycleCount();
        bd_result_t r = bd_analyze(win, fs);
        uint32_t us = (ESP.getCycleCount() - c0) / (uint32_t)ESP.getCpuFreqMHz();

        /* debounce: act only when the same command is decoded twice in a row */
        bd_cmd_t decided = (r.cmd == prev) ? r.cmd : active;
        prev = r.cmd;
        if (decided != active) {
            active = decided;
            if (active != BD_CMD_NONE) execute(active);
        }

        if (SELFTEST) {
            n_eval++;
            if (active == truth) n_ok++;
            if (truth == BD_CMD_NONE && active != BD_CMD_NONE) n_false++;
        }
        Serial.printf("fs=%.1f Hz  peak=%.2f Hz  snr=%.1f  -> %s  (truth %s)  dsp=%lu us  acc=%.1f%%  false=%lu\n",
                      fs, r.peak_hz, r.snr, bd_cmd_name(active), bd_cmd_name(truth),
                      (unsigned long)us, n_eval ? 100.0f * n_ok / n_eval : 0.0f, (unsigned long)n_false);
    }
}

#if SELFTEST
static void blinkTask(void *)
{
    /* each phase lasts 8 s: NONE, LEFT (2 Hz), RIGHT (4 Hz), STOP (6 Hz), repeat */
    const float hz[4] = { 0.0f, 2.0f, 4.0f, 6.0f };
    for (int phase = 0;; phase = (phase + 1) % 4) {
        truth = BD_CMD_NONE;                       /* don't score the transition */
        uint32_t start = millis();
        bool settled = false;
        while (millis() - start < 8000) {
            if (!settled && millis() - start > 3000) { truth = (bd_cmd_t)phase; settled = true; }
            if (hz[phase] == 0.0f) { digitalWrite(SELFTEST_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(50)); continue; }
            uint32_t half_ms = (uint32_t)(500.0f / hz[phase]);
            digitalWrite(SELFTEST_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(half_ms));
            digitalWrite(SELFTEST_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(half_ms));
        }
    }
}
#endif

void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
    bd_init();
    if (!camera_init()) { Serial.println("camera init failed"); while (true) delay(1000); }
#if SELFTEST
    pinMode(SELFTEST_PIN, OUTPUT);
    xTaskCreatePinnedToCore(blinkTask, "blink", 2048, nullptr, 2, nullptr, 0);
#endif
    xTaskCreatePinnedToCore(camTask, "cam", 4096, nullptr, 5, nullptr, 1);   /* capture never waits on DSP */
    xTaskCreatePinnedToCore(dspTask, "dsp", 8192, nullptr, 4, nullptr, 0);
    Serial.println("blink-fft running: aim the LED at the centre of the frame");
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
