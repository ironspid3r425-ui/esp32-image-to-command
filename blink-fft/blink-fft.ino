/*
 * blink-fft.ino - optical command channel for the image-to-command controller.
 *
 * MODE_FRAME: one brightness sample per frame (~25 Hz time series). LED blinking at
 *             2 / 4 / 6 Hz -> LEFT / RIGHT / STOP.
 * MODE_ROWS : rolling shutter. The OV2640 reads each frame out row by row, so the 120
 *             row averages of a single frame are a time series sampled at the row rate
 *             (kHz). LED blinking at 200 / 400 / 600 Hz (invisible to the eye) ->
 *             LEFT / RIGHT / STOP. A running per-row background removes the static
 *             scene, and the row rate is calibrated at boot against a 400 Hz LED.
 *
 *   camTask (core 1): capture + reduce each frame (ROI mean, or row means).
 *   dspTask (core 0): background removal, Hann + radix-2 FFT (blink_dsp.c), peak-to-noise
 *                     gate, debounce, act. Capture never waits on the DSP.
 *
 * SELFTEST drives an external LED on SELFTEST_PIN through known frequencies (LEDC PWM in
 * ROWS mode for exact timing) and scores every decision against that ground truth.
 *
 * Board: Seeed XIAO ESP32-S3 Sense, PSRAM enabled, ESP32 Arduino core 3.x. Serial 115200.
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "board_config.h"
#include "blink_dsp.h"

#define MODE_FRAME     0
#define MODE_ROWS      1
#define MODE           MODE_ROWS   /* pick the mode here */

#define LED_PIN        21          /* onboard user LED: "command executed" */
#define SELFTEST       1           /* 1 = drive our own LED for calibration + accuracy logging */
#define SELFTEST_PIN   2           /* D1: LED + 220 ohm to GND, diffused (tissue) right at the lens */
#define ROI            40          /* FRAME mode: centre ROI x ROI pixels */
#define HOP            8           /* FRAME mode: analyse every 8 new frames */
#define CAL_HZ         400.0f      /* ROWS mode: known calibration frequency */
#define CAL_FRAMES     30
#define BG_ALPHA       0.1f        /* ROWS mode: background update rate per frame */

static volatile bd_cmd_t truth = BD_CMD_NONE;     /* self-test ground truth */
static uint32_t n_eval = 0, n_ok = 0, n_false = 0;

/* ================= camera ================= */
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
    c.pixel_format = PIXFORMAT_GRAYSCALE;          /* brightness only */
    c.frame_size = FRAMESIZE_QQVGA;                /* 160 x 120 */
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;
    if (esp_camera_init(&c) != ESP_OK) return false;

    /* Lock exposure and gain so the camera doesn't adapt to the blink. In ROWS mode the
     * exposure must also be short compared with the blink period (2.5 ms at 400 Hz),
     * otherwise each row integrates over the whole blink and the stripes wash out. */
    sensor_t *s = esp_camera_sensor_get();
    s->set_exposure_ctrl(s, 0);
    s->set_aec_value(s, MODE == MODE_ROWS ? 20 : 200);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, MODE == MODE_ROWS ? 8 : 0);
    return true;
}

static void execute(bd_cmd_t cmd)
{
    /* stand-in actuator: blink the onboard LED 1/2/3 times for LEFT/RIGHT/STOP */
    for (int k = 0; k < (int)cmd; k++) {
        digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(60));
        digitalWrite(LED_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(60));
    }
}

/* debounce + scoring shared by both modes */
static void decide_and_log(bd_result_t r, float rate, uint32_t us, const char *rate_name)
{
    static bd_cmd_t prev = BD_CMD_NONE, active = BD_CMD_NONE;
    bd_cmd_t decided = (r.cmd == prev) ? r.cmd : active;   /* same result twice in a row */
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
    Serial.printf("%s=%.1f Hz  peak=%.1f Hz  snr=%.1f  -> %s  (truth %s)  dsp=%lu us  acc=%.1f%%  false=%lu\n",
                  rate_name, rate, r.peak_hz, r.snr, bd_cmd_name(active), bd_cmd_name(truth),
                  (unsigned long)us, n_eval ? 100.0f * n_ok / n_eval : 0.0f, (unsigned long)n_false);
}

#if MODE == MODE_FRAME
/* ================= FRAME mode ================= */
#define RB 128
static float    rb_val[RB];
static uint32_t rb_ms[RB];
static volatile uint32_t rb_count = 0;
static portMUX_TYPE rb_mux = portMUX_INITIALIZER_UNLOCKED;

static void camTask(void *)
{
    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;
        uint32_t t = millis();
        const int w = fb->width, h = fb->height, x0 = (w - ROI) / 2, y0 = (h - ROI) / 2;
        uint32_t sum = 0;
        for (int y = y0; y < y0 + ROI; y++) {
            const uint8_t *row = fb->buf + y * w + x0;
            for (int x = 0; x < ROI; x++) sum += row[x];
        }
        esp_camera_fb_return(fb);
        portENTER_CRITICAL(&rb_mux);
        uint32_t i = rb_count % RB;
        rb_val[i] = (float)sum / (float)(ROI * ROI);
        rb_ms[i] = t;
        rb_count++;
        portEXIT_CRITICAL(&rb_mux);
    }
}

static void dspTask(void *)
{
    static float win[BD_N];
    uint32_t last = 0;
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
        float fs = (t_last > t_first) ? (float)(BD_N - 1) * 1000.0f / (float)(t_last - t_first) : 0.0f;
        uint32_t c0 = ESP.getCycleCount();
        bd_result_t r = bd_analyze(win, fs);
        decide_and_log(r, fs, (ESP.getCycleCount() - c0) / (uint32_t)ESP.getCpuFreqMHz(), "fs");
    }
}

#if SELFTEST
static void blinkTask(void *)
{
    const float hz[4] = { 0.0f, 2.0f, 4.0f, 6.0f };      /* NONE, LEFT, RIGHT, STOP: 8 s each */
    for (int phase = 0;; phase = (phase + 1) % 4) {
        truth = BD_CMD_NONE;
        uint32_t start = millis(); bool settled = false;
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

#else
/* ================= ROWS mode (rolling shutter) ================= */
typedef struct { float rows[BD_ROWS_MAX]; int n; uint32_t ms; } rows_msg_t;
static QueueHandle_t rows_q;
static volatile uint32_t dropped = 0;
static volatile bool calibrating = (SELFTEST != 0);
static float row_rate = 3000.0f;                    /* replaced by calibration */

static void led_hz(float hz)
{
    if (hz <= 0.0f) { ledcWrite(SELFTEST_PIN, 0); return; }
    ledcChangeFrequency(SELFTEST_PIN, (uint32_t)hz, 10);
    ledcWrite(SELFTEST_PIN, 512);                   /* 50% duty */
}

static void camTask(void *)
{
    rows_msg_t m;
    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;
        m.ms = millis();
        const int w = fb->width;
        m.n = fb->height < BD_ROWS_MAX ? fb->height : BD_ROWS_MAX;
        for (int y = 0; y < m.n; y++) {              /* average each full row */
            const uint8_t *row = fb->buf + y * w;
            uint32_t sum = 0;
            for (int x = 0; x < w; x++) sum += row[x];
            m.rows[y] = (float)sum / (float)w;
        }
        esp_camera_fb_return(fb);
        if (xQueueSend(rows_q, &m, 0) != pdTRUE) dropped++;   /* never block capture */
    }
}

static int cmp_float(const void *a, const void *b)
{
    float d = *(const float *)a - *(const float *)b;
    return (d > 0) - (d < 0);
}

static void dspTask(void *)
{
    static rows_msg_t m;
    static float bg[BD_ROWS_MAX], out[BD_ROWS_MAX], cal[CAL_FRAMES];
    bool first = true; int ncal = 0;
    for (;;) {
        if (xQueueReceive(rows_q, &m, portMAX_DELAY) != pdTRUE) continue;
        bd_rows_remove_background(m.rows, bg, out, m.n, BG_ALPHA, first);
        first = false;

        if (calibrating) {
            /* LED at a known 400 Hz: stripes per row give the row rate */
            float cpr = bd_cycles_per_row(out, m.n);
            if (cpr > 0.0f && ncal < CAL_FRAMES) cal[ncal++] = cpr;
            if (ncal == CAL_FRAMES) {
                qsort(cal, CAL_FRAMES, sizeof(float), cmp_float);
                row_rate = CAL_HZ / cal[CAL_FRAMES / 2];     /* median: robust to bad frames */
                Serial.printf("calibrated: %.4f cycles/row at %.0f Hz -> row rate %.0f Hz\n",
                              cal[CAL_FRAMES / 2], CAL_HZ, row_rate);
                calibrating = false;
            }
            continue;
        }

        uint32_t c0 = ESP.getCycleCount();
        bd_result_t r = bd_analyze_rows(out, m.n, row_rate);
        decide_and_log(r, row_rate, (ESP.getCycleCount() - c0) / (uint32_t)ESP.getCpuFreqMHz(), "row_rate");
        static uint32_t last_dropped = 0;
        if (dropped != last_dropped) { last_dropped = dropped; Serial.printf("  (frames dropped so far: %lu)\n", (unsigned long)dropped); }
    }
}

#if SELFTEST
static void blinkTask(void *)
{
    led_hz(CAL_HZ);                                 /* calibration first */
    while (calibrating) vTaskDelay(pdMS_TO_TICKS(100));
    const float hz[4] = { 0.0f, 200.0f, 400.0f, 600.0f };   /* NONE, LEFT, RIGHT, STOP: 8 s each */
    for (int phase = 0;; phase = (phase + 1) % 4) {
        truth = BD_CMD_NONE;
        led_hz(hz[phase]);
        vTaskDelay(pdMS_TO_TICKS(3000));            /* let background + debounce settle */
        truth = (bd_cmd_t)phase;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif
#endif

/* ================= setup ================= */
void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
    bd_init();
    if (!camera_init()) { Serial.println("camera init failed"); while (true) delay(1000); }

#if MODE == MODE_ROWS
    rows_q = xQueueCreate(3, sizeof(rows_msg_t));
#if SELFTEST
    ledcAttach(SELFTEST_PIN, (uint32_t)CAL_HZ, 10);
#endif
#else
#if SELFTEST
    pinMode(SELFTEST_PIN, OUTPUT);
#endif
#endif

#if SELFTEST
    xTaskCreatePinnedToCore(blinkTask, "blink", 3072, nullptr, 2, nullptr, 0);
#endif
    xTaskCreatePinnedToCore(camTask, "cam", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(dspTask, "dsp", 8192, nullptr, 4, nullptr, 0);
    Serial.printf("blink-fft running in %s mode\n", MODE == MODE_ROWS ? "ROWS (rolling shutter)" : "FRAME");
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
