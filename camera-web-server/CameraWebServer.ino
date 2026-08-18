#include "esp_camera.h"
#include <ESP32QRCodeReader.h>

#define LED_PIN 21               // XIAO ESP32S3 user LED (active-low)
#define LED_ON()  digitalWrite(LED_PIN, LOW)
#define LED_OFF() digitalWrite(LED_PIN, HIGH)

// Use the XIAO model you defined in ESP32CameraPins.h
// (do NOT redefine it as (100) here)
CameraPins xiaoPins = CAMERA_MODEL_XIAO_ESP32S3;

// Tell the QR library to use those pins and QVGA frame size
ESP32QRCodeReader reader(xiaoPins, FRAMESIZE_QVGA);

void blinkEvent(const char* msg) {
  Serial.println(msg);
  LED_ON();  delay(150);
  LED_OFF(); delay(100);
}

void onQrCodeTask(void* pv) {
  QRCodeData qr;
  static String last;
  static uint32_t invalidCount = 0;

  for (;;) {
    if (reader.receiveQrCode(&qr, 100)) {
      if (qr.valid) {
        String payload = String((const char*)qr.payload);
        invalidCount = 0;  // reset on any valid QR

        if (payload != last) {       // debounce on change
          Serial.print("QR: ");
          Serial.println(payload);

          if (payload == "TRIGGER") {
            static bool ledState = false;
            ledState = !ledState;
            if (ledState) LED_ON(); else LED_OFF();
            Serial.println("Command: TOGGLE_LED");
          } else if (payload == "LED_ON") {
            LED_ON();
            Serial.println("Command: LED_ON");
          } else if (payload == "LED_OFF") {
            LED_OFF();
            Serial.println("Command: LED_OFF");
          } else {
            // Known “error” case: payload not in whitelist
            Serial.println("Command: UNKNOWN (ignored)");
          }

          last = payload;
        }
      } else {
        // Library says “not a valid QR” – treat as a soft error
        invalidCount++;
        if (invalidCount % 50 == 0) {   // log only occasionally
          Serial.println("Warning: many invalid QR frames, no action taken.");
        }
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}



void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  LED_OFF();   // LED off (active-low)

  // Initialize camera via the QR library (uses the XIAO pins)
  QRCodeReaderSetupErr err = reader.setup();
  Serial.print("reader.setup() = ");
  Serial.println((int)err);

  // Turn on library debug to see what's happening
  reader.setDebug(false);

  // Optional: tune sensor after init (keep QVGA for speed)
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  // Run capture/decoder on core 1 and start our task
  reader.beginOnCore(1);
  xTaskCreate(onQrCodeTask, "qr_task", 4096, nullptr, 4, nullptr);

  Serial.println("Ready: show a QR code to the camera.");
}


void loop() {
  // Nothing here; QR reading runs in its own task
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
