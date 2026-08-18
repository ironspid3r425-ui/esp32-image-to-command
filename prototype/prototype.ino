#include <Arduino.h>
#include <ESP32QRCodeReader.h>
#include "board_config.h"
#include "esp_camera.h"

#define LED_PIN 21  // change if your LED is on a different pin

// Correct object declaration using the XIAO ESP32S3 camera model
ESP32QRCodeReader reader{CAMERA_MODEL_XIAO_ESP32S3};

void signalEvent(const char *msg) {
  digitalWrite(LED_PIN, HIGH);
  Serial.println(msg);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

void onQrCodeTask(void *pvParameters)
{
  struct QRCodeData qrCodeData;

  while (true)
  {
    Serial.println("QR task running...");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    if (reader.receiveQrCode(&qrCodeData, 100))
    {
      Serial.println("Found QRCode");
      if (qrCodeData.valid)
      {
        Serial.print("Payload: ");
        Serial.println((const char *)qrCodeData.payload);
        signalEvent("QR detected - LED toggled");
      }
      else
      {
        Serial.print("Invalid: ");
        Serial.println((const char *)qrCodeData.payload);
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  reader.setup();
  Serial.println("Setup QRCode Reader");

  reader.beginOnCore(1);
  Serial.println("Begin on Core 1");

  xTaskCreate(onQrCodeTask, "onQrCode", 4 * 1024, NULL, 4, NULL);
}

void loop()
{
  delay(100);
}
