#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
uint32_t t0;

void setup() {
  Serial.begin(921600);
  while (!Serial) { delay(1); }

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  t0 = millis();
  Serial.println("time_ms\tcurrent_mA\tpower_mW");
}

void loop() {
  uint32_t t = millis() - t0;
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();

  Serial.print(t);
  Serial.print("\t");
  Serial.print(current_mA);
  Serial.print("\t");
  Serial.println(power_mW);

//   delay(100);
}