void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Flash Size:");
  Serial.println(ESP.getFlashChipSize());
}

void loop() {
  Serial.println("ESP32 Flash Size:");
  Serial.println(ESP.getFlashChipSize());
  delay(100);
}
