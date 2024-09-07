#define LED_PIN 2
String y = "ON";
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, INPUT_PULLUP); 

  Serial.begin(115200);
  Serial.setTimeout(1);
}
void loop() {
  if (Serial.available())
  {
    int x = Serial.readString().toInt();
    if(x == 0)
    {
      digitalWrite(LED_PIN, LOW);
      y = "OFF";
      Serial.println("OFF");
    }
    else
    {
      digitalWrite(LED_PIN, HIGH);
      y = "ON";
      Serial.println("ON") ;
    }
  }
  delay(500);
  Serial.print("LED:= ");
  Serial.println(y);
}
