const int PIN_32 = 32;
const int PIN_33 = 33;
const int TEMP_DELAY = 300;  // 0.3 seconds
bool pin32Active = true;  // Start HIGH
bool pin33Active = true;  // Start HIGH

void setup() {
    pinMode(PIN_32, OUTPUT);
    pinMode(PIN_33, OUTPUT);
    digitalWrite(PIN_32, HIGH);
    digitalWrite(PIN_33, HIGH);
    Serial.begin(115200);
    delay(100);
}

void loop() {
    delay(50);
    
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "high") {
            pin32Active = true;
            pin33Active = true;
            digitalWrite(PIN_32, HIGH);
            digitalWrite(PIN_33, HIGH);
            Serial.println("Both pins activated");
        } else if (command == "low") {
            pin32Active = false;
            pin33Active = false;
            digitalWrite(PIN_32, LOW);
            digitalWrite(PIN_33, LOW);
            Serial.println("Both pins deactivated");
        } else if (command == "in") {
            digitalWrite(PIN_32, LOW);
            digitalWrite(PIN_33, HIGH);
            delay(TEMP_DELAY);
            digitalWrite(PIN_32, HIGH);
            digitalWrite(PIN_33, HIGH);
            Serial.println("In motion executed");
        } else if (command == "out") {
            digitalWrite(PIN_32, HIGH);
            digitalWrite(PIN_33, LOW);
            delay(TEMP_DELAY);
            digitalWrite(PIN_32, HIGH);
            digitalWrite(PIN_33, HIGH);
            Serial.println("Out motion executed");
        } else {
            int pin = command.toInt();
            if (pin == PIN_32) {
                if (pin32Active) {
                    // If 32 is HIGH, make it LOW and 33 HIGH
                    pin32Active = false;
                    pin33Active = true;
                    digitalWrite(PIN_32, LOW);
                    digitalWrite(PIN_33, HIGH);
                } else {
                    // If 32 is LOW, make both HIGH
                    pin32Active = true;
                    pin33Active = true;
                    digitalWrite(PIN_32, HIGH);
                    digitalWrite(PIN_33, HIGH);
                }
                Serial.println("Pin 32 toggled");
            } else if (pin == PIN_33) {
                if (pin33Active) {
                    // If 33 is HIGH, make it LOW and 32 HIGH
                    pin33Active = false;
                    pin32Active = true;
                    digitalWrite(PIN_33, LOW);
                    digitalWrite(PIN_32, HIGH);
                } else {
                    // If 33 is LOW, make both HIGH
                    pin32Active = true;
                    pin33Active = true;
                    digitalWrite(PIN_32, HIGH);
                    digitalWrite(PIN_33, HIGH);
                }
                Serial.println("Pin 33 toggled");
            }
        }
    }
}
