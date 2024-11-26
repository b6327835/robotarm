const int sensorPin = 20;
const int vacuumInPin = 32;
const int vacuumOutPin = 33;

unsigned long sensorActivatedTime = 0;
unsigned long vacuumActivatedTime = 0;
bool vacuumInActive = false;
bool vacuumOutActive = false;

void setup() {
    pinMode(sensorPin, INPUT);
    pinMode(vacuumInPin, OUTPUT);
    pinMode(vacuumOutPin, OUTPUT);
    digitalWrite(vacuumInPin, LOW);
    digitalWrite(vacuumOutPin, LOW);
    Serial.begin(115200);
}

void loop() {
    int sensorValue = digitalRead(sensorPin);
    unsigned long currentTime = millis();

    // Print sensor status
    Serial.print("Sensor status: ");
    if (sensorValue == HIGH) {
        Serial.println("HIGH");
    } else {
        Serial.println("LOW");
    }

    if (sensorValue == HIGH) {
        if (sensorActivatedTime == 0) {
            sensorActivatedTime = currentTime;
        } else if (currentTime - sensorActivatedTime >= 5000) {
            activateVacuumIn();
        }
        Serial.print("Sensor has been HIGH for ");
        Serial.print(currentTime - sensorActivatedTime);
        Serial.println(" milliseconds");
    } else {
        sensorActivatedTime = 0;
    }

    if (vacuumInActive && currentTime - vacuumActivatedTime >= 1000) {
        deactivateVacuumIn();
    }

    if (vacuumOutActive && currentTime - vacuumActivatedTime >= 1000) {
        deactivateVacuumOut();
    }

    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command == "in") {
            activateVacuumIn();
        } else if (command == "out") {
            activateVacuumOut();
        } else if (command == "off") {
            deactivateVacuumIn();
            deactivateVacuumOut();
        }
    }
}

void activateVacuumIn() {
    digitalWrite(vacuumInPin, HIGH);
    vacuumActivatedTime = millis();
    vacuumInActive = true;
}

void deactivateVacuumIn() {
    digitalWrite(vacuumInPin, LOW);
    vacuumInActive = false;
}

void activateVacuumOut() {
    digitalWrite(vacuumOutPin, HIGH);
    vacuumActivatedTime = millis();
    vacuumOutActive = true;
}

void deactivateVacuumOut() {
    digitalWrite(vacuumOutPin, LOW);
    vacuumOutActive = false;
}