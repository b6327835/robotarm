// Depth sensor pin for detecting object presence/distance
const int depthSensorPin = 14;
const int vacuumInPin = 32;
const int vacuumOutPin = 33;

const int DEBOUNCE_DELAY = 50;    // Debounce time in milliseconds
const int READING_COUNT = 5;       // Number of readings to average
const int STABLE_THRESHOLD = 20;   // Maximum difference between readings to be considered stable

unsigned long depthSensorActivatedTime = 0;
unsigned long vacuumActivatedTime = 0;
bool vacuumInActive = false;
bool vacuumOutActive = false;

unsigned long lastDebounceTime = 0;
int lastSensorStableValue = 0;
int sensorReadings[READING_COUNT];
int readingIndex = 0;

void setup() {
    pinMode(depthSensorPin, INPUT);
    pinMode(vacuumInPin, OUTPUT);
    pinMode(vacuumOutPin, OUTPUT);
    digitalWrite(vacuumInPin, LOW);
    digitalWrite(vacuumOutPin, LOW);
    Serial.begin(115200);
    delay(100);
}

void loop() {
    delay(50);  // Reduced delay since we're using debouncing
    int currentReading = getStableSensorReading();
    unsigned long currentTime = millis();

    // Print depth sensor value
    Serial.print("Stable depth sensor value: ");
    Serial.println(currentReading);

    if (currentReading < 200) {
        if (depthSensorActivatedTime == 0 && (currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
            depthSensorActivatedTime = currentTime;
            lastDebounceTime = currentTime;
        } else if (currentTime - depthSensorActivatedTime >= 5000) {
            activateVacuumIn();
        }
        Serial.print("Object detected for ");
        Serial.print(currentTime - depthSensorActivatedTime);
        Serial.println(" milliseconds");
    } else {
        if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
            depthSensorActivatedTime = 0;
            lastDebounceTime = currentTime;
        }
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

int getStableSensorReading() {
    // Add new reading to array
    sensorReadings[readingIndex] = analogRead(depthSensorPin);
    readingIndex = (readingIndex + 1) % READING_COUNT;

    // Calculate average
    int sum = 0;
    for (int i = 0; i < READING_COUNT; i++) {
        sum += sensorReadings[i];
    }
    int average = sum / READING_COUNT;

    // Check stability
    bool isStable = true;
    for (int i = 0; i < READING_COUNT; i++) {
        if (abs(sensorReadings[i] - average) > STABLE_THRESHOLD) {
            isStable = false;
            break;
        }
    }

    if (isStable) {
        lastSensorStableValue = average;
    }

    return lastSensorStableValue;
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