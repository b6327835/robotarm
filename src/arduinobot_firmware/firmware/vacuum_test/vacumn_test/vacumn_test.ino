// Depth sensor pin for detecting object presence/distance
const int depthSensorPin = 14;
const int vacuumInPin = 32;
const int vacuumOutPin = 33;

const int NUM_READINGS = 10;
const int SAMPLES_PER_READING = 4;  // Number of samples to take for each reading
const int ADC_MAX_RESOLUTION = 12;  // 12-bit resolution
int readings[NUM_READINGS];
int readIndex = 0;

unsigned long depthSensorActivatedTime = 0;
unsigned long vacuumActivatedTime = 0;
bool vacuumInActive = false;
bool vacuumOutActive = false;

void setup() {
    analogReadResolution(ADC_MAX_RESOLUTION);  // Set ADC to 12-bit resolution
    // Initialize readings array to avoid garbage values
    for(int i = 0; i < NUM_READINGS; i++) {
        readings[i] = 0;
    }
    pinMode(depthSensorPin, INPUT);
    pinMode(vacuumInPin, OUTPUT);
    pinMode(vacuumOutPin, OUTPUT);
    digitalWrite(vacuumInPin, LOW);
    digitalWrite(vacuumOutPin, LOW);
    Serial.begin(115200);
    delay(100);
}

void loop() {
    delay(50);
    int currentReading = getAveragedReading();
    unsigned long currentTime = millis();

    // Print depth sensor value
    Serial.print("Averaged depth sensor value: ");
    Serial.println(currentReading);

    if (currentReading < 200) {
        if (depthSensorActivatedTime == 0) {
            depthSensorActivatedTime = currentTime;
        } else if (currentTime - depthSensorActivatedTime >= 5000) {
            activateVacuumIn();
        }
        Serial.print("Object detected for ");
        Serial.print(currentTime - depthSensorActivatedTime);
        Serial.println(" milliseconds");
    } else {
        depthSensorActivatedTime = 0;
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

int getAveragedReading() {
    // Take multiple samples and average them for each reading
    long sampleSum = 0;
    for(int i = 0; i < SAMPLES_PER_READING; i++) {
        sampleSum += analogRead(depthSensorPin);
        delayMicroseconds(100);  // Short delay between samples
    }
    int currentReading = sampleSum / SAMPLES_PER_READING;
    
    // Add to rolling average
    readings[readIndex] = currentReading;
    readIndex = (readIndex + 1) % NUM_READINGS;
    
    // Calculate final average
    long sum = 0;
    for (int i = 0; i < NUM_READINGS; i++) {
        sum += readings[i];
    }
    return sum / NUM_READINGS;
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
