#define X_STEP_PIN 25  
#define X_DIR_PIN 5    
#define Y_STEP_PIN 26  
#define Y_DIR_PIN 18   
#define Z_STEP_PIN 27  
#define Z_DIR_PIN 19   
#define LEDPIN 2

#define pinLimitX 21
#define pinLimitY 22
#define pinLimitZ 23

float stepsPerMM = 266.0;  
int pulseDelay = 100;   
int directionDelay = 5;   

float lastX = 0.0, lastY = 0.0, lastZ = 0.0;

// Mapping ranges for X, Y, Z
float xMin = -125, xMax = 120, xTargetMin = 0, xTargetMax = 300;
float yMin = -51, yMax = 118, yTargetMin = 0, yTargetMax = 200;
float zMin = -54, zMax = 115, zTargetMin = 0, zTargetMax = 200;

void setup() {
  Serial.begin(115200);

  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);

  pinMode(pinLimitX, INPUT_PULLUP);
  pinMode(pinLimitY, INPUT_PULLUP);
  pinMode(pinLimitZ, INPUT_PULLUP);

  Serial.println("Homing...");
  homeAxes();

  Serial.println("Ready to receive XYZ commands.");
}

void loop() {
  static unsigned long lastCommandTime = 0;
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (millis() - lastCommandTime > 100) { // Adjust time interval as needed
      lastCommandTime = millis();
      parseAndExecuteCommand(input);
    }
  }
  
  delay(10); // Reduced delay for responsiveness
}

void homeAxes() {
  // Home the X axis
  digitalWrite(X_DIR_PIN, LOW); // Assume LOW moves towards the sensor
  while (digitalRead(pinLimitX) == HIGH) {
    step(X_STEP_PIN);
  }
  lastX = 0;  // Set home position for X
  digitalWrite(X_DIR_PIN, HIGH);

  // Home the Y axis
  digitalWrite(Y_DIR_PIN, LOW); // Assume LOW moves towards the sensor
  while (digitalRead(pinLimitY) == HIGH) {
    step(Y_STEP_PIN);
  }
  lastY = 0;  // Set home position for Y

  // Home the Z axis
  digitalWrite(Z_DIR_PIN, LOW); // Assume LOW moves towards the sensor
  while (digitalRead(pinLimitZ) == HIGH) {
    step(Z_STEP_PIN);
  }
  lastZ = 0;  // Set home position for Z

  Serial.println("Homing complete.");
}

void step(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(pulseDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(pulseDelay);
}

void parseAndExecuteCommand(String input) {
  float rawX = lastX, rawY = lastY, rawZ = lastZ;

  if (input.indexOf('x') != -1) {
    rawX = getValueAfterChar(input, 'x');
  }
  if (input.indexOf('y') != -1) {
    rawY = getValueAfterChar(input, 'y');
  }
  if (input.indexOf('z') != -1) {
    rawZ = getValueAfterChar(input, 'z');
  }

  // Map raw values to target range (mm)
  float targetX = mapValue(rawX, xMin, xMax, xTargetMin, xTargetMax);
  float targetY = mapValue(rawY, yMin, yMax, yTargetMin, yTargetMax);
  float targetZ = mapValue(rawZ, zMin, zMax, zTargetMin, zTargetMax);

  // Adjust target positions based on the home position
  float adjustedX = targetX - lastX;
  float adjustedY = targetY - lastY;
  float adjustedZ = targetZ - lastZ;

  // Calculate steps and directions
  long stepsX = convertMMToSteps(abs(adjustedX));
  long stepsY = convertMMToSteps(abs(adjustedY));
  long stepsZ = convertMMToSteps(abs(adjustedZ));

  int dirX = (adjustedX > 0) ? HIGH : LOW;
  int dirY = (adjustedY > 0) ? HIGH : LOW;
  int dirZ = (adjustedZ > 0) ? HIGH : LOW;

  // Move motors
  moveAxesSimultaneously(stepsX, stepsY, stepsZ, dirX, dirY, dirZ);

  // Update positions
  lastX = targetX;
  lastY = targetY;
  lastZ = targetZ;
}

float mapValue(float rawValue, float inMin, float inMax, float outMin, float outMax) {
  return (rawValue - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float clamp(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

long convertMMToSteps(float mm) {
  return mm * stepsPerMM;
}

float getValueAfterChar(String input, char key) {
  int startIndex = input.indexOf(key) + 1;
  int endIndex = input.indexOf(',', startIndex);
  if (endIndex == -1) {
    endIndex = input.length();
  }
  return input.substring(startIndex, endIndex).toFloat();
}

void moveAxesSimultaneously(long stepsX, long stepsY, long stepsZ, int dirX, int dirY, int dirZ) {
  digitalWrite(X_DIR_PIN, dirX);
  digitalWrite(Y_DIR_PIN, dirY);
  digitalWrite(Z_DIR_PIN, dirZ);

  long maxSteps = max(stepsX, max(stepsY, stepsZ));

  for (long i = 0; i < maxSteps; i++) {
    if (i < stepsX) {
      digitalWrite(X_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(X_STEP_PIN, LOW);
    }

    if (i < stepsY) {
      digitalWrite(Y_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(Y_STEP_PIN, LOW);
    }

    if (i < stepsZ) {
      digitalWrite(Z_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(Z_STEP_PIN, LOW);
    }
    
    delayMicroseconds(pulseDelay);
  }

  Serial.print("Final X (steps): "); Serial.println(stepsX);
  Serial.print("Final Y (steps): "); Serial.println(stepsY);
  Serial.print("Final Z (steps): "); Serial.println(stepsZ);
}
