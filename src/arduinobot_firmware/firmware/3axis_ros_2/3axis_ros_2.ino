#define X_STEP_PIN 25  
#define X_DIR_PIN 5    
#define Y_STEP_PIN 26  
#define Y_DIR_PIN 18   
#define Z_STEP_PIN 27  
#define Z_DIR_PIN 19   
#define LEDPIN 2

int pulseDelay = 100;   
float stepsPerMM = 10.0;  
int directionDelay = 5;   
int minStepThreshold = 1; // Minimum number of steps required to move (at least 1)

float lastX = 0.0, lastY = 0.0, lastZ = 0.0;

// Mapping ranges for X, Y, Z
float xMin = -125, xMax = 120, xTargetMin = 0, xTargetMax = 300;
float yMin = -051, yMax = 118, yTargetMin = 0, yTargetMax = 200;
float zMin = -054, zMax = 115, zTargetMin = 0, zTargetMax = 200;

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

  Serial.println("Ready to receive XYZ commands.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    Serial.println(input);
    parseAndMove(input);
  }
  delay(100);
}

void parseAndMove(String input) {
  float rawX = lastX, rawY = lastY, rawZ = lastZ;

  // Parse X, Y, Z values from input
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

  // Validate the mapped values (clamp them within physical limits)
  targetX = clamp(targetX, xTargetMin, xTargetMax);
  targetY = clamp(targetY, yTargetMin, yTargetMax);
  targetZ = clamp(targetZ, zTargetMin, zTargetMax);

  // Calculate steps and directions
  long stepsX = convertMMToSteps(abs(targetX - lastX));
  long stepsY = convertMMToSteps(abs(targetY - lastY));
  long stepsZ = convertMMToSteps(abs(targetZ - lastZ));

  // Check if the steps are less than the minimum step threshold, adjust
  stepsX = (stepsX < minStepThreshold && stepsX > 0) ? minStepThreshold : stepsX;
  stepsY = (stepsY < minStepThreshold && stepsY > 0) ? minStepThreshold : stepsY;
  stepsZ = (stepsZ < minStepThreshold && stepsZ > 0) ? minStepThreshold : stepsZ;

  int dirX = (targetX > lastX) ? HIGH : LOW;
  int dirY = (targetY > lastY) ? HIGH : LOW;
  int dirZ = (targetZ > lastZ) ? HIGH : LOW;

  // Move motors with acceleration profile
  moveAxesWithAcceleration(stepsX, stepsY, stepsZ, dirX, dirY, dirZ);

  // Update positions
  lastX = targetX;
  lastY = targetY;
  lastZ = targetZ;

  digitalWrite(LEDPIN, LOW);
}

// Map raw input value to a target range
float mapValue(float rawValue, float inMin, float inMax, float outMin, float outMax) {
  return (rawValue - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// Clamp the value within the min-max range
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

// Function to move axes with acceleration profile
void moveAxesWithAcceleration(long stepsX, long stepsY, long stepsZ, int dirX, int dirY, int dirZ) {
  digitalWrite(X_DIR_PIN, dirX);
  digitalWrite(Y_DIR_PIN, dirY);
  digitalWrite(Z_DIR_PIN, dirZ);
  delay(directionDelay);

  long currentX = 0, currentY = 0, currentZ = 0;
  long maxSteps = max(stepsX, max(stepsY, stepsZ));

  int accelPhase = 500;  // Number of steps for acceleration/deceleration phase
  int constantPhase = maxSteps - (2 * accelPhase); // Constant speed phase
  if (constantPhase < 0) {
    accelPhase = maxSteps / 2;
    constantPhase = 0;
  }

  // Acceleration phase
  for (long i = 0; i < accelPhase; i++) {
    int accelDelay = map(i, 0, accelPhase, pulseDelay * 2, pulseDelay);  // Gradually decrease delay
    moveStep(stepsX, &currentX, X_STEP_PIN, accelDelay);
    moveStep(stepsY, &currentY, Y_STEP_PIN, accelDelay);
    moveStep(stepsZ, &currentZ, Z_STEP_PIN, accelDelay);
  }

  // Constant speed phase
  for (long i = 0; i < constantPhase; i++) {
    moveStep(stepsX, &currentX, X_STEP_PIN, pulseDelay);
    moveStep(stepsY, &currentY, Y_STEP_PIN, pulseDelay);
    moveStep(stepsZ, &currentZ, Z_STEP_PIN, pulseDelay);
  }

  // Deceleration phase
  for (long i = 0; i < accelPhase; i++) {
    int decelDelay = map(i, 0, accelPhase, pulseDelay, pulseDelay * 2);  // Gradually increase delay
    moveStep(stepsX, &currentX, X_STEP_PIN, decelDelay);
    moveStep(stepsY, &currentY, Y_STEP_PIN, decelDelay);
    moveStep(stepsZ, &currentZ, Z_STEP_PIN, decelDelay);
  }

  Serial.print("Final X (steps): "); Serial.println(currentX);
  Serial.print("Final Y (steps): "); Serial.println(currentY);
  Serial.print("Final Z (steps): "); Serial.println(currentZ);
}

// Helper function to move a step if needed
void moveStep(long steps, long* currentSteps, int stepPin, int delayTime) {
  if (*currentSteps < steps) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayTime);
    (*currentSteps)++;
  }
}
