#include "FastAccelStepper.h"
//MAX x10y6.5,z7.1
// Create FastAccelStepper engine
FastAccelStepperEngine engine = FastAccelStepperEngine();

// Create stepper instances
FastAccelStepper *stepperX = NULL;
FastAccelStepper *stepperY = NULL;
FastAccelStepper *stepperZ = NULL;

// Pin Definitions
const int X_STEP_PIN = 25;
const int X_DIR_PIN = 5;
const int Y_STEP_PIN = 26;
const int Y_DIR_PIN = 18;
const int Z_STEP_PIN = 27;
const int Z_DIR_PIN = 19;

const int PIN_LIMIT_X = 21;
const int PIN_LIMIT_Y = 22;
const int PIN_LIMIT_Z = 23;

const int LED_PIN = 2;  // Update this to match your LED pin
const int32_t HOMING_STEPS = 10000;
const uint32_t HOMING_TIMEOUT = 10000;  // 10 seconds timeout

// Add homing state variable
bool isHomed = false;
const int32_t HOMING_SPEED = 20000;     // Increased from 20000 for smoother motion
const int32_t HOMING_ACCEL = 90000;     // Reduced from 320000 for gentler starts
const int32_t BACKOFF_STEPS = 0;      // Steps to back off after hitting limit

// Motor Configuration Constants
const float STEP_ANGLE = 0.36;                    // Degrees per step
const int STEPS_PER_REV = (int)(360 / STEP_ANGLE); // 1000 steps per revolution

// Fixed configuration parameters
const float MICROSTEPPING = 16.0;
const float LEAD_SCREW_PITCH = 2.0;
const int32_t MAX_SPEED = 40000;
const int32_t ACCELERATION = 160000;

// Maximum coordinate limits
const float MAX_X = 10.0;
const float MAX_Y = 6.5;
const float MAX_Z = 7.1;

void setup() {
  Serial.begin(250000);
  
  // Initialize engine
  engine.init();
  
  // Initialize steppers
  stepperX = engine.stepperConnectToPin(X_STEP_PIN);
  stepperY = engine.stepperConnectToPin(Y_STEP_PIN);
  stepperZ = engine.stepperConnectToPin(Z_STEP_PIN);
  
  if (stepperX && stepperY && stepperZ) {
    // Set default directions
    stepperX->setDirectionPin(X_DIR_PIN);
    stepperY->setDirectionPin(Y_DIR_PIN);
    stepperZ->setDirectionPin(Z_DIR_PIN);
    
    // Setup limit switch pins
    pinMode(PIN_LIMIT_X, INPUT_PULLUP);
    pinMode(PIN_LIMIT_Y, INPUT_PULLUP);
    pinMode(PIN_LIMIT_Z, INPUT_PULLUP);
    Serial.println("Send anything to start homing.");
  } else {
    Serial.println("Error initializing steppers!");
  }
}

// Convert millimeters to steps with improved precision
int32_t mmToSteps(float mm, float stepsPerMm) {
  return (int32_t)(mm * stepsPerMm + 0.5f); // Added 0.5 for proper rounding
}

// Updated function to parse coordinates
bool parseCoordinates(String input, float &x, float &y, float &z) {
  // Remove any whitespace and ensure consistent format
  input.trim();
  if (!input.endsWith(",")) {
    input += ",";
  }
  
  // Find the position of coordinate markers
  int xStart = input.indexOf('x') + 1;
  int yStart = input.indexOf('y') + 1;
  int zStart = input.indexOf('z') + 1;
  
  if (xStart <= 0 || yStart <= 0 || zStart <= 0) {
    Serial.println("Error: Missing coordinate markers");
    return false;
  }
  
  // Find the end positions (next marker or comma)
  int xEnd = input.indexOf('y');
  int yEnd = input.indexOf('z');
  int zEnd = input.indexOf(',', zStart);
  
  if (xEnd == -1 || yEnd == -1 || zEnd == -1) {
    Serial.println("Error: Invalid coordinate format");
    return false;
  }
  
  // Extract and clean substrings
  String xStr = input.substring(xStart, xEnd);
  String yStr = input.substring(yStart, yEnd);
  String zStr = input.substring(zStart, zEnd);
  
  // Remove any trailing commas
  xStr.replace(",", "");
  yStr.replace(",", "");
  zStr.replace(",", "");
  
  // Convert to float values
  x = xStr.toFloat();
  y = yStr.toFloat();
  z = zStr.toFloat();
  
  // Debug output
  Serial.printf("Raw substrings - X:'%s' Y:'%s' Z:'%s'\n", xStr.c_str(), yStr.c_str(), zStr.c_str());
  Serial.printf("Parsed coordinates - X: %.3f, Y: %.3f, Z: %.3f\n", x, y, z);
  
  return true;
}

bool validateCoordinates(float x, float y, float z) {
  if (x < 0 || x > MAX_X || y < 0 || y > MAX_Y || z < 0 || z > MAX_Z) {
    Serial.printf("Error: Coordinates out of bounds. Limits are: X:0-%0.1f Y:0-%0.1f Z:0-%0.1f\n", 
                 MAX_X, MAX_Y, MAX_Z);
    return false;
  }
  return true;
}

void moveToPosition(float x, float y, float z, float microstepping, float leadScrewPitch, int32_t maxSpeed, int32_t acceleration) {
  if (!validateCoordinates(x, y, z)) {
    return;
  }
  
  float stepsPerMm = (STEPS_PER_REV * microstepping) / leadScrewPitch;

  // Configure speed and acceleration
  stepperX->setSpeedInHz(maxSpeed);
  stepperY->setSpeedInHz(maxSpeed);
  stepperZ->setSpeedInHz(maxSpeed);
  
  stepperX->setAcceleration(acceleration);
  stepperY->setAcceleration(acceleration);
  stepperZ->setAcceleration(acceleration);
  
  // Convert millimeters to steps
  int32_t xSteps = mmToSteps(x, stepsPerMm);
  int32_t ySteps = mmToSteps(y, stepsPerMm);
  int32_t zSteps = mmToSteps(z, stepsPerMm);
  
  // Debug output for steps calculation
  Serial.printf("Steps per mm: %.2f\n", stepsPerMm);
  Serial.printf("Target steps - X:%d Y:%d Z:%d\n", xSteps, ySteps, zSteps);
  
  // Move all motors to target positions
  stepperX->moveTo(xSteps);
  stepperY->moveTo(ySteps);
  stepperZ->moveTo(zSteps);
  
  // Monitor movement with debug output
  while (stepperX->isRunning() || stepperY->isRunning() || stepperZ->isRunning()) {
    Serial.printf("Current positions - X:%d Y:%d Z:%d\n", 
                 stepperX->getCurrentPosition(),
                 stepperY->getCurrentPosition(),
                 stepperZ->getCurrentPosition());
    delay(100);  // Increased delay for readable debug output
  }
  
  Serial.println("Movement complete");
}

void blinkError() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

bool homeOneAxis(FastAccelStepper* stepper, int limitPin, const char* axisName) {
  Serial.printf("Homing %s axis...\n", axisName);
  uint32_t startTime = millis();
  
  // Set direction for continuous movement
  stepper->setSpeedInHz(HOMING_SPEED);
  stepper->setAcceleration(HOMING_ACCEL);
  stepper->moveTo(-100000); // Move a large distance in negative direction
  
  // Monitor movement until limit switch is hit
  while (digitalRead(limitPin) == HIGH) {
    if (millis() - startTime > HOMING_TIMEOUT) {
      Serial.printf("Error: %s axis homing timeout!\n", axisName);
      stepper->forceStop();
      return false;
    }
    delay(1);
  }
  
  // Stop smoothly when limit is hit
  stepper->forceStopAndNewPosition(0);
  
  if (BACKOFF_STEPS > 0) {
    // Optional: Move away from limit switch if BACKOFF_STEPS is set
    stepper->moveTo(BACKOFF_STEPS);
    while (stepper->isRunning()) delay(1);
  }
  
  return true;
}

void homeAxis() {
  Serial.println("Starting homing sequence...");
  pinMode(LED_PIN, OUTPUT);
  
  // Configure homing speed and acceleration for all steppers
  stepperX->setSpeedInHz(HOMING_SPEED);
  stepperY->setSpeedInHz(HOMING_SPEED);
  stepperZ->setSpeedInHz(HOMING_SPEED);
  
  stepperX->setAcceleration(HOMING_ACCEL);
  stepperY->setAcceleration(HOMING_ACCEL);
  stepperZ->setAcceleration(HOMING_ACCEL);
  
  // Home Z axis first (for safety)
  if (!homeOneAxis(stepperZ, PIN_LIMIT_Z, "Z")) {
    blinkError();
    return;
  }
  
  // Home Y axis
  if (!homeOneAxis(stepperY, PIN_LIMIT_Y, "Y")) {
    blinkError();
    return;
  }
  
  // Home X axis
  if (!homeOneAxis(stepperX, PIN_LIMIT_X, "X")) {
    blinkError();
    return;
  }
  
  Serial.println("Homing complete!");
  isHomed = true;
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    
    // Check if homing is needed
    if (!isHomed) {
      Serial.println("Performing initial homing...");
      homeAxis();
      Serial.println("Ready for commands");
      return;
    }
    
    float x, y, z;
    if (parseCoordinates(input, x, y, z)) {
      moveToPosition(x, y, z, MICROSTEPPING, LEAD_SCREW_PITCH, MAX_SPEED, ACCELERATION);
    }
  }
}
