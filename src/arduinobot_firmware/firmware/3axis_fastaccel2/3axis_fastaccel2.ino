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
const int VACUUM_PICK_PIN = 32;  // Vacuum pin for picking
const int VACUUM_PLACE_PIN = 33; // Vacuum pin for placing
const int32_t HOMING_STEPS = 10000;
const uint32_t HOMING_TIMEOUT = 10000;  // 10 seconds timeout

// Add homing state variable
bool isHomed = false;
const int32_t HOMING_SPEED = 20000;   // Reduced from 20000 for more precise stopping
const int32_t HOMING_ACCEL = 90000;  // Reduced for gentler movement
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

// ROS coordinate system limits
const float ROS_MAX_X = 240.0;
const float ROS_MAX_Y = 150.0;
const float ROS_MAX_Z = 120.0;

const int DEBOUNCE_DELAY = 10;
const int CONSISTENT_READINGS = 3;

// Add state tracking for vacuum
bool isPicking = false;
bool isPlacing = false;
const float VACUUM_TRIGGER_HEIGHT = 108.0; // Z height threshold in ROS coordinates

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
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
    
    // Initialize vacuum control pins
    pinMode(VACUUM_PICK_PIN, OUTPUT);
    pinMode(VACUUM_PLACE_PIN, OUTPUT);
    digitalWrite(VACUUM_PICK_PIN, HIGH);   // Off by default
    digitalWrite(VACUUM_PLACE_PIN, HIGH);   // Off by default
  } else {
    Serial.println("Error initializing steppers!");
  }
}

// Convert millimeters to steps with improved precision
int32_t mmToSteps(float mm, float stepsPerMm) {
  return (int32_t)(mm * stepsPerMm + 0.5f); // Added 0.5 for proper rounding
}

// Map ROS coordinates to robot coordinates
void mapROStoRobot(float &x, float &y, float &z) {
    x = (x / ROS_MAX_X) * MAX_X;
    y = (y / ROS_MAX_Y) * MAX_Y;
    z = (z / ROS_MAX_Z) * MAX_Z;
}

// Updated function to parse coordinates
bool parseCoordinates(String input, float &x, float &y, float &z, int &vacuum) {
  // Remove any whitespace and ensure consistent format
  input.trim();
  if (!input.endsWith(",")) {
    input += ",";
  }
  
  // Find the coordinate markers
  int lastX = input.lastIndexOf('x');
  int lastY = input.lastIndexOf('y');
  int lastZ = input.lastIndexOf('z');
  int lastV = input.lastIndexOf('v');
  
  if (lastX == -1 || lastY == -1 || lastZ == -1) {
    Serial.println("Error: Missing coordinate markers");
    return false;
  }
  
  // Find the end positions
  int xEnd = input.indexOf('y', lastX);
  int yEnd = input.indexOf('z', lastY);
  int zEnd = lastV != -1 ? input.indexOf('v', lastZ) : input.indexOf(',', lastZ);
  int vEnd = lastV != -1 ? input.indexOf(',', lastV) : -1;
  
  if (xEnd == -1 || yEnd == -1 || zEnd == -1) {
    Serial.println("Error: Invalid coordinate format");
    return false;
  }
  
  // Extract coordinates
  String xStr = input.substring(lastX + 1, xEnd);
  String yStr = input.substring(lastY + 1, yEnd);
  String zStr = input.substring(lastZ + 1, zEnd);
  String vStr = lastV != -1 ? input.substring(lastV + 1, vEnd) : "0";
  
  // Remove any trailing commas
  xStr.replace(",", "");
  yStr.replace(",", "");
  zStr.replace(",", "");
  vStr.replace(",", "");
  
  // Convert to values
  x = xStr.toFloat();
  y = yStr.toFloat();
  z = zStr.toFloat();
  vacuum = vStr.toInt();
  
  // Map ROS coordinates to robot coordinates
  mapROStoRobot(x, y, z);
  
  // Debug output
  Serial.printf("Raw substrings - X:'%s' Y:'%s' Z:'%s'\n", xStr.c_str(), yStr.c_str(), zStr.c_str());
  Serial.printf("Parsed coordinates - X: %.3f, Y: %.3f, Z: %.3f\n", x, y, z);
  Serial.printf("ROS coordinates - X: %.3f, Y: %.3f, Z: %.3f\n", x * (ROS_MAX_X/MAX_X), y * (ROS_MAX_Y/MAX_Y), z * (ROS_MAX_Z/MAX_Z));
  Serial.printf("Mapped coordinates - X: %.3f, Y: %.3f, Z: %.3f\n", x, y, z);
  
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

void moveToPosition(float x, float y, float z, int vacuum, float microstepping, float leadScrewPitch, int32_t maxSpeed, int32_t acceleration) {
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
    delay(100);
  }
  
  // Handle vacuum control
  switch (vacuum) {
    case 1: // Pick
      digitalWrite(VACUUM_PICK_PIN, LOW);   // Turn on picking vacuum
      digitalWrite(VACUUM_PLACE_PIN, HIGH); // Ensure placing is off
      isPicking = true;
      isPlacing = false;
      Serial.println("Picking vacuum ON");
      break;
    case 2: // Place
      digitalWrite(VACUUM_PICK_PIN, HIGH);  // Turn off picking vacuum
      digitalWrite(VACUUM_PLACE_PIN, LOW);  // Turn on placing vacuum
      isPicking = false;
      isPlacing = true;
      Serial.println("Placing vacuum ON");
      break;
    case 0: // Off
      digitalWrite(VACUUM_PICK_PIN, HIGH);  // Turn off picking vacuum
      digitalWrite(VACUUM_PLACE_PIN, HIGH); // Turn off placing vacuum
      isPicking = false;
      isPlacing = false;
      Serial.println("Vacuum OFF");
      break;
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
  int consistentCount = 0;
  bool lastReading = HIGH;
  
  // Set direction for continuous movement
  stepper->setSpeedInHz(HOMING_SPEED);
  stepper->setAcceleration(HOMING_ACCEL);
  stepper->moveTo(-100000);
  
  while (consistentCount < CONSISTENT_READINGS) {
    bool currentReading = digitalRead(limitPin);
    
    // Immediately stop if limit switch is hit
    if (currentReading == LOW) {
      if (lastReading == LOW) {
        consistentCount++;
      }
    } else {
      consistentCount = 0;
    }
    
    if (consistentCount == 1) {  // First consistent LOW reading
      stepper->forceStop();      // Emergency stop when limit is first detected
    }
    
    lastReading = currentReading;
    
    if (millis() - startTime > HOMING_TIMEOUT) {
      Serial.printf("Error: %s axis homing timeout!\n", axisName);
      stepper->forceStop();
      return false;
    }
    
    delayMicroseconds(500);  // Minimal delay for switch stability
  }
  
  stepper->forceStopAndNewPosition(0);
  
  // Quick verification
  delayMicroseconds(1000);
  if (digitalRead(limitPin) == HIGH) {
    Serial.printf("Error: %s axis lost limit switch contact!\n", axisName);
    return false;
  }
  
  if (BACKOFF_STEPS > 0) {
    stepper->moveTo(BACKOFF_STEPS);
    while (stepper->isRunning()) delayMicroseconds(500);
  }
  
  return true;
}

// Add this new function
void resetVacuum() {
  digitalWrite(VACUUM_PICK_PIN, HIGH);   // Turn off picking vacuum
  digitalWrite(VACUUM_PLACE_PIN, HIGH);  // Turn off placing vacuum
  isPicking = false;
  isPlacing = false;
  Serial.println("Vacuum reset - all OFF");
}

void homeAxis() {
  Serial.println("Starting homing sequence...");
  pinMode(LED_PIN, OUTPUT);
  
  // Reset vacuum state at start of homing
  resetVacuum();
  
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

void moveToMax() {
  Serial.println("Moving to maximum position...");
  
  // Reset vacuum state at start of max movement
  resetVacuum();
  
  // Move X axis first
  moveToPosition(MAX_X, 0, 0, 0, MICROSTEPPING, LEAD_SCREW_PITCH, MAX_SPEED, ACCELERATION);
  // Then Y axis
  moveToPosition(MAX_X, MAX_Y, 0, 0, MICROSTEPPING, LEAD_SCREW_PITCH, MAX_SPEED, ACCELERATION);
  // Finally Z axis
  moveToPosition(MAX_X, MAX_Y, MAX_Z, 0, MICROSTEPPING, LEAD_SCREW_PITCH, MAX_SPEED, ACCELERATION);
  
  Serial.println("Reached maximum position!");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove any whitespace
    
    // Check if homing is needed
    if (!isHomed) {
      Serial.println("Performing initial homing...");
      homeAxis();
      Serial.println("Ready for commands");
      Serial.println("x0.0,y0.0,z0.0");
      return;
    }
    
    // Handle commands
    if (input.equals("HOME")) {
      homeAxis();
      return;
    }
    
    if (input.equals("MAX")) {
      moveToMax();
      return;
    }
    
    // Handle coordinate input
    float x, y, z;
    int vacuum;
    if (parseCoordinates(input, x, y, z, vacuum)) {
      moveToPosition(x, y, z, vacuum, MICROSTEPPING, LEAD_SCREW_PITCH, MAX_SPEED, ACCELERATION);
    }
  }
}
