#include "FastAccelStepper.h"

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

// Motor Configuration Constants
const float STEP_ANGLE = 0.36;                    // Degrees per step
const int STEPS_PER_REV = (int)(360 / STEP_ANGLE); // 1000 steps per revolution

// Fixed configuration parameters
const float MICROSTEPPING = 16.0;
const float LEAD_SCREW_PITCH = 2.0;
const int32_t MAX_SPEED = 40000;
const int32_t ACCELERATION = 320000;

void setup() {
  Serial.begin(115200);
  
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
  // Ensure input ends with comma for consistent parsing
  if (!input.endsWith(",")) {
    input += ",";
  }
  
  // Find the position of x, y, z markers
  int xPos = input.indexOf('x');
  int yPos = input.indexOf('y');
  int zPos = input.indexOf('z');
  
  if (xPos == -1 || yPos == -1 || zPos == -1) {
    Serial.println("Error: Invalid coordinate format");
    return false;
  }
  
  // Extract and validate values
  x = input.substring(xPos + 1, yPos).toFloat();
  y = input.substring(yPos + 1, zPos).toFloat();
  z = input.substring(zPos + 1, input.indexOf(',')).toFloat();
  
  // Debug output
  Serial.printf("Parsed coordinates - X: %.3f, Y: %.3f, Z: %.3f\n", x, y, z);
  return true;
}

void moveToPosition(float x, float y, float z, float microstepping, float leadScrewPitch, int32_t maxSpeed, int32_t acceleration) {
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

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    float x, y, z;
    
    if (parseCoordinates(input, x, y, z)) {
      moveToPosition(x, y, z, MICROSTEPPING, LEAD_SCREW_PITCH, MAX_SPEED, ACCELERATION);
    }
  }
}
