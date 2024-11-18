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
// XY motor 0,0,0,16,2,40000,320000,40000
// Y motor 0,0,0,8,2,44000,42000,44000

//X and Y 0,0,0,8,2,42000,35000,40000
//XYZ 20,10,10,8,2,42000,35000,40000
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
    
    Serial.println("ARM66SAK Cartesian Controller Ready");
    Serial.println("Enter parameters as 'X,Y,Z,MS,LP,MAX,ACC,START'");
    Serial.println("Example: '100,50,-25,1,2,4000,2000,1000'");
  } else {
    Serial.println("Error initializing steppers!");
  }
}

// Convert millimeters to steps with improved precision
int32_t mmToSteps(float mm, float stepsPerMm) {
  return (int32_t)(mm * stepsPerMm + 0.5f); // Added 0.5 for proper rounding
}

// Function to move to absolute position with dynamic parameters
void moveToPosition(float x, float y, float z, float microstepping, float leadScrewPitch, int32_t maxSpeed, int32_t acceleration, int32_t startSpeed) {
  float stepsPerMm = (STEPS_PER_REV * microstepping) / leadScrewPitch;

  // Configure speed and acceleration
  stepperX->setSpeedInHz(startSpeed);
  stepperY->setSpeedInHz(startSpeed);
  stepperZ->setSpeedInHz(startSpeed);
  
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
  
  // Log the movement
  Serial.printf("Moving to X:%.2f Y:%.2f Z:%.2f (steps: %d,%d,%d)\n", 
                x, y, z, xSteps, ySteps, zSteps);
  
  // Move all motors to target positions
  stepperX->moveTo(xSteps);
  stepperY->moveTo(ySteps);
  stepperZ->moveTo(zSteps);
  
  // Wait until all steppers are in position
  while (stepperX->isRunning() || stepperY->isRunning() || stepperZ->isRunning()) {
    delay(1);  // Small delay to prevent watchdog reset
  }
  
  Serial.println("Movement complete");
}

// Function to parse coordinates and additional parameters
bool parseParameters(String input, float &x, float &y, float &z, float &microstepping, float &leadScrewPitch, int32_t &maxSpeed, int32_t &acceleration, int32_t &startSpeed) {
  int firstComma = input.indexOf(',');
  int secondComma = input.indexOf(',', firstComma + 1);
  int thirdComma = input.indexOf(',', secondComma + 1);
  int fourthComma = input.indexOf(',', thirdComma + 1);
  int fifthComma = input.indexOf(',', fourthComma + 1);
  int sixthComma = input.indexOf(',', fifthComma + 1);
  int seventhComma = input.indexOf(',', sixthComma + 1);
  
  if (firstComma == -1 || secondComma == -1 || thirdComma == -1 || fourthComma == -1 || 
      fifthComma == -1 || sixthComma == -1 || seventhComma == -1) {
    Serial.println("Invalid format. Use X,Y,Z,MS,LP,MAX,ACC,START");
    return false;
  }
  
  x = input.substring(0, firstComma).toFloat();
  y = input.substring(firstComma + 1, secondComma).toFloat();
  z = input.substring(secondComma + 1, thirdComma).toFloat();
  microstepping = input.substring(thirdComma + 1, fourthComma).toFloat();
  leadScrewPitch = input.substring(fourthComma + 1, fifthComma).toFloat();
  maxSpeed = input.substring(fifthComma + 1, sixthComma).toInt();
  acceleration = input.substring(sixthComma + 1, seventhComma).toInt();
  startSpeed = input.substring(seventhComma + 1).toInt();
  
  return true;
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    float x, y, z, microstepping, leadScrewPitch;
    int32_t maxSpeed, acceleration, startSpeed;
    
    if (parseParameters(input, x, y, z, microstepping, leadScrewPitch, maxSpeed, acceleration, startSpeed)) {
      moveToPosition(x, y, z, microstepping, leadScrewPitch, maxSpeed, acceleration, startSpeed);
    }
  }
}
