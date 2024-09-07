#include <FastAccelStepper.h>

// Define stepper motor pins
#define STEP_PIN_X 12
#define DIR_PIN_X 14
#define STEP_PIN_Y 27
#define DIR_PIN_Y 26
#define STEP_PIN_Z 33
#define DIR_PIN_Z 32

FastAccelStepperEngine engine = FastAccelStepperEngine();  // Stepper engine instance
FastAccelStepper *stepperX = NULL;
FastAccelStepper *stepperY = NULL;
FastAccelStepper *stepperZ = NULL;

void setup() {
  Serial.begin(115200);
  
  // Initialize the stepper engine
  engine.init();

  // Attach stepper motors
  stepperX = engine.stepperConnectToPin(STEP_PIN_X);
  stepperY = engine.stepperConnectToPin(STEP_PIN_Y);
  stepperZ = engine.stepperConnectToPin(STEP_PIN_Z);

  if (stepperX && stepperY && stepperZ) {
    // Set the direction pins for each motor
    stepperX->setDirectionPin(DIR_PIN_X);
    stepperY->setDirectionPin(DIR_PIN_Y);
    stepperZ->setDirectionPin(DIR_PIN_Z);

    // Enable motors (optional, depending on your setup)
    stepperX->setEnablePin(-1);
    stepperY->setEnablePin(-1);
    stepperZ->setEnablePin(-1);

    // Set speed and acceleration for each motor
    stepperX->setAutoEnable(true);
    stepperY->setAutoEnable(true);
    stepperZ->setAutoEnable(true);

    // Set max speed (steps per second) and acceleration (steps per second^2)
    stepperX->setSpeedInHz(1000);   // Max speed in steps per second
    stepperX->setAcceleration(500); // Acceleration in steps per second^2
    
    stepperY->setSpeedInHz(1000);
    stepperY->setAcceleration(500);
    
    stepperZ->setSpeedInHz(1000);
    stepperZ->setAcceleration(500);
  }
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    // Parse and process the command
    parseCommand(command);
  }
}

void parseCommand(String command) {
  int xIndex = command.indexOf("X");
  int yIndex = command.indexOf("Y");
  int zIndex = command.indexOf("Z");
  
  Serial.print("x: ");
  Serial.println(xIndex);
  Serial.print("y: ");
  Serial.println(yIndex);
  Serial.print("z: ");
  Serial.println(zIndex);

  if (xIndex != -1) {
    int xEnd = command.indexOf(' ', xIndex);
    if (xEnd == -1) xEnd = command.length();
    String xValue = command.substring(xIndex + 1, xEnd);
    stepperX->moveTo(xValue.toInt()); // Move stepper X to target position
  }
  
  if (yIndex != -1) {
    int yEnd = command.indexOf(' ', yIndex);
    if (yEnd == -1) yEnd = command.length();
    String yValue = command.substring(yIndex + 1, yEnd);
    stepperY->moveTo(yValue.toInt()); // Move stepper Y to target position
  }
  
  if (zIndex != -1) {
    String zValue = command.substring(zIndex + 1);
    stepperZ->moveTo(zValue.toInt()); // Move stepper Z to target position
  }

  // Wait for all movements to complete
  while (stepperX->isRunning() || stepperY->isRunning() || stepperZ->isRunning()) {
    // Optionally print status, delay, or other actions
  }
  
  // Print a message when all movements are complete
  Serial.println("Movement complete");
}
