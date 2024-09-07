//#include <AccelStepper.h>
//
//// Define stepper motor pins
//#define STEP_PIN_X 12
//#define DIR_PIN_X 14
//#define STEP_PIN_Y 27
//#define DIR_PIN_Y 26
//#define STEP_PIN_Z 33
//#define DIR_PIN_Z 32
//
//// Create stepper motor instances
//AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
//AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);
//AccelStepper stepperZ(AccelStepper::DRIVER, STEP_PIN_Z, DIR_PIN_Z);
//
//void setup() {
//  Serial.begin(115200);
//  stepperX.setMaxSpeed(1000);
//  stepperY.setMaxSpeed(1000);
//  stepperZ.setMaxSpeed(1000);
//}
//
//void loop() {
//  if (Serial.available()) {
//    String command = Serial.readStringUntil('\n');
//    // Parse and process the command
//    parseCommand(command);
//  }
//  
//  // Run all stepper motors
//  stepperX.run();
//  stepperY.run();
//  stepperZ.run();
//}
//
//void parseCommand(String command) {
//  // Parse the command and set motor targets
//  int xIndex = command.indexOf("X");
//  int yIndex = command.indexOf("Y");
//  int zIndex = command.indexOf("Z");
//
//  if (xIndex != -1) {
//    int xEnd = command.indexOf(' ', xIndex);
//    String xValue = command.substring(xIndex + 1, xEnd);
//    stepperX.moveTo(xValue.toInt());
//  }
//  
//  if (yIndex != -1) {
//    int yEnd = command.indexOf(' ', yIndex);
//    String yValue = command.substring(yIndex + 1, yEnd);
//    stepperY.moveTo(yValue.toInt());
//  }
//  
//  if (zIndex != -1) {
//    String zValue = command.substring(zIndex + 1);
//    stepperZ.moveTo(zValue.toInt());
//  }
//}
