//#include <Arduino.h>

// Define pin connections
const int LEDPIN = 2;

const int X_STEP_PIN = 25;
const int Y_STEP_PIN = 26;
const int Z_STEP_PIN = 27;
const int X_DIR_PIN = 5;
const int Y_DIR_PIN = 18;
const int Z_DIR_PIN = 19;
const int X_LIMIT_PIN = 21;
const int Y_LIMIT_PIN = 22;
const int Z_LIMIT_PIN = 23;

// Define max travel distances in steps (assuming 1mm = 80 steps)
//const long mmStep = 80;

const long X_MAX_TRAVEL = 240 * 80;
const long Y_MAX_TRAVEL = 150 * 80;
const long Z_MAX_TRAVEL = 120 * 80;

const long pulseDelay = 100;

// Current position in steps
volatile long currentX = 0;
volatile long currentY = 0;
volatile long currentZ = 0;

// Target position in steps
volatile long targetX = 0;
volatile long targetY = 0;
volatile long targetZ = 0;

bool homed = false;

// Timer for controlling stepper pulses
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  // X axis
  if (currentX != targetX) {
    digitalWrite(X_DIR_PIN, currentX < targetX ? HIGH : LOW);
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(X_STEP_PIN, LOW);
    currentX += (currentX < targetX) ? 1 : -1;
  }

  // Y axis
  if (currentY != targetY) {
    digitalWrite(Y_DIR_PIN, currentY < targetY ? HIGH : LOW);
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(Y_STEP_PIN, LOW);
    currentY += (currentY < targetY) ? 1 : -1;
  }

  // Z axis
  if (currentZ != targetZ) {
    digitalWrite(Z_DIR_PIN, currentZ < targetZ ? HIGH : LOW);
    digitalWrite(Z_STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(Z_STEP_PIN, LOW);
    currentZ += (currentZ < targetZ) ? 1 : -1;
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(500000);
  
  // Configure pins
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP);

  pinMode(LEDPIN, OUTPUT);

  // Set up timer interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100, true);  // 100 microseconds = 10kHz
  timerAlarmEnable(timer);
  
  // Home the machine
  digitalWrite(LEDPIN,HIGH);
  homeAxis();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    digitalWrite(LEDPIN,LOW);
    Serial.print("Recieved cmd: ");Serial.println(command);
    processCommand(command);
    digitalWrite(LEDPIN,HIGH);
  }
}

void homeAxis() {
  Serial.println("Moving to home position...");
  // Disable timer interrupt during homing
  timerAlarmDisable(timer);

  // Home Z axis
  while (digitalRead(Z_LIMIT_PIN) == HIGH) {
    digitalWrite(Z_DIR_PIN, LOW);
    digitalWrite(Z_STEP_PIN, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(Z_STEP_PIN, LOW);
    delayMicroseconds(pulseDelay);
  }
  currentZ = 0;
  targetZ = 0;
  
  // Home Y axis
  while (digitalRead(Y_LIMIT_PIN) == HIGH) {
    digitalWrite(Y_DIR_PIN, LOW);
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(pulseDelay);
  }
  currentY = 0;
  targetY = 0;
  
  // Home X axis
  while (digitalRead(X_LIMIT_PIN) == HIGH) {
    digitalWrite(X_DIR_PIN, LOW);
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(pulseDelay);
  }
  currentX = 0;
  targetX = 0;
  
  homed = true;
  Serial.println("Homing complete");

  // Re-enable timer interrupt
  timerAlarmEnable(timer);
}

void processCommand(String command) {
  if (!homed) {
    Serial.println("Error: Machine not homed");
    return;
  }
  
  int x = -1, y = -1, z = -1;
  char axis;
  int value;
  
  // Parse the command
  int startIndex = 0;
  while (startIndex < command.length()) {
    axis = command.charAt(startIndex);
    startIndex++;
    int endIndex = command.indexOf(',', startIndex);
    if (endIndex == -1) endIndex = command.length();
    value = command.substring(startIndex, endIndex).toInt();
    
    switch (axis) {
      case 'x':
        x = value;
        break;
      case 'y':
        y = value;
        break;
      case 'z':
        z = value;
        break;
    }
    
    startIndex = endIndex + 1;
  }
  
  // Convert mm to steps and move
  if (x != -1) moveAxis(x * 80, X_MAX_TRAVEL, targetX);
  if (y != -1) moveAxis(y * 80, Y_MAX_TRAVEL, targetY);
  if (z != -1) moveAxis(z * 80, Z_MAX_TRAVEL, targetZ);
  Serial.print("Parsed cmd: ");
  Serial.print("x:");Serial.print(x);
  Serial.print(" y:");Serial.print(y);
  Serial.print(" z:");Serial.println(z);

}

void moveAxis(long targetPosition, long maxTravel, volatile long &axisTarget) {
  if (targetPosition < 0 || targetPosition > maxTravel) {
    Serial.println("Error: Position out of bounds");
    return;
  }
  
  axisTarget = targetPosition;
}
