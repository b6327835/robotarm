#define X_STEP_PIN 25  // Pin connected to the X-axis optocoupler
#define X_DIR_PIN 4    // Pin for X-axis direction control
#define Y_STEP_PIN 26  // Pin connected to the Y-axis optocoupler
#define Y_DIR_PIN 18   // Pin for Y-axis direction control
#define Z_STEP_PIN 27  // Pin connected to the Z-axis optocoupler
#define Z_DIR_PIN 19   // Pin for Z-axis direction control
#define LEDPIN 2

int pulseDelay = 100;   // Delay in microseconds between pulses (adjust for speed)
float stepsPerMM = 10.0;  // Conversion factor: steps per mm (adjust based on your setup)
int directionDelay = 5;   // Delay in milliseconds to allow direction change to take effect

// Variables to store last known positions in mm
float lastX = 0.0, lastY = 0.0, lastZ = 0.0;

void setup() {
  Serial.begin(115200);

  // Setup pins as output
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
    String input = Serial.readStringUntil('\n');  // Read the entire line of input
    input.trim();  // Remove extra spaces or newlines
    Serial.println(input);  // Echo the received input
    parseAndMove(input);  // Parse the input and move the motors
  }
  delay(100);  // Small delay to avoid overwhelming the serial buffer
}

// Function to parse the ROS2 input and move the motors
void parseAndMove(String input) {
  // Expected format: "x-5,y-35,z-29" (mm values)
  float targetX = 0.0, targetY = 0.0, targetZ = 0.0;

  // Parse X, Y, Z values
  int xIndex = input.indexOf('x');
  int yIndex = input.indexOf('y');
  int zIndex = input.indexOf('z');
  
  if (xIndex != -1) {
    targetX = input.substring(xIndex + 1, yIndex).toFloat();
  }

  if (yIndex != -1) {
    targetY = input.substring(yIndex + 1, zIndex).toFloat();
  }

  if (zIndex != -1) {
    targetZ = input.substring(zIndex + 1).toFloat();
  }

  // Calculate steps to move for each axis
  long stepsX = convertMMToSteps(abs(targetX - lastX));
  long stepsY = convertMMToSteps(abs(targetY - lastY));
  long stepsZ = convertMMToSteps(abs(targetZ - lastZ));

  // Determine direction for each axis
  int dirX = (targetX > lastX) ? HIGH : LOW;
  int dirY = (targetY > lastY) ? HIGH : LOW;
  int dirZ = (targetZ > lastZ) ? HIGH : LOW;

  // Move axes
  moveAxesSimultaneously(stepsX, stepsY, stepsZ, dirX, dirY, dirZ);

  // Update last known positions
  lastX = targetX;
  lastY = targetY;
  lastZ = targetZ;

  digitalWrite(LEDPIN, LOW);
}

// Function to convert mm to steps
long convertMMToSteps(float mm) {
  return mm * stepsPerMM;  // Convert mm to steps
}

// Function to move 3 axes simultaneously
void moveAxesSimultaneously(long stepsX, long stepsY, long stepsZ, int dirX, int dirY, int dirZ) {
  // Set directions for each axis
  digitalWrite(X_DIR_PIN, dirX);
  delay(directionDelay);  // Wait for direction to settle
  digitalWrite(Y_DIR_PIN, dirY);
  delay(directionDelay);  // Wait for direction to settle
  digitalWrite(Z_DIR_PIN, dirZ);
  delay(directionDelay);  // Wait for direction to settle

  long currentX = 0, currentY = 0, currentZ = 0;
  long maxSteps = max(stepsX, max(stepsY, stepsZ));

  for (long i = 0; i < maxSteps; i++) {
    if (currentX < stepsX) {
      digitalWrite(X_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(X_STEP_PIN, LOW);
      currentX++;
    }

    if (currentY < stepsY) {
      digitalWrite(Y_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(Y_STEP_PIN, LOW);
      currentY++;
    }

    if (currentZ < stepsZ) {
      digitalWrite(Z_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(Z_STEP_PIN, LOW);
      currentZ++;
    }

    delayMicroseconds(pulseDelay);  // Adjust speed by controlling this delay
  }

  // Print final positions for debugging
  Serial.print("Final X (steps): "); Serial.println(currentX);
  Serial.print("Final Y (steps): "); Serial.println(currentY);
  Serial.print("Final Z (steps): "); Serial.println(currentZ);
}
