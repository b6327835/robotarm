#define X_STEP_PIN 25  // Pin connected to the X-axis optocoupler
#define X_DIR_PIN 4   // Optional: Pin for X-axis direction control
#define Y_STEP_PIN 26  // Pin connected to the Y-axis optocoupler
#define Y_DIR_PIN 18   // Optional: Pin for Y-axis direction control
#define Z_STEP_PIN 27  // Pin connected to the Z-axis optocoupler
#define Z_DIR_PIN 19   // Optional: Pin for Z-axis direction control

long targetX = 25000;  // Example: Number of steps for X-axis
long targetY = 25000;   // Example: Number of steps for Y-axis
long targetZ = 25000;   // Example: Number of steps for Z-axis
int pulseDelay = 20; // Delay in microseconds between pulses (adjust for speed)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Setup pins as output
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);

  // Set the directions (Optional, depending on your setup)
  digitalWrite(X_DIR_PIN, HIGH);  // Set direction for X-axis
  digitalWrite(Y_DIR_PIN, HIGH);  // Set direction for Y-axis
  digitalWrite(Z_DIR_PIN, HIGH);  // Set direction for Z-axis

  Serial.println("Send 'X' to move all axes.");
}

void loop() {
  // Check for serial input to start movement
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'X' || input == 'x') {
      Serial.println("Moving X, Y, Z axes...");

      // Move all axes simultaneously
      moveAxesSimultaneously(targetX, targetY, targetZ);

      Serial.println("Movement completed!");
    }
  }

  delay(100);  // Small delay to avoid flooding the serial buffer
}

// Function to move 3 axes simultaneously
void moveAxesSimultaneously(long stepsX, long stepsY, long stepsZ) {
  long currentX = 0, currentY = 0, currentZ = 0;

  // Calculate the total steps for the longest axis (Bresenham's algorithm base)
  long maxSteps = max(stepsX, max(stepsY, stepsZ));

  // Bresenham-style increment ratios
  long dx = stepsX, dy = stepsY, dz = stepsZ;
  long errorX = 0, errorY = 0, errorZ = 0;

  for (long i = 0; i < maxSteps; i++) {
    // Step X axis if needed
    if (2 * errorX >= maxSteps) {
      digitalWrite(X_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(X_STEP_PIN, LOW);
      currentX++;
      errorX += dx - maxSteps;
    } else {
      errorX += dx;
    }

    // Step Y axis if needed
    if (2 * errorY >= maxSteps) {
      digitalWrite(Y_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(Y_STEP_PIN, LOW);
      currentY++;
      errorY += dy - maxSteps;
    } else {
      errorY += dy;
    }

    // Step Z axis if needed
    if (2 * errorZ >= maxSteps) {
      digitalWrite(Z_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(Z_STEP_PIN, LOW);
      currentZ++;
      errorZ += dz - maxSteps;
    } else {
      errorZ += dz;
    }

    delayMicroseconds(pulseDelay);  // Adjust speed by controlling this delay
  }

  // Print final positions (for debugging)
  Serial.print("Final X: "); Serial.println(currentX);
  Serial.print("Final Y: "); Serial.println(currentY);
  Serial.print("Final Z: "); Serial.println(currentZ);
}
