#define X_STEP_PIN 25  // Pin connected to the X-axis optocoupler
#define X_DIR_PIN 4   // Pin for X-axis direction control
#define Y_STEP_PIN 26  // Pin connected to the Y-axis optocoupler
#define Y_DIR_PIN 18   // Pin for Y-axis direction control
#define Z_STEP_PIN 27  // Pin connected to the Z-axis optocoupler
#define Z_DIR_PIN 19   // Pin for Z-axis direction control
#define LEDPIN 2

int pulseDelay = 100;  // Delay in microseconds between pulses (adjust for speed)

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
  pinMode(LEDPIN,OUTPUT);

  digitalWrite(LEDPIN,HIGH);

  Serial.println("Send commands like '1X1000 2Y2000 1Z3000' to move the motors simultaneously.");
}

void loop() {
  if (Serial.available() > 0) {
    digitalWrite(LEDPIN,HIGH);
    String input = Serial.readStringUntil('\n');  // Read the entire line of input
    Serial.println(input);
    parseAndMove(input);  // Parse the input and move the axes simultaneously
  }

  delay(100);  // Small delay to avoid overwhelming the serial buffer
}

// Function to parse the input and move the motors
void parseAndMove(String input) {
  // Example input: "1X1000 2Y2000 1Z3000"
  input.trim();  // Remove any extra spaces or newline characters

  // Variables to store the number of steps and directions for each axis
  long stepsX = 0, stepsY = 0, stepsZ = 0;
  int dirX = 1, dirY = 1, dirZ = 1;

  // Split the input based on spaces between commands
  int startIndex = 0;
  while (startIndex < input.length()) {
    int spaceIndex = input.indexOf(' ', startIndex);
    if (spaceIndex == -1) {
      spaceIndex = input.length();  // Handle the last command
    }

    String command = input.substring(startIndex, spaceIndex);
    if (command.length() >= 3) {
      int direction = command[0] - '0';  // Convert '1' or '2' to int (1 = forward, 2 = backward)
      char axis = command[1];            // 'X', 'Y', or 'Z'
      long steps = command.substring(2).toInt();  // Number of steps

      // Set direction and steps for each axis
      switch (axis) {
        case 'X':
          dirX = (direction == 1) ? HIGH : LOW;
          stepsX = steps;
          break;
        case 'Y':
          dirY = (direction == 1) ? HIGH : LOW;
          stepsY = steps;
          break;
        case 'Z':
          dirZ = (direction == 1) ? HIGH : LOW;
          stepsZ = steps;
          break;
        default:
          Serial.println("Invalid axis");
          break;
      }
    }
    startIndex = spaceIndex + 1;  // Move to the next command
  }

  // Move all axes simultaneously
  moveAxesSimultaneously(stepsX, stepsY, stepsZ, dirX, dirY, dirZ);
  digitalWrite(LEDPIN,LOW);
}

// Function to move 3 axes simultaneously
void moveAxesSimultaneously(long stepsX, long stepsY, long stepsZ, int dirX, int dirY, int dirZ) {
  // Set directions for each axis
  digitalWrite(X_DIR_PIN, dirX);
  digitalWrite(Y_DIR_PIN, dirY);
  digitalWrite(Z_DIR_PIN, dirZ);

  long currentX = 0, currentY = 0, currentZ = 0;

  // Calculate the total steps for the longest axis (Bresenham's algorithm base)
  long maxSteps = max(stepsX, max(stepsY, stepsZ));

  // Bresenham-style increment ratios
  long dx = stepsX, dy = stepsY, dz = stepsZ;
  long errorX = 0, errorY = 0, errorZ = 0;

  for (long i = 0; i < maxSteps; i++) {
    // Step X axis if needed
    if (currentX < stepsX && 2 * errorX >= maxSteps) {
      digitalWrite(X_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(X_STEP_PIN, LOW);
      currentX++;
      errorX += dx - maxSteps;
    } else {
      errorX += dx;
    }

    // Step Y axis if needed
    if (currentY < stepsY && 2 * errorY >= maxSteps) {
      digitalWrite(Y_STEP_PIN, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(Y_STEP_PIN, LOW);
      currentY++;
      errorY += dy - maxSteps;
    } else {
      errorY += dy;
    }

    // Step Z axis if needed
    if (currentZ < stepsZ && 2 * errorZ >= maxSteps) {
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
