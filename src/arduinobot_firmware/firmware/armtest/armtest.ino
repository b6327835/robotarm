#define Y_STEP_PIN 26  // Pin connected to the Y-axis optocoupler
#define Y_DIR_PIN 18   // Pin for Y-axis direction control
#define LEDPIN 2
int pulseDelay = 100;  // Delay in microseconds between pulses (adjust for speed)
int stepCount = 1000;   // Number of steps to move


void setup() {
  Serial.begin(115200);

  // Setup pins as output
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);

  Serial.println("Testing Y-axis forward and backward movement.");
}

void loop() {
    if (Serial.available() > 0) {
    // Read the incoming data (you can add specific checks if needed)
    String input = Serial.readString();
    Serial.println("Received input: " + input);
  // Move forward
  Serial.println("Moving Y-axis forward...");
  digitalWrite(Y_DIR_PIN, HIGH);  // Set direction to forward
  moveY(stepCount);               // Move Y-axis forward by stepCount steps
  delay(1000);                    // Wait for 1 second

  // Move backward
  Serial.println("Moving Y-axis backward...");
  digitalWrite(Y_DIR_PIN, LOW);   // Set direction to backward
  moveY(stepCount);               // Move Y-axis backward by stepCount steps
  delay(1000);                    // Wait for 1 second
}
}
// Function to move the Y-axis by the specified number of steps
void moveY(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(Y_STEP_PIN, HIGH);  // Step high
    delayMicroseconds(pulseDelay);   // Wait for the pulse delay
    digitalWrite(Y_STEP_PIN, LOW);   // Step low
    delayMicroseconds(pulseDelay);   // Wait for the pulse delay
  }
}
