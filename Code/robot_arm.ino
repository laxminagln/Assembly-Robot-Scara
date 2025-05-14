#include <Servo.h>
#include <math.h>

// ====== Pins ======
#define BASE_STEP_PIN 2
#define BASE_DIR_PIN 3

#define Z_STEP_PIN 4
#define Z_DIR_PIN 5

#define SERVO1_PIN 6
#define SERVO2_PIN 7

#define DC_IN1 8
#define DC_IN2 9

// ====== Robot Dimensions ======
const float L1 = 10.0;  // Link 1 length (cm)
const float L2 = 8.0;   // Link 2 length (cm)

// ====== Motor Setup ======
Servo servo1;
Servo servo2;

// ====== Setup ======
void setup() {
  Serial.begin(9600);

  pinMode(BASE_STEP_PIN, OUTPUT);
  pinMode(BASE_DIR_PIN, OUTPUT);

  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);

  pinMode(DC_IN1, OUTPUT);
  pinMode(DC_IN2, OUTPUT);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // Move to home
  moveBase(100, true);       // Optional rotation
  moveZ(100, true);          // Raise arm
  moveToXY(10, 5);           // Move to XY
}

// ====== Loop (for serial command) ======
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    float x = input.substring(0, input.indexOf(',')).toFloat();
    float y = input.substring(input.indexOf(',') + 1).toFloat();

    moveBase(500, true);

    moveZ(3500, false);  // Lower arm
    delay(500);
    
    moveToXY(-6, 7);
    delay(500);

    moveZ(1000, true);  // Lower arm
    delay(500);

    moveBase(500, false);
  
    moveZ(1000, false);   // Raise arm
    delay(500);

    moveToXY(0, 18);
    delay(500);

    moveZ(100, false);  // Lower arm
    delay(500);

    screwIn();          // Screw
    delay(2000);

    stopScrewdriver();
    delay(500);

    moveZ(3500, true);  // Lower arm
    
  }
}

// ====== Inverse Kinematics ======
void moveToXY(float x, float y) {
  float r = sqrt(x * x + y * y);

  if (r > (L1 + L2) || r < abs(L1 - L2)) {
    Serial.println("Target unreachable!");
    return;
  }

  float cos_theta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
  float theta2 = acos(cos_theta2);

  float k1 = L1 + L2 * cos(theta2);
  float k2 = L2 * sin(theta2);
  float theta1 = atan2(y, x) - atan2(k2, k1);

  int deg1 = (int)(theta1 * 180.0 / PI);
  int deg2 = (int)(theta2 * 180.0 / PI);

  deg1 = constrain(deg1, 0, 180);
  deg2 = constrain(deg2, 0, 180);

  servo1.write(deg1);
  delay(500);
  servo2.write(deg2);
  delay(500);

  Serial.print("Moved to X: ");
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.print(y);
  Serial.print(" => Servo1: ");
  Serial.print(deg1);
  Serial.print("°, Servo2: ");
  Serial.println(deg2);
}

// ====== Stepper: Base Rotation ======
void moveBase(int steps, bool clockwise) {
  digitalWrite(BASE_DIR_PIN, clockwise ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(BASE_STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(BASE_STEP_PIN, LOW);
    delayMicroseconds(800);
  }
}

// ====== Stepper: Z-axis ======
    void moveZ(int steps, bool up) {
      digitalWrite(Z_DIR_PIN, up ? HIGH : LOW);
      for (int i = 0; i < steps; i++) {
        digitalWrite(Z_STEP_PIN, HIGH);
        delayMicroseconds(800);
        digitalWrite(Z_STEP_PIN, LOW);
        delayMicroseconds(800);
      }
    }

// ====== Screwdriver ======
    void screwIn() {
      digitalWrite(DC_IN1, HIGH);
      digitalWrite(DC_IN2, LOW);
    }

    void stopScrewdriver() {
      digitalWrite(DC_IN1, LOW);
      digitalWrite(DC_IN2, LOW);
    }

