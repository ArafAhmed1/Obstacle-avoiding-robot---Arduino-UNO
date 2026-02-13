#include <Servo.h>
#include <AFMotor.h>

#define Echo A0
#define Trig A1
#define motor 10
#define SpeedL 250
#define SpeedR 200
#define spoint 83 // Servo center position

char value;
int distance = 0;
int Left;
int Right;
int L = 0;
int R = 0;
int L1 = 0;
int R1 = 0;

Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);


void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(motor);
  // Initialize motors
  //M1.setSpeed(Speed);
  M2.setSpeed(SpeedL);
  M3.setSpeed(SpeedR);
  M4.setSpeed(SpeedR);

  // Center the servo
  servo.write(spoint);
  delay(500);
}

void loop() {
  // Obstacle();
  Bluetoothcontrol();
  // voicecontrol();
}

void Obstacle() {
  distance = ultrasonic();

  if (distance == -1) {
    Serial.println("Error: No valid distance reading");
    // Stop();
    // return;
    forward();
  }

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance <= 25) { // If an obstacle is within 25 cm
    Serial.println("Obstacle detected! Initiating avoidance...");
    avoidObstacle();
  } else {
    Serial.println("Path clear. Moving forward...");
    forward(); // Move forward if no obstacle is detected
  }
  delay(100);
}

int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  long t = pulseIn(Echo, HIGH, 60000); // Timeout after 60 ms
  if (t == 0) {
    return -1; // No valid reading
  }
  return t / 29 / 2; // Convert time to distance in cm
}

void avoidObstacle() {
  Stop();
  backward();
  delay(300);
  Stop();

  // Scan left and right
  L = leftsee();
  delay(300);
  R = rightsee();
  delay(300);
  servo.write(spoint); // Center the servo
  delay(300);

  Serial.print("Left Distance: ");
  Serial.println(L);
  Serial.print("Right Distance: ");
  Serial.println(R);

  if (L > 25 && L > R) {
    Serial.println("Turning left...");
    left();
    delay(700); // Wider turn
  } else if (R > 25 && R > L) {
    Serial.println("Turning right...");
    right();
    delay(700); // Wider turn
  } else {
    Serial.println("Both sides blocked. Moving backward...");
    backward();
    delay(500);
  }

  Stop();
  delay(200);
}

void Bluetoothcontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
  }
  if (value == 'F') {
    forward();
  } else if (value == 'B') {
    backward();
  } else if (value == 'L') {
    right();
  } else if (value == 'R') {
    left();
  } else if (value == 'S') {
    Stop();
  }
}

void voicecontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);

    
    if (value == '^') {
      forward();
    } else if (value == '-') {
      backward();
    } else if (value == '>') {
      L = rightsee();
      servo.write(spoint);
      if (L >= 10 ) {
        left();
        delay(500);
        Stop();
      } else if (L < 10) {
        Stop();
      }
    } else if (value == '<') {
      R = leftsee();
      servo.write(spoint);
      if (R >= 10 ) {
        right();
        delay(500);
        Stop();
      } else if (R < 10) {
        Stop();
      }
    } else if (value == '*') {
      Stop();
    }
  }
}

int leftsee() {
  servo.write(170); // Look left
  delay(500); // Allow servo to stabilize
  int distance = ultrasonic();
  Serial.print("Left see distance: ");
  Serial.println(distance);
  return distance;
}

int rightsee() {
  servo.write(10); // Look right
  delay(500); // Allow servo to stabilize
  int distance = ultrasonic();
  Serial.print("Right see distance: ");
  Serial.println(distance);
  return distance;
}

// Motor control functions
void forward() {
  Serial.println("Motors moving forward.");
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

void backward() {
  Serial.println("Motors moving backward.");
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void left() {
  Serial.println("Motors turning left.");
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void right() {
  Serial.println("Motors turning right.");
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

void Stop() {
  Serial.println("Motors stopped.");
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}