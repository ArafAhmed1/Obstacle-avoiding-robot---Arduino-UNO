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



void setup()
{
    Serial.begin(9600);
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    servo.attach(motor);
    // Initialize motors
    // M1.setSpeed(Speed);
    M2.setSpeed(SpeedL);
    M3.setSpeed(SpeedR);
    M4.setSpeed(SpeedR);

    // Center the servo
    servo.write(spoint);
    delay(500);
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

