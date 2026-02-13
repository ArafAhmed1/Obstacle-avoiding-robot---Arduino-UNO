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