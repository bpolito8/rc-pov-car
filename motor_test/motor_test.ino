#include <Servo.h>

Servo myservo;  // create servo object to control a servo

//int potpin = 0;  // analog pin used to connect the potentiometer
int servoMotorAngle;    // variable to read the value from the analog pin

#define MOTOR_PIN_3 10
#define MOTOR_PIN_4 8

#define ENABLE_PIN 11

#define OFF_SPEED 10

int sensorValue = 0;
int motorOutput = 0;
int currentMotorSpeed = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR_PIN_3, OUTPUT);
  pinMode(MOTOR_PIN_4, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  pinMode(A0, INPUT);

  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);

  Serial.begin(9600);
}

void loop() {

   // read the input on analog pin 0:
  sensorValue = analogRead(A0);

  // DC Motor
  // map the sensor value to a motor speed between 0 and 255
  motorOutput = map(sensorValue, 0, 1023, 50, 255);

  // if we want to stop the motor
  if(motorOutput < 50 + OFF_SPEED) {
    currentMotorSpeed = motorOutput;
    digitalWrite(MOTOR_PIN_3, LOW);
    digitalWrite(MOTOR_PIN_4, LOW);
    analogWrite(ENABLE_PIN, 0);

    Serial.println("motor stopped");
  }
  // if we want to change the motor speed
  else if(currentMotorSpeed + 5 < motorOutput + 155 || currentMotorSpeed - 5 > motorOutput + 155){
    currentMotorSpeed = motorOutput;
    digitalWrite(MOTOR_PIN_3, HIGH);
    digitalWrite(MOTOR_PIN_4, LOW);
    analogWrite(ENABLE_PIN, currentMotorSpeed);

    Serial.println(currentMotorSpeed);
  }

  // Servo Motor
  servoMotorAngle = map(sensorValue, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  Serial.println(servoMotorAngle);
  myservo.write(servoMotorAngle);                  // sets the servo position according to the scaled value

  delay(10);  // delay in between reads for stability
  
}
