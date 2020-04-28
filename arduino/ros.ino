#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include<AccelStepper.h>

//___________________________dvig Open_____________
#define Sila 9
#define Naprav 8

//______________________________Kon________
#define front_end_switch 7
#define back_end_switch  6

//___________________________servo________________
Servo servo44; //big
Servo servo45; //small

//______________________________stepper_______________
#define HALFSTEP 8  
#define motorPin1  10 // IN1 на 1-м драйвере ULN2003
#define motorPin2  11 // IN2 на 1-м драйвере ULN2003
#define motorPin3  12 // IN3 на 1-м драйвере ULN2003
#define motorPin4  13 // IN4 на 1-м драйвере ULN2003

AccelStepper stepper(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);


class NewHardware : public ArduinoHardware
{
  public:
    NewHardware():
      ArduinoHardware(&Serial1, 115200) {};
};

std_msgs::Int32 cmd_msg;

ros::NodeHandle  servos;          ///arduino/servo1    /arduino/servo2     Int32
void s1(const std_msgs::Int32& cmd_msg) { // servo big
  servo44.write(cmd_msg.data);
}
void s2(const std_msgs::Int32 & cmd_msg) { // sevo small
  servo45.write(cmd_msg.data);
}
void motorRotation1(const std_msgs::Int32 & cmd_msg) { // moto-open
  if (cmd_msg.data < 0) { // open 
    analogWrite(Sila, 130);
    digitalWrite(Naprav, LOW);
  }
  else if (cmd_msg.data > 0) { // closse
    analogWrite(Sila, 130);
    digitalWrite(Naprav, HIGH);
  }
  else { // stop
    analogWrite(Sila, 255);
    digitalWrite(Naprav, HIGH);
  }
}

void motorline(const std_msgs::Int32 & cmd_msg2) { // stepper a need testing 
  if (cmd_msg2.data == 0) {
    stepper.setSpeed(-50);
    while (digitalRead(back_end_switch) == 1) {
      stepper.runSpeed();
    }
//    stepper.stop();
  }
  else {
    stepper.setSpeed(50);
    while (digitalRead(back_end_switch) == 1) {
      stepper.runSpeed();
    }
//    stepper.stop();
  }
}


ros::Subscriber<std_msgs::Int32> subs1("/arduino/servo1", &s1); //servo big
ros::Subscriber<std_msgs::Int32> subs2("/arduino/servo2", &s2); // sevo small
ros::Subscriber<std_msgs::Int32> subs3("/arduino/motor1", &motorRotation1); // moto open
ros::Subscriber<std_msgs::Int32> subs4("/arduino/slider", &motorline); // steper

void setup()
{
  pinMode(Sila, OUTPUT);
  pinMode(Naprav, OUTPUT);
  digitalWrite(Naprav, HIGH);
  analogWrite(Sila, 255);
  servo44.attach(44);
  servo45.attach(45);
  servos.initNode();
  pinMode(front_end_switch, INPUT_PULLUP);
  pinMode(back_end_switch, INPUT_PULLUP);
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(50);
  servos.subscribe(subs1);
  servos.subscribe(subs2);
  servos.subscribe(subs3);
  servos.subscribe(subs4);
}

void loop()
{
  servos.spinOnce();
  delay(1);
}

