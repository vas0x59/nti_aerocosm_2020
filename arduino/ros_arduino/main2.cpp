#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

int max_servo1 = 100;
int min_servo1 = 10;

int max_servo2 = 100;
int min_servo2 = 10;

//___________________________dvig Open_____________
#define Sila 9
#define Naprav 8

//______________________________Kon________
#define front_end_switch 7
#define back_end_switch 6

//___________________________servo________________
Servo servo44; //big
Servo servo45; //small

//______________________________stepper_______________
#include <Stepper_28BYJ.h>
// изменить количество шагов для вашего мотора
#define STEPS 2048
Stepper_28BYJ stepper(STEPS, 10, 11, 12, 13);


//_____________________________HC-12__________________
#define HC12SetPin 40

class NewHardware : public ArduinoHardware
{
  public:
    NewHardware(): ArduinoHardware(&Serial1, 115200) {};
};
ros::NodeHandle_<NewHardware>  nh;

void s1(const std_msgs::Int32 &cmd_msg)
{ // servo big
  if ((cmd_msg.data <  max_servo1) and  (cmd_msg.data >  min_servo1)) {
    servo44.write(cmd_msg.data);
  }
}
void s2(const std_msgs::Int32 &cmd_msg)
{ // sevo small
  if ((cmd_msg.data <  max_servo2) and (cmd_msg.data >  min_servo2)) {
    servo45.write(cmd_msg.data);
  }
}
void motorRotation1(const std_msgs::Int32 &cmd_msg)
{ // moto-open
  if (cmd_msg.data < 0)
  { // open
    int qwe;
    qwe = 255 + cmd_msg.data;
    analogWrite(Sila, abs(qwe));
    digitalWrite(Naprav, LOW);
  }
  else if (cmd_msg.data > 0)
  { // closse

    analogWrite(Sila, abs(cmd_msg.data));
    digitalWrite(Naprav, HIGH);
  }
  else
  { // stop
    analogWrite(Sila, 255);
    digitalWrite(Naprav, HIGH);
  }
}

int slider_speed = 0;
int slider_maxspeed = 50;

void motorline(const std_msgs::Int32 &cmd_msg)
{
  if ((digitalRead(front_end_switch) == 1) and (cmd_msg.data < 0)) 
  {
  stepper.setSpeed(13);
  stepper.step(cmd_msg.data);
  }
  if ((digitalRead(back_end_switch) == 1) and (cmd_msg.data > 0))
  {
  stepper.setSpeed(13);
  stepper.step(cmd_msg.data);
  }
    if ((digitalRead(front_end_switch) == 0) and (cmd_msg.data > 0)) 
  {
  stepper.setSpeed(13);
  stepper.step(0);
  }
  if ((digitalRead(back_end_switch) == 0) and (cmd_msg.data < 0))
  {
  stepper.setSpeed(13);
  stepper.step(0);
  }
  
  
}

ros::Subscriber<std_msgs::Int32> subs1("/arduino/servo1", &s1);             //servo big
ros::Subscriber<std_msgs::Int32> subs2("/arduino/servo2", &s2);             // sevo small
ros::Subscriber<std_msgs::Int32> subs3("/arduino/motor1", &motorRotation1); // moto open
ros::Subscriber<std_msgs::Int32> subs4("/arduino/slider", &motorline);    // steper %

void setup()
{
  Serial3.begin(9600);
  digitalWrite(HC12SetPin, LOW);
  delay(500);
  //HC-12

  Serial3.println("AT");
  delay(50);
  Serial3.println("AT+V");
  delay(50);
  Serial3.println("AT+DEFAULT");
  delay(50);
  Serial3.println("AT+P8");
  delay(50);
  Serial3.println("AT+C005");
  // delay(10);
  delay(500);
  digitalWrite(HC12SetPin, HIGH);
  delay(10);


  pinMode(Sila, OUTPUT);
  pinMode(Naprav, OUTPUT);
  digitalWrite(Naprav, HIGH);
  analogWrite(Sila, 255);
  servo44.attach(44);
  servo45.attach(45);
  nh.initNode();
  pinMode(front_end_switch, INPUT_PULLUP);
  pinMode(back_end_switch, INPUT_PULLUP);
  Serial1.begin(115200);

  nh.subscribe(subs1);
  nh.subscribe(subs2);
  nh.subscribe(subs3);
  nh.subscribe(subs4);




}

void loop() {
  
  nh.spinOnce();

  delay(1);

}