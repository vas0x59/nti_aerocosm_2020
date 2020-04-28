
#include <Arduino.h>

#include <ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <Servo.h>
class NewHardware : public ArduinoHardware
{
public:
    NewHardware() : ArduinoHardware(&Serial1, 115200){};
};

ros::NodeHandle_<NewHardware> nh;


Servo servo44; //big
Servo servo45; //small


void s1(const std_msgs::Int32 &cmd_msg)
{ // servo big
    servo44.write(cmd_msg.data);
}
void s2(const std_msgs::Int32 &cmd_msg)
{ // sevo small
    servo45.write(cmd_msg.data);
}
ros::Subscriber<std_msgs::Int32> subs1("/arduino/servo1", &s1);             //servo big
ros::Subscriber<std_msgs::Int32> subs2("/arduino/servo2", &s2);             // sevo small

void setup()
{
    servo44.attach(44);
    servo45.attach(45);
    nh.initNode();
    nh.subscribe(subs1);
    nh.subscribe(subs2);
    Serial1.begin(115200);
}

void loop()
{
    nh.spinOnce();
    delay(1);
    
}