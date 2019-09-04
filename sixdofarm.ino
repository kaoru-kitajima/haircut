#include <LiquidCrystal.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <VarSpeedServo.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
//#include <std_msgs/MultiArrayDimension.h>
//#include <std_msgs/MultiArrayLayout.h>
#define MaxVel 100
#define Debug 1 //サーボを繋がないなら1、つなぐなら0
int led = 13;
VarSpeedServo servo1;
VarSpeedServo servo2;
VarSpeedServo servo3;
VarSpeedServo servo4;
VarSpeedServo servo5;
VarSpeedServo servo6;

ros::NodeHandle nh;

std_msgs::Float32MultiArray msg;
ros::Publisher responser("response", &msg);
void messageCb(const std_msgs::Float32MultiArray& msg_sub)
{
  servo1.write(msg_sub.data[0], MaxVel);
  servo2.write(msg_sub.data[1], MaxVel);
  servo3.write(msg_sub.data[2], MaxVel);
  servo4.write(msg_sub.data[3], MaxVel);
  servo5.write(msg_sub.data[4], MaxVel);
  servo6.write(msg_sub.data[5], MaxVel);
  servo1.wait();
  servo2.wait();
  servo3.wait();
  servo4.wait();
  servo5.wait();
  servo6.wait();
  msg.data[0] = servo1.read();
  msg.data[1] = servo1.read();
  msg.data[2] = servo1.read();
  msg.data[3] = servo1.read();
  msg.data[4] = servo1.read();
  msg.data[5] = servo1.read();
  responser.publish( &msg );

}
//同じ数字を送り返す。
void messageCbDebug(const std_msgs::Float32MultiArray& msg_sub)
{
  msg.data[0] =msg_sub.data[0];
  msg.data[1] =msg_sub.data[1];
  msg.data[2] =msg_sub.data[2];
  msg.data[3] =msg_sub.data[3];
  msg.data[4] =msg_sub.data[4];
  msg.data[5] =msg_sub.data[5];
  responser.publish( &msg );
  if(digitalRead(led)==LOW)digitalWrite(led, HIGH);
  else digitalWrite(led, LOW);
}
ros::Subscriber<std_msgs::Float32MultiArray> sub("request", (Debug ? &messageCbDebug : &messageCb));

void initpos()  //initialization
{
  int sea = servo1.read();
  int seb = servo2.read();
  int sec = servo3.read();
  int sed = servo4.read();
  int see = servo5.read();
  int sef = servo6.read();

  servo1.write(66, abs(sea - 66));
  servo2.write(60, abs(seb - 60));
  servo3.write(20, abs(sec - 20));
  servo4.write(60, abs(sed - 60));
  servo5.write(90, abs(see - 90));
  servo6.write(90, abs(sef - 90));
}

void setup()
{
  servo1.attach(3);  //  Control waist (A) port number is   3
  servo2.attach(5);  //  Control lorearm（B）port number is 5
  servo3.attach(6);  //  Control  Forearm（C）port number is 6
  servo4.attach(9);  // Control Forearm rotation (D) port number is 9
  servo5.attach(10); // Control wrist（E）port number is 10 wrist
  servo6.attach(11); // Control wrist rotation (F) port number is 9
  initpos();

  pinMode(led, OUTPUT);
  msg.data_length = 6;
  msg.data = (float *)malloc(sizeof(float)*6);
  nh.getHardware()->setBaud(921600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(responser);
}


void loop()
{
  nh.spinOnce();
}
