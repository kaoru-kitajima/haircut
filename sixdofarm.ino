#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <VerSpeedServo.h>
//駆動に必要な最低PCMパルス数
#define L_PULSE_START 80 //駆動に必要な最低PCMパルス数
#define R_PULSE_START 80 //駆動に必要な最低PCMパルス数
#define PULSE_1MPS 80 //1.0m/sに必要なPCMパルス数

ros::NodeHandle nh;
int cmdvel_cnt=0;
int l_motor=0, r_motor=0;
std_msgs::Int32 count_msg;
void MotorCmdCallback(const geometry_msgs::Twist& msg){
cmdvel_cnt = 0;

// 進行方向設定
if(msg.linear.x > 0.0){
  l_motor = (int)(L_PULSE_START + PULSE_1MPS * msg.linear.x);
  r_motor = (int)(R_PULSE_START + PULSE_1MPS * msg.linear.x);
}else if(msg.linear.x < 0.0){
  l_motor = (int)(-1*L_PULSE_START + PULSE_1MPS * msg.linear.x);
  r_motor = (int)(-1*R_PULSE_START + PULSE_1MPS * msg.linear.x);
}else{
  l_motor = 0;
  r_motor = 0;
}
// 角度方向設定
if(msg.angular.z > 0.0){
  if(msg.linear.x == 0.0){
    l_motor = (int)(-1*L_PULSE_START - PULSE_1MPS * msg.angular.z * 0.75 /2);
    r_motor = (int)( R_PULSE_START + PULSE_1MPS * msg.angular.z * 0.75 /2); 
  }else if(msg.linear.x > 0.0){
    r_motor += PULSE_1MPS * msg.angular.z;
    l_motor -= PULSE_1MPS * msg.angular.z*0.5;
  }else{
    r_motor -= PULSE_1MPS * msg.angular.z;
    l_motor += PULSE_1MPS * msg.angular.z*0.5;
   }
 }else if(msg.angular.z < 0.0){
    if(msg.linear.x == 0.0){
      l_motor = (int)( L_PULSE_START + PULSE_1MPS * msg.angular.z * -0.75 /2);
      r_motor = (int)( -1 * R_PULSE_START - PULSE_1MPS * msg.angular.z * -0.75 /2); 
    }else if(msg.linear.x > 0.0){
      l_motor += PULSE_1MPS * msg.angular.z * -1;
      r_motor -= PULSE_1MPS * msg.angular.z*0.5 * -1;
  } else{
      l_motor -= PULSE_1MPS * msg.angular.z * -1;
      r_motor += PULSE_1MPS * msg.angular.z*0.5 * -1;
  }
}
  //限度設定
  if(l_motor > 255) l_motor = 255;
  if(l_motor < -255) l_motor = -255;
  if(r_motor > 255) r_motor = 255;
  if(r_motor < -255) r_motor = -255;
return;
}
bool MotorRun(int LS,int RS){ 
  if(LS >= 0 && LS <= 255){
    analogWrite(5, LS); 
    digitalWrite(14, HIGH); 
    digitalWrite(15, LOW);
  }
  if(LS < 0 && LS >= -255){
    analogWrite(5, abs(LS)); 
    digitalWrite(14, LOW); 
    digitalWrite(15, HIGH);
  } 
  if(RS >= 0 && RS <= 255){
    analogWrite(6, RS); 
    digitalWrite(17, HIGH); 
    digitalWrite(16, LOW);
  }
  if(RS < 0 && RS >= -255){
    analogWrite(6, RS); 
    digitalWrite(17, LOW); 
    digitalWrite(16, HIGH);
  }
  if  (RS > 255 || RS < -255 || LS > 255 || LS < -255){
    return false;
  } 
  return true;
}

bool MotorBrake(){ 
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
}
ros::Subscriber<geometry_msgs::Twist> sub_cmdvel("alphabot/cmd_vel", MotorCmdCallback);
void setup(){
  nh.initNode();
  nh.subscribe(sub_cmdvel);
}

void loop(){
//cmd_vel設定の反映
  if(cmdvel_cnt <= 20){ //cmd_vel命令が 1000ms(50*20)以内の場合
    MotorRun(l_motor, r_motor); 
    cmdvel_cnt++; 
  }else{
    MotorBrake(); 
  }
nh.spinOnce();
delay(50);
}
