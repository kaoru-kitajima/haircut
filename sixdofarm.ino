#include <LiquidCrystal.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <VarSpeedServo.h>
//駆動に必要な最低PCMパルス数
#define L_PULSE_START 80 //駆動に必要な最低PCMパルス数
#define R_PULSE_START 80 //駆動に必要な最低PCMパルス数
#define PULSE_1MPS 80 //1.0m/sに必要なPCMパルス数

VarSpeedServo servo1;
VarSpeedServo servo2;
VarSpeedServo servo3;
VarSpeedServo servo4;
VarSpeedServo servo5;
VarSpeedServo servo6;
void initpos()  //initialization
{
  sea = servo1.read();
  seb = servo2.read();
  sec = servo3.read();
  sed = servo4.read();
  see = servo5.read();
  sef = servo6.read();

  servo1.write(66, abs(sea - 66));
  servo2.write(60, abs(seb - 60));
  servo3.write(20, abs(sec - 20));
  servo4.write(60, abs(sed - 60));
  servo5.write(90, abs(see - 90));
  servo6.write(90, abs(sef - 90));
}

void setup()
{
  myshow = 0;
  mycomflag = 0;       // the  ARM default  state: 2 automatic operation
  myservoA.attach(3);  //  Control waist (A) port number is   3
  myservoB.attach(5);  //  Control lorearm（B）port number is 5
  myservoC.attach(6);  //  Control  Forearm（C）port number is 6
  myservoD.attach(9);  // Control Forearm rotation (D) port number is 9
  myservoE.attach(10); // Control wrist（E）port number is 10 wrist
  myservoF.attach(11); // Control wrist rotation (F) port number is 9

  myservoA.write(66);
  myservoB.write(60);
  myservoC.write(20);
  myservoD.write(60);
  myservoE.write(90);
  myservoF.write(90);

}
ros::NodeHandle nh;
int cmdvel_cnt = 0;
int l_motor = 0, r_motor = 0;
std_msgs::Int32 count_msg;
void PosCmdCallback(const geometry_msgs::Twist& msg) {
  cmdvel_cnt = 0;

  // 進行方向設定
  if (msg.linear.x > 0.0) {
    l_motor = (int)(L_PULSE_START + PULSE_1MPS * msg.linear.x);
    r_motor = (int)(R_PULSE_START + PULSE_1MPS * msg.linear.x);
  } else if (msg.linear.x < 0.0) {
    l_motor = (int)(-1 * L_PULSE_START + PULSE_1MPS * msg.linear.x);
    r_motor = (int)(-1 * R_PULSE_START + PULSE_1MPS * msg.linear.x);
  } else {
    l_motor = 0;
    r_motor = 0;
  }
  // 角度方向設定
  if (msg.angular.z > 0.0) {
    if (msg.linear.x == 0.0) {
      l_motor = (int)(-1 * L_PULSE_START - PULSE_1MPS * msg.angular.z * 0.75 / 2);
      r_motor = (int)( R_PULSE_START + PULSE_1MPS * msg.angular.z * 0.75 / 2);
    } else if (msg.linear.x > 0.0) {
      r_motor += PULSE_1MPS * msg.angular.z;
      l_motor -= PULSE_1MPS * msg.angular.z * 0.5;
    } else {
      r_motor -= PULSE_1MPS * msg.angular.z;
      l_motor += PULSE_1MPS * msg.angular.z * 0.5;
    }
  } else if (msg.angular.z < 0.0) {
    if (msg.linear.x == 0.0) {
      l_motor = (int)( L_PULSE_START + PULSE_1MPS * msg.angular.z * -0.75 / 2);
      r_motor = (int)( -1 * R_PULSE_START - PULSE_1MPS * msg.angular.z * -0.75 / 2);
    } else if (msg.linear.x > 0.0) {
      l_motor += PULSE_1MPS * msg.angular.z * -1;
      r_motor -= PULSE_1MPS * msg.angular.z * 0.5 * -1;
    } else {
      l_motor -= PULSE_1MPS * msg.angular.z * -1;
      r_motor += PULSE_1MPS * msg.angular.z * 0.5 * -1;
    }
  }
  //限度設定
  if (l_motor > 255) l_motor = 255;
  if (l_motor < -255) l_motor = -255;
  if (r_motor > 255) r_motor = 255;
  if (r_motor < -255) r_motor = -255;
  return;
}
bool MotorRun(int LS, int RS) {
  if (LS >= 0 && LS <= 255) {
    analogWrite(5, LS);
    digitalWrite(14, HIGH);
    digitalWrite(15, LOW);
  }
  if (LS < 0 && LS >= -255) {
    analogWrite(5, abs(LS));
    digitalWrite(14, LOW);
    digitalWrite(15, HIGH);
  }
  if (RS >= 0 && RS <= 255) {
    analogWrite(6, RS);
    digitalWrite(17, HIGH);
    digitalWrite(16, LOW);
  }
  if (RS < 0 && RS >= -255) {
    analogWrite(6, RS);
    digitalWrite(17, LOW);
    digitalWrite(16, HIGH);
  }
  if  (RS > 255 || RS < -255 || LS > 255 || LS < -255) {
    return false;
  }
  return true;
}

bool MotorBrake() {
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
}
ros::Subscriber<geometry_msgs::Twist> sub_cmdvel("alphabot/cmd_vel", MotorCmdCallback);
void setup() {
  nh.initNode();
  nh.subscribe(sub_cmdvel);
}

void loop() {
  //cmd_vel設定の反映
  if (cmdvel_cnt <= 20) { //cmd_vel命令が 1000ms(50*20)以内の場合
    MotorRun(l_motor, r_motor);
    cmdvel_cnt++;
  } else {
    MotorBrake();
  }
  nh.spinOnce();
  delay(50);
}
