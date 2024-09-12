#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <std_msgs/String.h>

#include "Motor1.h"
#include "Motor2.h"
#include <PID_v1.h>
#define LOOPTIME 10
#define MANUAL_MODE 1
#define AUTO_MODE 0
#define LOCKED_MODE 2
#define UNLOCKED_MODE 5

typedef struct {
//  byte  R_EN;
//  byte L_EN;
  byte DIR;
  byte PWM;
  byte ENCODE_CA;
  byte ENCODE_CB;
} motor_data;


ros::NodeHandle nh;

//Motor1 right(7, 9, 6,8);         // trai
//Motor2 left(15, 16, 2, 4);       //phai

float Vx = 0, Vx_out = 0,preVx=0, preVw=0,Vxpro=0,Vwpro=0;
float alphaX = 0, alphaW = 0;
float Vw = 0, Vw_out = 0;
int mode = 0,cntVx=0,cntVw=0;
motor_data motorset1, motorset2;
double v1;
double v2;
unsigned long prevMillis;

//-----------------------
float u_in_2, u_in_1 = 0;
float ev_i_store_2 = 0, ev_i_store_1 = 0;
float v_kalman_2 = 0, v_kalman_1 = 0, P_v_kalman_2 = 1, P_v_kalman_1 = 1;
float _previousTime, _currentTime, _eT;
float _dT = 0.01;
bool motorBlocked = false;

unsigned long currentMillis;
volatile long int pos2 = 0, pos1 = 0;


float eir = 0;

//-------------------------

void cmd_vel_cb(const geometry_msgs::Twist& twist) {
  Vx = twist.linear.x;
  //Vw = twist.angular.z ;
  mode = twist.linear.y;
  Vw = 0.17 * twist.angular.z;
  //    if (Vw == -1 or Vw == 1)/ Vw = 0.5;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);

//char buffer[50];
//
//std_msgs::String str_msg;
//ros::Publisher pub("chatter", &str_msg);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);
  //    nh.advertise(pub);
  motorBlocked = false;
  ev_i_store_2 = 0; P_v_kalman_2 = 0;
  ev_i_store_1 = 0; P_v_kalman_1 = 0;

  cau_hinh_motor();
  // pinMode(18, INPUT);
  // pinMode(19, INPUT);
  // pinMode(20, INPUT);
  // pinMode(23, INPUT);

  // attachInterrupt(digitalPinToInterrupt(18), ReadEncoderL, RISING);
  // attachInterrupt(digitalPinToInterrupt(20), ReadEncoderR, RISING);

  pinMode(motorset1.ENCODE_CA, INPUT);
  pinMode(motorset1.ENCODE_CB, INPUT);
  pinMode(motorset2.ENCODE_CA, INPUT);
  pinMode(motorset2.ENCODE_CB, INPUT);

  pinMode(motorset1.DIR, OUTPUT);
  pinMode(motorset1.PWM, OUTPUT);
  pinMode(motorset2.DIR, OUTPUT);
  pinMode(motorset2.PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(motorset2.ENCODE_CA), ReadEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(motorset1.ENCODE_CA), ReadEncoder1, RISING);

  Serial.begin(57600);
  while (!Serial) {
    ; // đợi cổng serial kết nối. Chỉ cần thiết cho cổng USB gốc
  }
  delay(500);
  _currentTime = micros();
}

void loop() {
  //    currentMillis = millis();
  //    if (currentMillis - prevMillis >= LOOPTIME) {
  //--------------------------------------- LẤY GIÁ TRỊ ENCODER TRƯỚC VÀ SAU BÁNH TRÁI ----------------------------------
  //        /int p_posL = get_EncoderL();
  //        /int p_posR = get_EncoderR();

  _previousTime = _currentTime;
  _currentTime = micros();
  _eT = (_currentTime - _previousTime) / 1000000.0; // Chia cho 1000 để lấy giây

  if (_eT < 0) 
    _eT = _dT;

  while (_eT < _dT) {
    _currentTime = micros();
    _eT = (_currentTime - _previousTime) / 1000000.0;
  }

//if (Vx > 0.5) Vx = 0.5;
//if (Vx < -0.5) Vx = -0.5;
//if (Vw > 0.3) Vw = 0.3;
//if (Vw < -0.3) Vw = -0.3;

//  int same_max = 20;
//  if(mode == 0){
//
//if (Vx > 0.5) Vx = 0.5;
//if (Vx < -0.5) Vx = -0.5;
//if (Vw > 0.15) Vw = 0.15;
//if (Vw < -0.15) Vw = -0.15;
//    
//if ((preVx==Vx)&& (fabs(Vx) > 0.25))
//  cntVx++;
//else cntVx = 0;
//
//if(cntVx >= same_max)
//{
//  cntVx=same_max; 
//  Vxpro=0;
// }
//else Vxpro = Vx;
//
//preVx=Vx;
//
//if ((preVw==Vw)&& (fabs(Vw) > 0.05))
//  cntVw++;
//else cntVw = 0;
//
//if(cntVw >= same_max)
//{
//  cntVw=same_max; 
//  Vwpro=0;
//  }
//else Vwpro = Vw;
//preVw=Vw;
//  }
//  else
//  {
//    Vwpro = Vw;
//    Vxpro = Vx;
//   }

  int p_pos2_current = get_Encoder2();
  int p_pos1_current = get_Encoder1();
//Serial.println(p_pos2_current);

  //------------------------------------------------------------------------------------------------------

  //        float vel_posL_rpm = (((float)(p_posL_current - p_posL) / _eT) / 691.2) * 60;         //  xung/s  chia   xung/vòng  --> vòng/phút
  //        float vel_posR_rpm = (((float)(p_posR_current - p_posR) / _eT) / 691.2) * 60;


  float vel_pos2_rpm = (( ((float)(p_pos2_current)) / _eT) / 345.6) * 60;         //  xung/s  chia   xung/vòng  --> vòng/phút                 xung/vong = 19.2 * 12 * tile pully
  float vel_pos1_rpm = (( ((float)(p_pos1_current)) / _eT) / 345.6) * 60;

  float vel_pos2_mps = (vel_pos2_rpm / 60) / (1 / (3.14 * 0.145));  // vòng/s   chia    vòng/mét  --> m/s
  float vel_pos1_mps = (vel_pos1_rpm / 60) / (1 / (3.14 * 0.145));

  //        publishSpeed(ev_i_store_L, ((float)(p_posL_current - p_posL) // _eT),  ((float)(p_posR_current - p_posR) / _eT));


  //---------------------------------------- LỌC KALMAN -----------------------------------------------
  //        v_kalman_L = kalmanFilter_bac1(v_kalman_L, vel_posL_ppm, &P_v_kalman_L, 0.1, 5);
  //        v_kalman_R = kalmanFilter_bac1(v_kalman_R, vel_posR_ppm, &P_v_kalman_R, 0.1, 5);

  v_kalman_2 = kalmanFilter_bac1(v_kalman_2, vel_pos2_mps, &P_v_kalman_2, 0.1, 5); //mps
  v_kalman_1 = kalmanFilter_bac1(v_kalman_1, vel_pos1_mps, &P_v_kalman_1, 0.1, 5); //mps

  //pid cao
  //Vx = -0.3; Vw = 0;
  float vxm = 0.5 * (v_kalman_2 + v_kalman_1);
  float vwm = -(v_kalman_1 - v_kalman_2);
//Vx=0; Vw=-0.5;
  float vxc = 0.7 * Vx + 0.7 * (Vx - vxm);
//  if (vxc>0.5)
//  vxc=0.5;
//  if (vxc <-0.5)
//  vxc=-0.5;
  float vwc = 0.7 * Vw + 0.7* (Vw - vwm);
//  if (vwc>2)
//  vwc=2;
//  if( vwc <-2)
//  vwc=-2;
//vxc=Vx;
//vwc=Vw;
  //---------------------------------------------- BỘ ĐIỀU KHIỂN PID ------------------------------------------------
  // int vd = 30;
  /*------------- Tính toán V từ Ros------------------*/
  if (mode == MANUAL_MODE) //Manual mode
  { alphaX = 0.01; alphaW = 0.01;
  }
  else //Automode
  {
    alphaX = 0.001 / (0.1 + fabs(Vx)); //0.01
    alphaW = 0.001 / (0.01 + fabs(Vw)); //0.01
  }

  Vx_out = (1 - alphaX) * Vx_out + alphaX * vxc;
  Vw_out = (1 - alphaW) * Vw_out + alphaW * vwc;

  double motor2Speed = (Vx_out + Vw_out); // Động cơ trái
  double motor1Speed = (Vx_out - Vw_out); // Động cơ phải

  // motorLSpeed = -0.4; motorRSpeed = 0.2;
  /*--------------------------------*/
  if(mode == UNLOCKED_MODE)
    motorBlocked = false;
  if(mode == LOCKED_MODE)
    motorBlocked = true;
   //SET CHE DO LOCK VAF UNLOCK DONG CO
  if(motorBlocked)
  {
    motor2Speed = 0; motor1Speed = 0; //DUNG DONG CO
  }

   
  u_in_2 = PI_VEL_CONTROLLER(motor2Speed, v_kalman_2, &ev_i_store_2, 300*0.7, 20, _dT, 255, -255); //100 2     350   30 ---// 300 40
  u_in_1 = PI_VEL_CONTROLLER(motor1Speed, v_kalman_1, &ev_i_store_1, 300*0.7, 20, _dT, 255, -255);

  // Gọi hàm publishSpeed để xuất bản dữ liệu
  //        publishSpeed(motor1Speed, v_kalman_L, u_in_L);


  float dead_zone_dung = 0.03;
  //        publishSpeed(ev_i_store_L, (p_posL_current - p_posL) / _eT,  (p_posR_current - p_posR) / _eT);

  if ((motor2Speed < dead_zone_dung && motor2Speed > -dead_zone_dung) && (v_kalman_2 < dead_zone_dung && v_kalman_2 > -dead_zone_dung)) {
   // u_in_2 = 0;
  //  ev_i_store_2 = 0;
  ev_i_store_2 = 0.3*ev_i_store_2; // 0.9
  }
  if ((motor1Speed < dead_zone_dung && motor1Speed > -dead_zone_dung) && (v_kalman_1 < dead_zone_dung && v_kalman_1 > -dead_zone_dung)) {
   // u_in_1 = 0;
   // ev_i_store_1 = 0;
   ev_i_store_1 = 0.3*ev_i_store_1; //0.9
  }
  
u_in_2 =-30;u_in_1 =-30;
  Serial.println("test1: "); Serial.println(v_kalman_1); Serial.println(v_kalman_2);//Serial.println(u_in_2);

 publishSpeed(vxm, vwm,  mode);
  //      publishSpeed(eir ,motor1Speed-v_kalman_L,  u_in_L);
 // u_in_2 = 30;
  // u_in_1= -10;
  // left.rotate2(u_in_L);  // L
 

  rotate_motor(2, u_in_2);
  //       right.rotate1(u_in_R);  // R
  rotate_motor(1, u_in_1);


  //        prevMillis = currentMillis;
  //    }

  nh.spinOnce();
}
