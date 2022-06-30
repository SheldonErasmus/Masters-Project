#include <MyDynamixel.h>
#define DXL_SERIAL Serial5
const uint8_t DEPin = 19; // DYNAMIXEL DIR PIN
MyDynamixel dxl(DXL_SERIAL, 1000000, DEPin);

float theta1[6] = {0,0,0,0,0,0};
float theta2[6] = {0,0,0,0,0,0};
float theta3[6] = {-1.5, -1.5, -1.5, -1.5, -1.5, -1.5};

long currentmillis = 0;
long prevmillis = millis();

void SetAngles(float* th1,float* th2,float* th3 ,float spd1=0,float spd2=0, float spd3=0);

//ROS
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <my_message/LegPath.h>
#include <my_message/thetaMessage.h>

ros::NodeHandle nh;

//Feedback publishers
std_msgs::String motor_feedback[18];
ros::Publisher pub_m_feed0("motor_feedback11", &motor_feedback[0]);
ros::Publisher pub_m_feed1("motor_feedback21", &motor_feedback[1]);
ros::Publisher pub_m_feed2("motor_feedback31", &motor_feedback[2]);
ros::Publisher pub_m_feed3("motor_feedback12", &motor_feedback[3]);
ros::Publisher pub_m_feed4("motor_feedback22", &motor_feedback[4]);
ros::Publisher pub_m_feed5("motor_feedback32", &motor_feedback[5]);
ros::Publisher pub_m_feed6("motor_feedback13", &motor_feedback[6]);
ros::Publisher pub_m_feed7("motor_feedback23", &motor_feedback[7]);
ros::Publisher pub_m_feed8("motor_feedback33", &motor_feedback[8]);
ros::Publisher pub_m_feed9("motor_feedback14", &motor_feedback[9]);
ros::Publisher pub_m_feed10("motor_feedback24", &motor_feedback[10]);
ros::Publisher pub_m_feed11("motor_feedback34", &motor_feedback[11]);
ros::Publisher pub_m_feed12("motor_feedback15", &motor_feedback[12]);
ros::Publisher pub_m_feed13("motor_feedback25", &motor_feedback[13]);
ros::Publisher pub_m_feed14("motor_feedback35", &motor_feedback[14]);
ros::Publisher pub_m_feed15("motor_feedback16", &motor_feedback[15]);
ros::Publisher pub_m_feed16("motor_feedback26", &motor_feedback[16]);
ros::Publisher pub_m_feed17("motor_feedback36", &motor_feedback[17]);

void setup() 
{
  Serial.begin(115200);

   // ROS setup
  nh.getHardware()->setBaud(115200);      // set baud rate to 115200
  nh.initNode();                             // init ROS

  //feedback publisher
  nh.advertise(pub_m_feed0);
  nh.advertise(pub_m_feed1);
  nh.advertise(pub_m_feed2);
  nh.advertise(pub_m_feed3);
  nh.advertise(pub_m_feed4);
  nh.advertise(pub_m_feed5);
  nh.advertise(pub_m_feed6);
  nh.advertise(pub_m_feed7);
  nh.advertise(pub_m_feed8);
  nh.advertise(pub_m_feed9);
  nh.advertise(pub_m_feed10);
  nh.advertise(pub_m_feed11);
  nh.advertise(pub_m_feed12);
  nh.advertise(pub_m_feed13);
  nh.advertise(pub_m_feed14);
  nh.advertise(pub_m_feed15);
  nh.advertise(pub_m_feed16);
  nh.advertise(pub_m_feed17);

  
}

long tic,toc;
int once =1;

void loop() 
{
  nh.spinOnce();
  currentmillis = millis();
  if( once && (currentmillis - prevmillis >= 10000))
  {
    once=0;
    prevmillis = currentmillis;
  }
  
  if((currentmillis - prevmillis >= 10) && once==0)
  {
    prevmillis = currentmillis;
    tic = micros();
    SetAngles(theta1,theta2,theta3,5,5,5);
    toc = micros();
    Serial.println(toc-tic);
  }

}

double Angle[18];
double Spd[18];
uint8_t Id[18];
uint8_t x = 0;

void SetAngles(float* th1,float* th2,float* th3 ,float spd1,float spd2, float spd3)
{
  
  long milli = millis();
  String mfb0,mfb1,mfb2,mfb3,mfb4,mfb5,mfb6,mfb7,mfb8,mfb9,mfb10,mfb11,mfb12,mfb13,mfb14,mfb15,mfb16,mfb17;
  
  for(int num = 0; num<18; num++)
  {
    if (num == 0) //thata11
    {
      Angle[num] = th1[0]*180.0/M_PI+150.0;
      Spd[num] = spd1;
      Id[num] = 0;
      
      mfb0 = String(milli) + "," + String(th1[0],2) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 1)  //thata21
    {
      Angle[num] = -th2[0]*180.0/M_PI+150.0;
      Spd[num] = spd2;
      Id[num] = 1;

      mfb1 = String(milli) + "," + String(th2[0]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 2)  //thata31
    {
      Angle[num] = -th3[0]*180.0/M_PI+150.0;
      Spd[num] = spd3;
      Id[num] = 2;
      
      mfb2 = String(milli) + "," + String(th3[0]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 3)  //thata12
    {
      Angle[num] = th1[1]*180.0/M_PI+150.0;
      Spd[num] = spd1;
      Id[num] = 3;

      mfb3 = String(milli) + "," + String(th1[1]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 4)  //thata22
    {
      Angle[num] = -th2[1]*180.0/M_PI+150.0;
      Spd[num] = spd2;
      Id[num] = 4;

      mfb4 = String(milli) + "," + String(th2[1]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 5)  //thata32
    {
      Angle[num] = -th3[1]*180.0/M_PI+150.0;
      Spd[num] = spd3;
      Id[num] = 5;

      mfb5 = String(milli) + "," + String(th3[1]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 6)  //thata13
    {
      Angle[num] = th1[2]*180.0/M_PI+150.0;
      Spd[num] = spd1;
      Id[num] = 6;

      mfb6 = String(milli) + "," + String(th1[2]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 7)  //thata23
    {
      Angle[num] = -th2[2]*180.0/M_PI+150.0;
      Spd[num] = spd2;
      Id[num] = 7;

      mfb7 = String(milli) + "," + String(th2[2]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 8)  //thata33
    {
      Angle[num] = -th3[2]*180.0/M_PI+150.0;
      Spd[num] = spd3;
      Id[num] = 8;

      mfb8 = String(milli) + "," + String(th3[2]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 9)  //thata14
    {
      Angle[num] = th1[3]*180.0/M_PI+150.0;
      Spd[num] = spd1;
      Id[num] = 9;

      mfb9 = String(milli) + "," + String(th1[3]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 10) //thata24
    {
      Angle[num] = -th2[3]*180.0/M_PI+150.0;
      Spd[num] = spd2;
      Id[num] = 10;

      mfb10 = String(milli) + "," + String(th2[3]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 11) //thata34
    {
      Angle[num] = -th3[3]*180.0/M_PI+150.0;
      Spd[num] = spd3;
      Id[num] = 11;

      mfb11 = String(milli) + "," + String(th3[3]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 12) //thata15
    {
      Angle[num] = th1[4]*180.0/M_PI+150.0;
      Spd[num] = spd1;
      Id[num] = 12;

      mfb12 = String(milli) + "," + String(th1[4]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 13) //thata25
    {
      Angle[num] = -th2[4]*180.0/M_PI+150.0;
      Spd[num] = spd2;
      Id[num] = 13;

      mfb13 = String(milli) + "," + String(th2[4]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 14) //thata35
    {
      Angle[num] = -th3[4]*180.0/M_PI+150.0;
      Spd[num] = spd3;
      Id[num] = 14;

      mfb14 = String(milli) + "," + String(th3[4]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 15) //thata16
    {
      Angle[num] = th1[5]*180.0/M_PI+150.0;
      Spd[num] = spd1;
      Id[num] = 15;

      mfb15 = String(milli) + "," + String(th1[5]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 16) //thata26
    {
      Angle[num] = -th2[5]*180.0/M_PI+150.0;
      Spd[num] = spd2;
      Id[num] = 16;

      mfb16 = String(milli) + "," + String(th2[5]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
    else if (num == 17) //thata36
    {
      Angle[num] = -th3[5]*180.0/M_PI+150.0;
      Spd[num] = spd3;
      Id[num] = 17;

      mfb17 = String(milli) + "," + String(th3[5]) + "," + String((dxl.PresentPos(Id[num])-150)*M_PI/180,2);
    }
  }

  dxl.SyncMove(Id,Angle,Spd,18);
  pubAngleFB(mfb0,mfb1,mfb2,mfb3,mfb4,mfb5,mfb6,mfb7,mfb8,mfb9,mfb10,mfb11,mfb12,mfb13,mfb14,mfb15,mfb16,mfb17);
  
}

void pubAngleFB(String &mfb0,String &mfb1,String &mfb2,String &mfb3,String &mfb4,String &mfb5,String &mfb6,String &mfb7,String &mfb8,String &mfb9,String &mfb10,String &mfb11,String &mfb12,String &mfb13,String &mfb14,String &mfb15,String &mfb16,String &mfb17)
{
  
  motor_feedback[0].data = mfb0.c_str();
  pub_m_feed0.publish(&motor_feedback[0]);
  
  motor_feedback[1].data = mfb1.c_str();
  pub_m_feed1.publish(&motor_feedback[1]);
  
  motor_feedback[2].data = mfb2.c_str();
  pub_m_feed2.publish(&motor_feedback[2]);
  
  motor_feedback[3].data = mfb3.c_str();
  pub_m_feed3.publish(&motor_feedback[3]);
  
  motor_feedback[4].data = mfb4.c_str();
  pub_m_feed4.publish(&motor_feedback[4]);
  
  motor_feedback[5].data = mfb5.c_str();
  pub_m_feed5.publish(&motor_feedback[5]);
  
  motor_feedback[6].data = mfb6.c_str();
  pub_m_feed6.publish(&motor_feedback[6]);
  
  motor_feedback[7].data = mfb7.c_str();
  pub_m_feed7.publish(&motor_feedback[7]);
  
  motor_feedback[8].data = mfb8.c_str();
  pub_m_feed8.publish(&motor_feedback[8]);
  
  motor_feedback[9].data = mfb9.c_str();
  pub_m_feed9.publish(&motor_feedback[9]);
  
  motor_feedback[10].data = mfb10.c_str();
  pub_m_feed10.publish(&motor_feedback[10]);
  
  motor_feedback[11].data = mfb11.c_str();
  pub_m_feed11.publish(&motor_feedback[11]);
  
  motor_feedback[12].data = mfb12.c_str();
  pub_m_feed12.publish(&motor_feedback[12]);
  
  motor_feedback[13].data = mfb13.c_str();
  pub_m_feed13.publish(&motor_feedback[13]);
  
  motor_feedback[14].data = mfb14.c_str();
  pub_m_feed14.publish(&motor_feedback[14]);
  
  motor_feedback[15].data = mfb15.c_str();
  pub_m_feed15.publish(&motor_feedback[15]);
  
  motor_feedback[16].data = mfb16.c_str();
  pub_m_feed16.publish(&motor_feedback[16]);
  
  motor_feedback[17].data = mfb17.c_str();
  pub_m_feed17.publish(&motor_feedback[17]);
  
}
