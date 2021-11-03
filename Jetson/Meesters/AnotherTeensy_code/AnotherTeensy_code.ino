#include "interpo.h"
#include "InverseKin.h"
#include "ForwKin.h"

InverseKinematics InKin;

//Some variables

//Joint Angle Variables for SetNextpathPoint Mode
float theta1[6];
float theta2[6];
float theta3[6];


//Joint Angle Variables for SetAngle Mode
float A_theta1[6];
float A_theta2[6];
float A_theta3[6];


//Joint Angle Variables for Teleop demo Mode
float T_theta1[6];
float T_theta2[6];
float T_theta3[6];


// Dynamixel
#include <MyDynamixel.h>
#define DXL_SERIAL Serial5
const uint8_t DEPin = 19; // DYNAMIXEL DIR PIN
MyDynamixel dxl(DXL_SERIAL, 1000000, DEPin);

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
std_msgs::Float64 motor_feedback[18];
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

//LOG data to file publisher
std_msgs::String logdata;
ros::Publisher pub_log("LOGDATA", &logdata);

// ** ROS callback & subscriber **

//Change angle of joints subscriber
void angle_command_cb( const my_message::thetaMessage& msg)
{
  A_theta1[0] = msg.th1_1;
  A_theta1[1] = msg.th1_2;
  A_theta1[2] = msg.th1_3;
  A_theta1[3] = msg.th1_4;
  A_theta1[4] = msg.th1_5;
  A_theta1[5] = msg.th1_6;

  A_theta2[0] = msg.th2_1;
  A_theta2[1] = msg.th2_2;
  A_theta2[2] = msg.th2_3;
  A_theta2[3] = msg.th2_4;
  A_theta2[4] = msg.th2_5;
  A_theta2[5] = msg.th2_6;

  A_theta3[0] = msg.th3_1;
  A_theta3[1] = msg.th3_2;
  A_theta3[2] = msg.th3_3;
  A_theta3[3] = msg.th3_4;
  A_theta3[4] = msg.th3_5;
  A_theta3[5] = msg.th3_6;
  
}
ros::Subscriber<my_message::thetaMessage> rosSub("/simple_hexapod/Th_position_controller/command", angle_command_cb);


//Sub to teleop_keyboard for demo stuff
int tx=0,ty=0,tz=0;
float troll=0,tpitch=0,tyaw=0;
void teleop_cb(const geometry_msgs::Twist& msg)
{
  //lean x direction
  if (msg.linear.x > 0 && msg.linear.x <= 0.5)
  {
    tx = tx + 1;
  }
  else if (msg.linear.x < 0 && msg.linear.x >= -0.5)
  {
    tx = tx - 1;
  }
  //lean y direction
  if (msg.linear.y > 0 && msg.linear.y <= 0.5)
  {
    ty = ty + 1;
  }
  else if (msg.linear.y < 0 && msg.linear.y >= -0.5)
  {
    ty = ty - 1;
  }
  //Change height
  if (msg.linear.z > 0)
  {
    tz = tz + 1;
  }
  else if (msg.linear.z < 0)
  {
    tz = tz - 1;
  }
  //Yaw movement
  if (msg.angular.z > 0)
  {
    tyaw = tyaw + 0.25;
  }
  else if (msg.angular.z < 0)
  {
    tyaw = tyaw - 0.25;
  }
  //Roll movement
  if (msg.linear.x > 0.5)
  {
    troll = troll + 0.25;
  }
  else if (msg.linear.x < -0.5)
  {
    troll = troll - 0.25;
  }
  //Pitch movement
  if (msg.linear.y > 0.5)
  {
    tpitch = tpitch + 0.25;
  }
  else if (msg.linear.y < -0.5)
  {
    tpitch = tpitch - 0.25;
  }
}
ros::Subscriber<geometry_msgs::Twist> rosSubTeleop("/cmd_vel", teleop_cb);

//Leg Path subsciber
#define Pathsize 7
float XPath[6][Pathsize*2-2];
float YPath[6][Pathsize*2-2];
float ZPath[6][Pathsize*2-2];
float TurnPath[Pathsize*2-2];
float dt=0;
int startSetPath = 0;
void legpath_cb(const my_message::LegPath& msg)
{

  for (int col=0;col<(Pathsize*2-2);col++)
  {
    XPath[0][col]=msg.PathL0x[col];
    XPath[1][col]=msg.PathL1x[col];
    XPath[2][col]=msg.PathL2x[col];
    XPath[3][col]=msg.PathL3x[col];
    XPath[4][col]=msg.PathL4x[col];
    XPath[5][col]=msg.PathL5x[col];

    YPath[0][col]=msg.PathL0y[col];
    YPath[1][col]=msg.PathL1y[col];
    YPath[2][col]=msg.PathL2y[col];
    YPath[3][col]=msg.PathL3y[col];
    YPath[4][col]=msg.PathL4y[col];
    YPath[5][col]=msg.PathL5y[col];

    ZPath[0][col]=msg.PathL0z[col];
    ZPath[1][col]=msg.PathL1z[col];
    ZPath[2][col]=msg.PathL2z[col];
    ZPath[3][col]=msg.PathL3z[col];
    ZPath[4][col]=msg.PathL4z[col];
    ZPath[5][col]=msg.PathL5z[col];

    TurnPath[col]=msg.PathAng[col];

    dt = msg.DT;
  }

  startSetPath = 1;

}
ros::Subscriber<my_message::LegPath> rosSubPATH("/simple_hexapod/Legs_paths", legpath_cb);

//mode select subscriber
int mode = -1;
int startUp = -1;
void modeSelect_cb(const std_msgs::Float32& msg)
{
  if(msg.data == -1)
  {
    startUp = 0;
  }
  else
  {
    mode = msg.data;
  }
}
ros::Subscriber<std_msgs::Float32> rosSubModeSelect("/mode_selected", modeSelect_cb);

void setup() 
{
 Serial.begin(9600);

  // ROS setup
  nh.getHardware()->setBaud(115200);      // set baud rate to 115200
  nh.initNode();                             // init ROS
  //angle subscriber
  nh.subscribe(rosSub);
  //teleop_keyboard subscriber
  nh.subscribe(rosSubTeleop);
  //Leg Path subsciber
  nh.subscribe(rosSubPATH);
  //mode select subscriber
  nh.subscribe(rosSubModeSelect);
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
  //data log publisher
  nh.advertise(pub_log);
  //broadcaster.init(nh);       // set up broadcaster fot tf
  //nh.advertise(pub_eye);   // advertise eye topic
 
}

double Angle[18];
double Spd[18];
uint8_t Id[18];
char dataStr[100] = "";
char buff[7];
long currentmillis = 0;
long prevmillis = millis();
float x=0.0;

//global variables for setnextpathpoint
long curtime = 0;
long prevtime = 0;

int stepStartFlag = 0;
int kinematicModeStartFlag = 0;

//Function prototype
void SetAngles(float* th1,float* th2,float* th3 ,float spd1=-1,float spd2=-1, float spd3=-1);

void loop()
{
  nh.spinOnce();
  
  currentmillis = millis();

  if(currentmillis - prevmillis >= 10)
  {
    prevmillis = currentmillis;

    //On Startup
    if(startUp == 0)
    {
      static long startUp_startTime = millis();
      InKin.IK(&theta1[0],&theta2[0],&theta3[0],283.71,   0.0,    -140,0,0,0,0,0,0);
      InKin.IK(&theta1[1],&theta2[1],&theta3[1],141.855,  -245.7, -140,1,0,0,0,0,0);
      InKin.IK(&theta1[2],&theta2[2],&theta3[2],-141.855, -245.7, -140,2,0,0,0,0,0);
      InKin.IK(&theta1[3],&theta2[3],&theta3[3],-283.71,  0.0,    -140,3,0,0,0,0,0);
      InKin.IK(&theta1[4],&theta2[4],&theta3[4],-141.855, 245.7,  -140,4,0,0,0,0,0);
      InKin.IK(&theta1[5],&theta2[5],&theta3[5],141.855,  245.7,  -140,5,0,0,0,0,0);
      SetAngles(theta1,theta2,theta3,10,10,10);
      if(currentmillis - startUp_startTime >= 5000) startUp = 1;
    }

    //Teleop demo Mode
    if(mode == 0 && startUp == 1)
    {
      stepStartFlag = 0;
      static elapsedMillis kinematicModeStartTimer;

      if(kinematicModeStartFlag == 0)
      {
        kinematicModeStartFlag = 1;
        kinematicModeStartTimer = 0;
      }

      if(kinematicModeStartTimer <= 1000)
      {
        InKin.IK(&T_theta1[0],&T_theta2[0],&T_theta3[0],283.71+tx,   0.0+ty,    -140-tz,0,1000,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[1],&T_theta2[1],&T_theta3[1],141.855+tx,  -245.7+ty, -140-tz,1,1000,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[2],&T_theta2[2],&T_theta3[2],-141.855+tx, -245.7+ty, -140-tz,2,1000,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[3],&T_theta2[3],&T_theta3[3],-283.71+tx,  0.0+ty,    -140-tz,3,1000,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[4],&T_theta2[4],&T_theta3[4],-141.855+tx, 245.7+ty,  -140-tz,4,1000,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[5],&T_theta2[5],&T_theta3[5],141.855+tx,  245.7+ty,  -140-tz,5,1000,troll,tpitch,-tyaw,0);
        SetAngles(T_theta1,T_theta2,T_theta3,0,0,0);
      }
      else
      {
        InKin.IK(&T_theta1[0],&T_theta2[0],&T_theta3[0],283.71+tx,   0.0+ty,    -140-tz,0,0,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[1],&T_theta2[1],&T_theta3[1],141.855+tx,  -245.7+ty, -140-tz,1,0,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[2],&T_theta2[2],&T_theta3[2],-141.855+tx, -245.7+ty, -140-tz,2,0,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[3],&T_theta2[3],&T_theta3[3],-283.71+tx,  0.0+ty,    -140-tz,3,0,troll,tpitch,-tyaw,0);
        InKin.IK(&T_theta1[4],&T_theta2[4],&T_theta3[4],-141.855+tx, 245.7+ty,  -140-tz,4,0,troll,tpitch,tyaw,0);
        InKin.IK(&T_theta1[5],&T_theta2[5],&T_theta3[5],141.855+tx,  245.7+ty,  -140-tz,5,0,troll,tpitch,-tyaw,0);
        SetAngles(T_theta1,T_theta2,T_theta3,0,0,0);
      }
    }

    //SetAngle Mode
    else if(mode == 1 && startUp == 1)
    {
      stepStartFlag = 0;
      kinematicModeStartFlag = 0;
      
      A_theta1[0] = 0;
      A_theta2[0] = 0;
      A_theta3[0] = -0;

      float px = 0, py = 0, pz = 0;
      
      FK03_inbody(px,py,pz, theta1[0],theta2[0],theta3[0],0);
      InKin.IK(&theta1[0],&theta2[0],&theta3[0],px,   py,    pz,0,0,0,0,0,0);

      FK03_inbody(px,py,pz, theta1[1],theta2[1],theta3[1],1);
      InKin.IK(&theta1[1],&theta2[1],&theta3[1],px,   py,    pz,1,0,0,0,0,0);

      FK03_inbody(px,py,pz, theta1[2],theta2[2],theta3[2],2);
      InKin.IK(&theta1[2],&theta2[2],&theta3[2],px,   py,    pz,2,0,0,0,0,0);

      FK03_inbody(px,py,pz, theta1[3],theta2[3],theta3[3],3);
      InKin.IK(&theta1[3],&theta2[3],&theta3[3],px,   py,    pz,3,0,0,0,0,0);

      FK03_inbody(px,py,pz, theta1[4],theta2[4],theta3[4],4);
      InKin.IK(&theta1[4],&theta2[4],&theta3[4],px,   py,    pz,4,0,0,0,0,0);

      FK03_inbody(px,py,pz, theta1[5],theta2[5],theta3[5],5);
      InKin.IK(&theta1[5],&theta2[5],&theta3[5],px,   py,    pz,5,0,0,0,0,0);
      
      SetAngles(A_theta1,A_theta2,A_theta3,20,20,20);
      //mode = 2;
    }
    
    //SetNextpathPoint Mode
    else if(mode == 2 && startSetPath == 1 && startUp == 1)
    {
      kinematicModeStartFlag = 0;
      static elapsedMillis stepStartTimer;
	    if (stepStartFlag == 0)
	    {
	      stepStartTimer = 0;
	      stepStartFlag = 1;
      }
      if(stepStartTimer <= 1000)
      {
        SetNextPathPoint(XPath,YPath,ZPath,TurnPath,1000);
        SetAngles(theta1,theta2,theta3,0,0,0);
        prevtime = millis();
      }
      else
      {
        curtime = millis();
        SetNextPathPoint(XPath,YPath,ZPath,TurnPath,dt);
        SetAngles(theta1,theta2,theta3,0,0,0);
      }
    }
  }


}



// ** Functions **
void SetNextPathPoint(float XP[][Pathsize*2-2],float YP[][Pathsize*2-2],float ZP[][Pathsize*2-2],float* TurnP,float d_t)
{
  static int currentPathPoint[6] = {3,9,3,9,3,9};
  static int currentPathPoint_tw[2] = {3,3};
  float x,y,z,yaww;

  if(d_t != -1000)
  {
    for(int i = 0;i<6;i++)
    {
      x = XP[i][currentPathPoint[i]];
      y = YP[i][currentPathPoint[i]];
      z = ZP[i][currentPathPoint[i]];
      yaww = TurnP[currentPathPoint_tw[i%2]];
  
      InKin.IK(&theta1[i],&theta2[i],&theta3[i],x,y,z,i,d_t,0,0,yaww);
  
    }
  
    if(curtime - prevtime >= d_t)
    {
      prevtime = curtime;
  
      for(int i=0;i<2;i++)
      {
        if(currentPathPoint_tw[i] >= 11)
        {
          currentPathPoint_tw[i] = 0;
        }
        else
        {
          currentPathPoint_tw[i] = currentPathPoint_tw[i] + 1;
        }
      }
      
      for(int i=0;i<6;i++)
      {
        if(currentPathPoint[i] >= 11)
        {
          currentPathPoint[i] = 0;
        }
        else
        {
          currentPathPoint[i] = currentPathPoint[i] + 1;
        }
      }
    }
  }
  else
  {
    for(int i = 0;i<6;i++)
    {
      x = XP[i][currentPathPoint[i]];
      y = YP[i][currentPathPoint[i]];
      z = ZP[i][currentPathPoint[i]];
      yaww = TurnP[currentPathPoint_tw[i%2]];
  
      InKin.IK(&theta1[i],&theta2[i],&theta3[i],x,y,z,i,500,0,0,yaww);
    }
    for(int i = 0;i<6;i++)
    {
      currentPathPoint_tw[i%2] = 3;
      currentPathPoint[i] = (i%2 == 0) ? 3 : 9;
    }
    prevtime = curtime;
    stepStartFlag = 0;
  }
}


void SetAngles(float* th1,float* th2,float* th3 ,float spd1,float spd2, float spd3)
{

  static double prevAngle[18];
  
  for(int num = 0; num<18; num++)
  {
    if (num == 0) //thata11
    {
      Angle[num] = th1[0]*180.0/M_PI+150.0;
      Spd[num] = (spd1==-1) ? (abs(prevAngle[num]-Angle[num])*60.0)/(360.0*10.0/1000.0) : spd1;
      Id[num] = 0;
      
      /*if(abs(prevAngle[num]-Angle[num]) >= 0.15)*/ prevAngle[num] = Angle[num];
        
    }
    else if (num == 1)  //thata21
    {
      Angle[num] = -th2[0]*180.0/M_PI+150.0;
      Spd[num] = (spd2==-1) ? (abs(prevAngle[num]-Angle[num])*60.0)/(360.0*10.0/1000.0) : spd2;
      Id[num] = 1;
      
      Serial.print(prevAngle[num]-Angle[num]);Serial.print(" ");Serial.println(Spd[num]);
      /*if(abs(prevAngle[num]-Angle[num]) >= 0.15)*/ prevAngle[num] = Angle[num];
  
      dataStr[0] = 0; 
      itoa( millis(),buff,10); //convert long to charStr
      strcat(dataStr, buff); //add it to the end
      strcat(dataStr, ", "); //append the delimiter
      dtostrf(th2[0], 5, 5, buff);
      strcat(dataStr, buff);
      
    }
    else if (num == 2)  //thata31
    {
      Angle[num] = -th3[0]*180.0/M_PI+150.0;
      Spd[num] = (spd3==-1) ? (abs(prevAngle[num]-Angle[num])*60.0)/(360.0*10.0/1000.0) : spd3;
      Id[num] = 2;

      /*if(abs(prevAngle[num]-Angle[num]) >= 0.15)*/ prevAngle[num] = Angle[num];

    }
    else if (num == 3)  //thata12
    {
      Angle[num] = th1[1]*180.0/M_PI+150.0;
      Spd[num] = 0;
      Id[num] = 3;
 
    }
    else if (num == 4)  //thata22
    {
      Angle[num] = th2[1];
      Spd[num] = 0;
      Id[num] = 4;

    }
    else if (num == 5)  //thata32
    {
      Angle[num] = th3[1];
      Spd[num] = 0;
      Id[num] = 5;

    }
    else if (num == 6)  //thata13
    {
      Angle[num] = th1[2];
      Spd[num] = 0;
      Id[num] = 6;

    }
    else if (num == 7)  //thata23
    {
      Angle[num] = th2[2];
      Spd[num] = 0;
      Id[num] = 7;

    }
    else if (num == 8)  //thata33
    {
      Angle[num] = th3[2];
      Spd[num] = 0;
      Id[num] = 8;

    }
    else if (num == 9)  //thata14
    {
      Angle[num] = th1[3];
      Spd[num] = 0;
      Id[num] = 9;

    }
    else if (num == 10) //thata24
    {
      Angle[num] = th2[3];
      Spd[num] = 0;
      Id[num] = 10;

    }
    else if (num == 11) //thata34
    {
      Angle[num] = th3[3];
      Spd[num] = 0;
      Id[num] = 11;

    }
    else if (num == 12) //thata15
    {
      Angle[num] = th1[4];
      Spd[num] = 0;
      Id[num] = 12;

    }
    else if (num == 13) //thata25
    {
      Angle[num] = th2[4];
      Spd[num] = 0;
      Id[num] = 13;

    }
    else if (num == 14) //thata35
    {
      Angle[num] = th3[4];
      Spd[num] = 0;
      Id[num] = 14;

    }
    else if (num == 15) //thata16
    {
      Angle[num] = th1[5];
      Spd[num] = 0;
      Id[num] = 15;

    }
    else if (num == 16) //thata26
    {
      Angle[num] = th2[5];
      Spd[num] = 0;
      Id[num] = 16;

    }
    else if (num == 17) //thata36
    {
      Angle[num] = th3[5];
      Spd[num] = 0;
      Id[num] = 17;

    }
  }

  dxl.SyncMove(Id,Angle,Spd,18);
  logdata.data = dataStr;
  pub_log.publish(&logdata);

}
