#include "interpo.h"
#include "InverseKin.h"

InverseKinematics InKin;

float theta1[6];
float theta2[6];
float theta3[6];

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

void setup() 
{
 Serial.begin(9600);

  // ROS setup
  nh.getHardware()->setBaud(115200);      // set baud rate to 115200
  nh.initNode();                             // init ROS
  //Leg Path subsciber
  nh.subscribe(rosSubPATH);
 
}

long currentmillis = 0;
long prevmillis = millis();

void loop()
{
  nh.spinOnce();

  float x_path[] = {233.71, 237.26, 254.70, 283.71, 312.72, 330.16, 333.71};
  float y_path[] = {0, 0, 0, 0, 0, 0, 0};
  float z_path[] = {-140, -131.43, -104.88, -90, -104.88, -131.43, -140};
  
  currentmillis = millis();

  if(currentmillis - prevmillis >= 10)
  {
    prevmillis = currentmillis;

    for(int i = 0; i<7; i++)
    {
      InKin.IK(&theta1[0],&theta2[0],&theta3[0],x_path[i],y_path[i],z_path[i],0,0,0,0,0,0);
//      InKin.IK(&theta1[1],&theta2[1],&theta3[1],141.855,  -245.7, -140,1,0,0,0,0,0);
//      InKin.IK(&theta1[2],&theta2[2],&theta3[2],-141.855, -245.7, -140,2,0,0,0,0,0);
//      InKin.IK(&theta1[3],&theta2[3],&theta3[3],-283.71,  0.0,    -140,3,0,0,0,0,0);
//      InKin.IK(&theta1[4],&theta2[4],&theta3[4],-141.855, 245.7,  -140,4,0,0,0,0,0);
//      InKin.IK(&theta1[5],&theta2[5],&theta3[5],141.855,  245.7,  -140,5,0,0,0,0,0);
       Serial.print(theta1[0]*180/M_PI+150);Serial.print(" ");Serial.print(-theta2[0]*180/M_PI+150);Serial.print(" ");Serial.println(-theta3[0]*180/M_PI+150);
    }

    Serial.println("---------------------------------------------");
  }
  
}
