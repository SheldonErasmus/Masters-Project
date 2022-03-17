// Dynamixel
#include <MyDynamixel.h>
#define DXL_SERIAL Serial5

const uint8_t DEPin = 19; // DYNAMIXEL DIR PIN

//ROS
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

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

std_msgs::String logdata;
ros::Publisher pub_log("LOGDATA", &logdata);

// ** ROS callback & subscriber **

void motor_command_cb( const std_msgs::Float64MultiArray& msg)
{
//  char result[8]; // Buffer big enough for 7-character float
//  char result1[8];
//  if(msg.data[0] == 1)
//  {
//    //dtostrf(msg.data[0], 6, 2, result); // Leave room for too large numbers!
//    dtostrf(msg.data[1], 6, 2, result1);
//    //nh.loginfo(result);
//    nh.loginfo(result1);
//  }
  SetAngles(msg.data[0], msg.data[1]);
}

ros::Subscriber<std_msgs::Float64MultiArray> rosSub0("/simple_hexapod/Th1_1_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub1("/simple_hexapod/Th2_1_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub2("/simple_hexapod/Th3_1_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub3("/simple_hexapod/Th1_2_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub4("/simple_hexapod/Th2_2_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub5("/simple_hexapod/Th3_2_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub6("/simple_hexapod/Th1_3_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub7("/simple_hexapod/Th2_3_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub8("/simple_hexapod/Th3_3_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub9("/simple_hexapod/Th1_4_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub10("/simple_hexapod/Th2_4_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub11("/simple_hexapod/Th3_4_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub12("/simple_hexapod/Th1_5_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub13("/simple_hexapod/Th2_5_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub14("/simple_hexapod/Th3_5_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub15("/simple_hexapod/Th1_6_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub16("/simple_hexapod/Th2_6_position_controller/command", motor_command_cb);
ros::Subscriber<std_msgs::Float64MultiArray> rosSub17("/simple_hexapod/Th3_6_position_controller/command", motor_command_cb);


// ** Setup **
void setup() {

  // Dynamixel setup
  DXL_SERIAL.begin(57600);
  DXL_SERIAL.transmitterEnable(DEPin);

  // ROS setup
  nh.getHardware()->setBaud(115200);      // set baud rate to 115200
  nh.initNode();              // init ROS
  nh.subscribe(rosSub0);
  nh.subscribe(rosSub1);
  nh.subscribe(rosSub2);
  nh.subscribe(rosSub3);
  nh.subscribe(rosSub4);
  nh.subscribe(rosSub5);
  nh.subscribe(rosSub6);
  nh.subscribe(rosSub7);
  nh.subscribe(rosSub8);
  nh.subscribe(rosSub9);
  nh.subscribe(rosSub10);
  nh.subscribe(rosSub11);
  nh.subscribe(rosSub12);
  nh.subscribe(rosSub13);
  nh.subscribe(rosSub14);
  nh.subscribe(rosSub15);
  nh.subscribe(rosSub16);
  nh.subscribe(rosSub17);
  nh.advertise(pub_m_feed0);
  nh.advertise(pub_m_feed1);
  nh.advertise(pub_m_feed2);
  nh.advertise(pub_log);
  //broadcaster.init(nh);       // set up broadcaster fot tf
  //nh.advertise(pub_eye);   // advertise eye topic
}

MyDynamixel dxl(&DXL_SERIAL);

float Angle[18];
long currenttime = millis();
long prevtime = millis();
char dataStr[100] = "";
char buff[7];

// ** Main loop **
void loop() 
{
  nh.spinOnce();

//  motor_feedback[0].data = dxl.PresentPos(0); 
//  motor_feedback[1].data = dxl.PresentPos(1);
//  motor_feedback[2].data = dxl.PresentPos(2);
//  pub_m_feed0.publish(&motor_feedback[0]);
//  pub_m_feed1.publish(&motor_feedback[1]);
//  pub_m_feed2.publish(&motor_feedback[2]);
  //delay(3);
  
  //Serial.println(motor_feedback[1].data);Serial.println(currenttime-prevtime);
  //char result[8]; // Buffer big enough for 7-character float
  //char result1[8];
  //dtostrf(motor_feedback[1].data, 6, 2, result); // Leave room for too large numbers!
  //dtostrf(currenttime-prevtime, 6, 2, result1);
  //nh.loginfo(result);
  //nh.loginfo(result1);
}


// ** Functions **
void SetAngles(int num, float angle)
{
  if (num == 0)
  {
    Angle[num] = angle*180/M_PI+150;
    dxl.MoveServos(0,Angle[num],0);   
  }
  else if (num == 1)
  {
    prevtime = currenttime;
    currenttime = millis(); 
    Angle[num] = angle*180/M_PI+150;

    dataStr[0] = 0; 
    itoa( millis(),buff,10); //convert long to charStr
    strcat(dataStr, buff); //add it to the end
    strcat(dataStr, ", "); //append the delimiter
    dtostrf(angle, 5, 5, buff);
    strcat(dataStr, buff);
    //strcat(dataStr, 0);
    
    dxl.MoveServos(1,Angle[num],0);

    logdata.data = dataStr;
    pub_log.publish(&logdata);
  }
  else if (num == 2)
  {
    Angle[num] = angle*180/M_PI+150;
    dxl.MoveServos(2,Angle[num],0);
  }
  else if (num == 3)
  {
    Angle[num] = angle*180/M_PI+150;
    //dxl.MoveServos(1,Angle[num],0);
  }
  else if (num == 4)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 5)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 6)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 7)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 8)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 9)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 10)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 11)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 12)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 13)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 14)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 15)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 16)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
  else if (num == 17)
  {
    Angle[num] = angle;
    //dxl.setGoalPosition(10, Angle[num], UNIT_DEGREE);
  }
}
