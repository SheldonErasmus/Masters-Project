#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include "std_msgs/String.h"

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

namespace gazebo
{
  class Control : public ModelPlugin
  {
    // brief A node use for ROS transport
    // brief Constructor
    public: Control() {}

    double Angle[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int flag = 1; // saam met update()

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        if (_parent->GetJointCount() == 0)
        {
          std::cerr << "Invalid joint count, My_Controller plugin not loaded\n";
          return;
        }

        // Get number of joints
        //this->numJoints = _parent->GetJointCount();

        std::cout << "Started Control\n";

        // Store the pointer to the model
        this->model = _parent;

        // Joint PID values
        this->JointPID[0] = common::PID(3, 0, 0.);
        this->JointPID[1] = common::PID(3, 0, 0.);
        this->JointPID[2] = common::PID(3, 0, 0.);

        // Store the pointer to the joints
        for (int i=0;i<18;i++)
        {
          this->joint[i] = this->model->GetJoints()[i];
          std::cout << this->joint[i]->GetScopedName();
          std::cout << "\n";
          //this->model->GetJointController()->SetPositionPID(this->joint[i]->GetScopedName(), common::PID(3, 0.00, 0.0));
        }

        // Default to zero degree angle
        //double angle = 0;

        //this->SetStartAngles(-1.5);
        
        // Initialize ros, if it has not already been initialized.
        if (!ros::isInitialized())
        {
          int argc = 0;
          char **argv = NULL;
          ros::init(argc, argv, "My_Controller",ros::init_options::NoSigintHandler);
        }
        
        // Create the node
        this->rosNode.reset(new ros::NodeHandle("My_Controller"));
        
        int count = 0;
        // Subscribe to the topic, and register a callback
        for (int i=1;i<7;i++)
        {
          for (int j=1;j<4;j++)
          {
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64MultiArray>("/" + this->model->GetName() + "/Th" + std::to_string(j) + "_" + std::to_string(i) + "_position_controller/command",1,boost::bind(&Control::OnRosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);
            this->rosSub[count] = this->rosNode->subscribe(so);
            count++;
          } 
        }

        // Spin up the queue helper thread.
        this->rosQueueThread = std::thread(std::bind(&Control::QueueThread, this));

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Control::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      /* if(flag){
        for (int num=0;num<18;num++)
        {
          
          this->model->GetJointController()->SetJointPosition(this->joint[num]->GetScopedName(),Angle[num]);
          //ROS_INFO_STREAM(this->joint[num]->GetScopedName());
          this->model->GetJointController()->SetPositionTarget(this->joint[num]->GetScopedName(),Angle[num]);
        }
        flag = 0;
      }
      this->model->GetJointController()->Update(); //nie nodig */

      // compute the steptime for the PID
      common::Time currTime = this->model->GetWorld()->SimTime();
      common::Time stepTime = currTime - this->prevUpdateTime;
      this->prevUpdateTime = currTime;

      for(int num=0;num<18;num++)
      {
        // set the current position of the joint, and the target position, 
        // and the maximum effort limit
        double pos_target = Angle[num];
        double pos_curr = this->joint[num]->Position(0);
 
        // calculate the error between the current position and the target one
        double pos_err = pos_curr - pos_target;

        // compute the effort via the PID, which you will apply on the joint
        double effort_cmd = this->JointPID[num%3].Update(pos_err, stepTime);

        // apply the force on the joint
        this->joint[num]->SetForce(0, effort_cmd);
      }
    }

    // Set the start angle of the joints
    public: void SetStartAngles(const double angle)
    {
      this->model->GetJointController()->SetJointPosition(this->joint[2]->GetScopedName(),angle);
      this->model->GetJointController()->SetJointPosition(this->joint[5]->GetScopedName(),angle);
      this->model->GetJointController()->SetJointPosition(this->joint[8]->GetScopedName(),angle);
      this->model->GetJointController()->SetJointPosition(this->joint[11]->GetScopedName(),angle);
      this->model->GetJointController()->SetJointPosition(this->joint[14]->GetScopedName(),angle);
      this->model->GetJointController()->SetJointPosition(this->joint[17]->GetScopedName(),angle);

      this->model->GetJointController()->SetJointPosition(this->joint[1]->GetScopedName(),0);
      this->model->GetJointController()->SetJointPosition(this->joint[4]->GetScopedName(),0);
      this->model->GetJointController()->SetJointPosition(this->joint[7]->GetScopedName(),0);
      this->model->GetJointController()->SetJointPosition(this->joint[10]->GetScopedName(),0);
      this->model->GetJointController()->SetJointPosition(this->joint[13]->GetScopedName(),0);
      this->model->GetJointController()->SetJointPosition(this->joint[16]->GetScopedName(),0);
    }

    // Set the angle of the joints
    public: void SetAngles(const int num, const double angle)
    {
      //this->model->GetJointController()->SetJointPosition(this->joint[num]->GetScopedName(),angle);
      //ROS_INFO_STREAM(this->joint[num]->GetScopedName());
      flag =1;
      if (num == 0) 
      {
        Angle[num] = angle;
      }  
      else if (num == 1) 
      {  
        Angle[num] = angle;
      }
      else if (num == 2) 
      {  
        Angle[num] = angle;
      }
      else if (num == 3) 
      {  
        Angle[num] = angle;
      }
      else if (num == 4) 
      {  
        Angle[num] = angle;
      }
      else if (num == 5) 
      {  
        Angle[num] = angle;
      }
      else if (num == 6) 
      {  
        Angle[num] = angle;
      }
      else if (num == 7) 
      {  
        Angle[num] = angle;
      }
      else if (num == 8) 
      {  
        Angle[num] = angle;
      }
      else if (num == 9) 
      {  
        Angle[num] = angle;
      }
      else if (num == 10) 
      {  
        Angle[num] = angle;
      }
      else if (num == 11) 
      {  
        Angle[num] = angle;
      }
      else if (num == 12) 
      {  
        Angle[num] = angle;
      }
      else if (num == 13) 
      {  
        Angle[num] = angle;
      }
      else if (num == 14) 
      {  
        Angle[num] = angle;
      }
      else if (num == 15) 
      {  
        Angle[num] = angle;
      }
      else if (num == 16) 
      {  
        Angle[num] = angle;
      }
      else if (num == 17) 
      {  
        Angle[num] = angle;
      }
    }

    // Handle incoming message
    public: void OnRosMsg(const std_msgs::Float64MultiArrayConstPtr &_msg)
    {
      SetAngles(_msg->data[0],_msg->data[1]);
    }

    // ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the joints.
    private: physics::JointPtr joint[18];

    // A node used for transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    // A subscriber to a named topic.
    private: ros::Subscriber rosSub[18];

    // A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    // A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: common::PID JointPID[3];

    private: common::Time prevUpdateTime;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Control)
}
