#include "actuatorcontroller_ros.h"
#include "actuatorcontroller.h"
#include <ros/package.h>

ActuatorController_ROS * INNFOS_ptr;


void actuator_event_callback(const ros::TimerEvent& event)
{
  ActuatorController::getInstance()->processEvents();
}




int main(int argc, char **argv) {

  ros::init(argc, argv, "innfos_actuator");

  ros::NodeHandle nh;
  ROS_INFO("Started Actuator ROS Node Ver. 3.1.0");

  ActuatorController::initController();

  Actuator::ErrorsDefine _errorCode;

  ActuatorController::getInstance()->lookupActuators(_errorCode);


  INNFOS_ptr = new ActuatorController_ROS();


  ros::Timer timer1 = nh.createTimer(ros::Duration(0.001), actuator_event_callback);

  ros::Timer timer2;

  int control_rate;
  // if a control rate is given, we will use it
  if (ros::param::get("/innfos_actuator/innfos_fixed_rate", control_rate)){
      double interval = 1.0/double(control_rate);
      timer2 = nh.createTimer(ros::Duration(interval), &ActuatorController_ROS::updateInformation , INNFOS_ptr);
  }
  else {

      while (ros::ok())
      {
          INNFOS_ptr->releaseJointStates();
          INNFOS_ptr->updateROSParam();

          ros::spinOnce();

          ros::Duration(0.005).sleep();
      }


  }

  ros::spin();

  return 0;
}