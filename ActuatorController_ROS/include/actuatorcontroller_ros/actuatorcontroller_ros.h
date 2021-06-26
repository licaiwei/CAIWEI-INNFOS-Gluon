#ifndef ACTUATORCONTROLLER_ROS_H
#define ACTUATORCONTROLLER_ROS_H


#include "ros/ros.h"

// Actuator SDK
#include "actuatorcontroller.h"
#include "actuatordefine.h"

// standard message type
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>

// custom message type
#include "actuatorcontroller_ros/ActuatorAttribute.h"
#include "actuatorcontroller_ros/ActuatorCommand.h"
#include "actuatorcontroller_ros/ActuatorModes.h"
#include "actuatorcontroller_ros/ActuatorArray.h"

// custom service type
#include "actuatorcontroller_ros/AttributeLookup.h"
#include "actuatorcontroller_ros/AttributeQuery.h"
#include "actuatorcontroller_ros/GeneralQuery.h"
#include "actuatorcontroller_ros/AttributeDictionary.h"
#include "actuatorcontroller_ros/DebugQuery.h"
#include "actuatorcontroller_ros/TriviaQuery.h"
#include "actuatorcontroller_ros/IDModify.h"
#include "actuatorcontroller_ros/ParametersSave.h"
#include "actuatorcontroller_ros/ZeroReset.h"

// std stuff
#include <map>
#include <string>


class ActuatorController_ROS {

public:


    ActuatorController_ROS();

    ~ActuatorController_ROS();


    void releaseJointStates();

    void updateROSParam();

    void updateROSParam(uint8_t joint_id);

    // timer callback
    void updateInformation(const ros::TimerEvent &  time_eve);




private:




    //Subscriber Callback
    void subscribeChangeAttribute(const actuatorcontroller_ros::ActuatorAttribute & msg);

    void subscribeEnableActuator(const actuatorcontroller_ros::ActuatorArray & msg);

    void subscribeDisableActuator(const actuatorcontroller_ros::ActuatorArray & msg);

    void subscribeSetTargetPosition(const actuatorcontroller_ros::ActuatorCommand & msg);

    void subscribeSetTargetVelocity(const actuatorcontroller_ros::ActuatorCommand & msg);

    void subscribeSetTargetCurrent(const actuatorcontroller_ros::ActuatorCommand & msg);

    void subscribeSetControlMode(const actuatorcontroller_ros::ActuatorModes & msg);

    void subscribeJointState(const sensor_msgs::JointState & msg);


    // service callback
    bool serviceAttributeQuery(actuatorcontroller_ros::AttributeQueryRequest & req,
                               actuatorcontroller_ros::AttributeQueryResponse & res );

    bool serviceGeneralQuery(actuatorcontroller_ros::GeneralQueryRequest & req,
                             actuatorcontroller_ros::GeneralQueryResponse & res );


    bool serviceAttributeDictionary(actuatorcontroller_ros::AttributeDictionaryRequest & req,
                                actuatorcontroller_ros::AttributeDictionaryResponse & res );


    bool serviceDebugQuery(actuatorcontroller_ros::DebugQueryRequest & req,
                                                   actuatorcontroller_ros::DebugQueryResponse & res );

    bool serviceTriviaQuery(actuatorcontroller_ros::TriviaQueryRequest & req,
                           actuatorcontroller_ros::TriviaQueryResponse & res );

    bool serviceIDModify(actuatorcontroller_ros::IDModifyRequest & req,
            actuatorcontroller_ros::IDModifyResponse & res);

    bool serviceParameterSave(actuatorcontroller_ros::ParametersSaveRequest & req,
                         actuatorcontroller_ros::ParametersSaveResponse & res);

    bool serviceZeroReset(actuatorcontroller_ros::ZeroResetRequest & req,
                          actuatorcontroller_ros::ZeroResetResponse & res);


    // private handles
	ros::NodeHandle nh;
    ActuatorController *m_pController;

    // Upload all the ros parameters to the server;
    void initializeROSParam();

    // Upload one ROSParam to the server;
    void initializeROSParam(uint8_t joint_id);

    // clear all the ros parameters in the server
    void clearROSParam();

    // clear the ros parameters of one actuator
    void clearROSParam(uint8_t joint_id);

    void populateAttributeMap();

    void populateDictionary();


    // publisher for current actuator information
	ros::Publisher m_pubJointState;

	ros::Subscriber m_subAttributeChange;
	ros::Subscriber m_subEnableActuator;
    ros::Subscriber m_subDisableActuator;
	ros::Subscriber m_subTargetPosition;
	ros::Subscriber m_subTargetVelocity;
	ros::Subscriber m_subTargetCurrent;
	ros::Subscriber m_subControlMode;
	ros::Subscriber m_subJointState;

	
	ros::ServiceServer m_serAttributeQuery;
	ros::ServiceServer m_serGeneralQuery;
	ros::ServiceServer m_serAttributeLookup;
    ros::ServiceServer m_serAttributeDictionary;
    ros::ServiceServer m_serTriviaQuery;
    ros::ServiceServer m_serDebugQuery;
    ros::ServiceServer m_serIDModify;
    ros::ServiceServer m_serIDChange;
    ros::ServiceServer m_serParametersSave;
    ros::ServiceServer m_mZeroReset;


	// temp message


	// param variable
	std::string m_sINNFOS;
    std::string m_sActuator;
    std::map<std::string , ActuatorAttribute > m_mChangableDoubleAttribute;
    std::map<std::string , ActuatorAttribute > m_mChangableIntAttribute;
    std::map<std::string , std::string > m_mAttributeDictionary;
    std::map<std::string , std::string > m_mAttributeType;
    std::map<std::string , bool > m_mAttributeChangeable;
    std::map<int, ActuatorMode > m_mActuatorMode;
    std::map<ActuatorMode, int > m_mActuatorModeReverse;


    // internal variables
    bool use_cvp;
    bool no_param;


}; //class

#endif 

