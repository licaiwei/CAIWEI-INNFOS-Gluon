#include "actuatorcontroller_ros.h"
#include <iostream>


ActuatorController_ROS::ActuatorController_ROS(){

    m_pController = ActuatorController::getInstance();

    m_sINNFOS = "/INNFOS";
    m_sActuator = "/Actuator/";


    m_pubJointState = nh.advertise<sensor_msgs::JointState>("/INNFOS/actuator_states", 10);

//    m_subAttributeChange = nh.subscribe("/INNFOS/changeAttribute", 100,
//                                              &ActuatorController_ROS::subscribeChangeAttribute, this);


    m_subEnableActuator = nh.subscribe("/INNFOS/enableActuator", 100,
                                              &ActuatorController_ROS::subscribeEnableActuator, this);

    m_subDisableActuator = nh.subscribe("/INNFOS/disableActuator", 100,
                                        &ActuatorController_ROS::subscribeDisableActuator, this);

    m_subTargetPosition = nh.subscribe("/INNFOS/setTargetPosition", 100,
                                              &ActuatorController_ROS::subscribeSetTargetPosition, this);


    m_subTargetVelocity = nh.subscribe("/INNFOS/setTargetVelocity", 100,
                                              &ActuatorController_ROS::subscribeSetTargetVelocity, this);

    m_subTargetCurrent = nh.subscribe("/INNFOS/setTargetCurrent", 100,
                                       &ActuatorController_ROS::subscribeSetTargetCurrent, this);

    m_subControlMode = nh.subscribe("/INNFOS/setControlMode", 100,
                                      &ActuatorController_ROS::subscribeSetControlMode, this);

    m_subJointState = nh.subscribe("/INNFOS/actuator_targets" , 100,
            &ActuatorController_ROS::subscribeJointState, this);


    m_serAttributeQuery = nh.advertiseService("/INNFOS/AttributeQuery", &ActuatorController_ROS::serviceAttributeQuery, this);
    m_serGeneralQuery  = nh.advertiseService("/INNFOS/GeneralQuery", &ActuatorController_ROS::serviceGeneralQuery, this);
//    m_serAttributeLookup = nh.advertiseService("/INNFOS/AttributeLookup", &ActuatorController_ROS::serviceAttributeLookup, this);

    m_serTriviaQuery  = nh.advertiseService("/INNFOS/TriviaQuery", &ActuatorController_ROS::serviceTriviaQuery, this);
    m_serDebugQuery  = nh.advertiseService("/INNFOS/DebugQuery", &ActuatorController_ROS::serviceDebugQuery, this);
    m_serAttributeDictionary = nh.advertiseService("/INNFOS/Dictionary", &ActuatorController_ROS::serviceAttributeDictionary, this);
    m_serIDChange = nh.advertiseService("/INNFOS/IDChange", &ActuatorController_ROS::serviceIDModify,this);
    m_serParametersSave = nh.advertiseService("/INNFOS/ParametersSave", &ActuatorController_ROS::serviceParameterSave, this);
    m_mZeroReset = nh.advertiseService("/INNFOS/ZeroReset", &ActuatorController_ROS::serviceZeroReset, this);

    // populate some maps and servers
    populateAttributeMap();
    populateDictionary();
    m_mActuatorMode[1] = ActuatorMode::Mode_Cur;
    m_mActuatorMode[2] = ActuatorMode::Mode_Pos;
    m_mActuatorMode[3] = ActuatorMode::Mode_Vel;
    m_mActuatorMode[4] = ActuatorMode::Mode_Profile_Pos;
    m_mActuatorMode[5] = ActuatorMode::Mode_Profile_Vel;
    m_mActuatorMode[6] = ActuatorMode::Mode_Homing;

    m_mActuatorModeReverse[ActuatorMode::Mode_Cur] = 1;
    m_mActuatorModeReverse[ActuatorMode::Mode_Pos] = 2;
    m_mActuatorModeReverse[ActuatorMode::Mode_Vel] = 3;
    m_mActuatorModeReverse[ActuatorMode::Mode_Profile_Pos] = 4;
    m_mActuatorModeReverse[ActuatorMode::Mode_Profile_Vel] = 5;
    m_mActuatorModeReverse[ActuatorMode::Mode_Homing] = 6;



    use_cvp = false ;
    no_param = false;

    bool temp_bool;
    if (ros::param::get("/innfos_actuator/innfos_use_cvp", temp_bool)){
        use_cvp = temp_bool;
    }
    if (ros::param::get("/innfos_actuator/innfos_no_param" , temp_bool)){
        no_param = temp_bool;
    }

    initializeROSParam();



}


ActuatorController_ROS::~ActuatorController_ROS() {
    // unregister all publishers here


    m_pubJointState.shutdown();


}

void ActuatorController_ROS::updateInformation(const ros::TimerEvent &  time_eve){
    releaseJointStates();
    updateROSParam();
}



void ActuatorController_ROS::releaseJointStates(){

    std::vector<uint8_t> temp_vec = m_pController->getActuatorIdArray();


    std_msgs::Header temp_header;
    temp_header.stamp = ros::Time::now();
    sensor_msgs::JointState temp_jointstate;
    temp_jointstate.header = temp_header;


    for (uint8_t joint_id : temp_vec){

        if ((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized) {

            temp_jointstate.name.push_back("Actuator" + std::to_string(int(joint_id)));

            if (use_cvp){
                m_pController->requestCVPValue(joint_id);
                temp_jointstate.position.push_back(m_pController->getPosition( joint_id,false));
                temp_jointstate.velocity.push_back(m_pController->getVelocity( joint_id,false));
                temp_jointstate.effort.push_back(m_pController->getCurrent( joint_id,false));
            }else {
                temp_jointstate.position.push_back(m_pController->getPosition( joint_id,true));
                temp_jointstate.velocity.push_back(m_pController->getVelocity( joint_id,true));
                temp_jointstate.effort.push_back(m_pController->getCurrent( joint_id,true));
            }

        }
    }

    if (temp_vec.size())
        m_pubJointState.publish(temp_jointstate);

}

//Subscriber Callback
void ActuatorController_ROS::subscribeChangeAttribute(const actuatorcontroller_ros::ActuatorAttribute & msg){
    m_pController->setActuatorAttribute(uint8_t(msg.JointID) , ActuatorAttribute(msg.AttributeID) ,  msg.Value);
}

void ActuatorController_ROS::subscribeEnableActuator(const actuatorcontroller_ros::ActuatorArray & msg){

        std::vector<int> temp_vec;

        int num_of_actuator= msg.JointIDs.size();

        if (num_of_actuator) {
            for (int j = 0 ; j < num_of_actuator ; j ++) {

                if (msg.JointIDs[j] == 0){
                    ROS_INFO("Turning on all actuators");
                    m_pController->enableAllActuators();
                    initializeROSParam();
                    return;
                }
                m_pController->enableActuator(uint8_t(msg.JointIDs[j]));
                initializeROSParam(uint8_t(msg.JointIDs[j]));
            }
        }else {
            ROS_INFO("Turning on all actuators");
            m_pController->enableAllActuators();
            initializeROSParam();
        }

}


void ActuatorController_ROS::subscribeDisableActuator(const actuatorcontroller_ros::ActuatorArray & msg){

    std::vector<int> temp_vec;

    int num_of_actuator= msg.JointIDs.size();

    if (num_of_actuator) {
        for (int j = 0 ; j < num_of_actuator ; j ++) {
            if (msg.JointIDs[j] == 0){
                ROS_INFO("Turning off all actuators");
                m_pController->disableAllActuators();
                clearROSParam();
                return;
            }
            m_pController->disableActuator(uint8_t(msg.JointIDs[j]));
            clearROSParam(msg.JointIDs[j]);
        }
    }else {
        ROS_INFO("Turning off all actuators");
        m_pController->disableAllActuators();
        clearROSParam();
    }

}

void ActuatorController_ROS::subscribeSetTargetPosition(const actuatorcontroller_ros::ActuatorCommand & msg){
    m_pController->setPosition(uint8_t(msg.JointID) , msg.TargetValue);
}

void ActuatorController_ROS::subscribeSetTargetVelocity(const actuatorcontroller_ros::ActuatorCommand & msg){
    m_pController->setVelocity(uint8_t(msg.JointID) , msg.TargetValue);
}

void ActuatorController_ROS::subscribeSetTargetCurrent(const actuatorcontroller_ros::ActuatorCommand & msg){
    m_pController->setCurrent(uint8_t(msg.JointID) , msg.TargetValue);
}



void ActuatorController_ROS::subscribeJointState(const sensor_msgs::JointState & msg){

    int num_of_joint = msg.name.size();

    if (msg.name.size() != msg.effort.size() ||
    msg.name.size() != msg.velocity.size() ||
    msg.name.size() != msg.position.size()){
        ROS_INFO("JOINT STATES COMMANDS DO NOT MATCH");
        return;
    }

    for (uint8_t name_id = 0 ; name_id < num_of_joint ; name_id++){

        uint8_t joint_id = stoi(msg.name[name_id].c_str());

//        ROS_INFO("ID: [%d]", int(joint_id));

        // process positions
        if (m_pController->getActuatorMode(joint_id) == Actuator::Mode_Profile_Pos || m_pController->getActuatorMode(joint_id) == Actuator::Mode_Pos){
            // It is in position mode
            double cmd = msg.position[name_id];

            if ( isnan(cmd) ){
                // something is wrong, received nan command
            } else {
                m_pController->setPosition(joint_id, cmd);
            }
        }


        // process velocities
        if (m_pController->getActuatorMode(joint_id) == Actuator::Mode_Profile_Vel || m_pController->getActuatorMode(joint_id) == Actuator::Mode_Vel){
            // It is in velocity mode
            double cmd = msg.velocity[name_id];

            if ( isnan(cmd)){
                // something is wrong, received nan command
            } else {
                m_pController->setVelocity(joint_id, cmd);
            }
        }


        // process currents
        if (m_pController->getActuatorMode(joint_id) == Actuator::Mode_Cur ){
            // It is in current mode
            double cmd = msg.effort[name_id];

            if ( isnan(cmd) ){
                // something is wrong, received nan command
            } else {
                m_pController->setCurrent(joint_id, cmd);
            }
        }


    }

}
void ActuatorController_ROS::subscribeSetControlMode(const actuatorcontroller_ros::ActuatorModes & msg){

    std::vector<uint8_t> temp_vec;

    int num_of_actuator= msg.JointIDs.size();

    if (num_of_actuator) {
        for (int j = 0 ; j < num_of_actuator ; j ++) {
            temp_vec.push_back(msg.JointIDs[j]);
        }

    }else {
        temp_vec = m_pController->getActuatorIdArray();
    }


    if (m_mActuatorMode.find(msg.ActuatorMode) != m_mActuatorMode.end()){
        m_pController->activateActuatorModeInBantch(temp_vec , m_mActuatorMode[msg.ActuatorMode]);
    }else {
        ROS_INFO("Wrong Actuator Modes Input!");
    }

}
