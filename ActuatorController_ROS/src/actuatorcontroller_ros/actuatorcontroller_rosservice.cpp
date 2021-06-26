#include "actuatorcontroller_ros.h"
#include <iostream>




bool ActuatorController_ROS::serviceAttributeQuery(actuatorcontroller_ros::AttributeQueryRequest & req,
                                                   actuatorcontroller_ros::AttributeQueryResponse & res ){

    uint8_t joint_id = uint8_t(req.ActuatorID);

    res.ACTUAL_CURRENT = m_pController->getCurrent(joint_id,false);
    res.ACTUAL_VELOCITY = m_pController->getVelocity(joint_id,false);
    res.ACTUAL_POSITION = m_pController->getPosition(joint_id,false);
    res.MODE_ID         = m_mActuatorModeReverse[m_pController->getActuatorMode(joint_id)];
    res.ACTUATOR_SWITCH = m_pController->isEnable(joint_id);
    res.ONLINE_STATUS = m_pController->isOnline(joint_id);
    res.INIT_STATE = ((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized);

    return true;
}


bool ActuatorController_ROS::serviceTriviaQuery(actuatorcontroller_ros::TriviaQueryRequest & req,
                        actuatorcontroller_ros::TriviaQueryResponse & res ){

    uint8_t joint_id = uint8_t(req.ActuatorID);

    res.VOLTAGE = m_pController->getVoltage(joint_id,true);
    res.CURRENT_SCALE = m_pController->getCurrentRange(joint_id,false);
    res.VELOCITY_SCALE = m_pController->getVelocityRange(joint_id,false);
    res.ACTUATOR_TEMPERATURE = m_pController->getMotorTemperature(joint_id,true);
    res.INVERTER_TEMPERATURE = m_pController->getInverterTemperature(joint_id,true);

    return true;
}


bool ActuatorController_ROS::serviceDebugQuery(actuatorcontroller_ros::DebugQueryRequest & req,
                                                   actuatorcontroller_ros::DebugQueryResponse & res ){

    uint8_t joint_id = uint8_t(req.ActuatorID);

    res.FIRMWARE_VERSION =  m_pController->getActuatorAttribute(joint_id, Actuator::FIRMWARE_VERSION);
    res.SN_ID =  m_pController->getActuatorAttribute(joint_id, Actuator::SN_ID);
    res.ERROR_ID = m_pController->getActuatorAttribute(joint_id, Actuator::ERROR_ID);
    res.VERSION_430 = m_pController->getActuatorAttribute(joint_id, Actuator::VERSION_430);
//    res.FRENQUENCY_430 = m_pController->getActuatorAttribute(joint_id, Actuator::FRENQUENCY_430);

    return true;
}



bool ActuatorController_ROS::serviceGeneralQuery(actuatorcontroller_ros::GeneralQueryRequest & req,
                                                 actuatorcontroller_ros::GeneralQueryResponse & res ){


    std::vector<uint8_t> temp_vec = m_pController->getActuatorIdArray();

    for (uint8_t joint_id : temp_vec){
        res.ActuatorList.push_back(joint_id);
        res.ActuatorSwitch.push_back((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized);
    }


    return true;

}




bool ActuatorController_ROS::serviceAttributeDictionary(actuatorcontroller_ros::AttributeDictionaryRequest & req,
                                actuatorcontroller_ros::AttributeDictionaryResponse & res ){

    if (m_mAttributeDictionary.find(req.LookupTerm.data)!= m_mAttributeDictionary.end()){
        res.isChangeable = m_mAttributeChangeable[req.LookupTerm.data];
        res.TermType.data = m_mAttributeType[req.LookupTerm.data];
        res.TermExplanation.data = m_mAttributeDictionary[req.LookupTerm.data];
    }else {
        res.TermExplanation.data = "Invalid term!";
    }

    return true;
}



bool ActuatorController_ROS::serviceIDModify(actuatorcontroller_ros::IDModifyRequest & req,
                     actuatorcontroller_ros::IDModifyResponse & res){

    uint8_t og_id = uint8_t(req.OriginalJointID);
    uint8_t mf_id = uint8_t(req.ModifiedJointID);

    res.isSuccessful = false;

    if (m_pController->isEnable(og_id)){

//        std::vector<uint8_t> temp_vec = m_pController->getActuatorIdArray();
//
//        for (uint8_t temp_id : temp_vec){
//            if (temp_id == mf_id){
//                res.isSuccessful = false;
//                return true;
//            }
//        }

        if (m_pController->setActuatorID(og_id, mf_id)){
            clearROSParam(og_id);
            initializeROSParam(mf_id);
            res.isSuccessful = true;
        }else{

        }
    }
    return true;
}



bool ActuatorController_ROS::serviceParameterSave(actuatorcontroller_ros::ParametersSaveRequest & req,
                          actuatorcontroller_ros::ParametersSaveResponse & res){
    res.isSuccessful = false;

    if (m_pController->saveAllParams(uint8_t(req.ActuatorID))){
        res.isSuccessful = true;
    }

    return true;
}


bool ActuatorController_ROS::serviceZeroReset(actuatorcontroller_ros::ZeroResetRequest & req,
                      actuatorcontroller_ros::ZeroResetResponse & res){

    uint8_t joint_id = uint8_t(req.JointID);

    double temp_double ;
    res.isSuccessful = false;

    if (m_pController->isEnable(joint_id)){
        m_pController->activateActuatorMode(joint_id,ActuatorMode::Mode_Homing);
        m_pController->setHomingPosition(joint_id , m_pController->getPosition(joint_id,true));
        temp_double = m_pController->getMaximumPosition(joint_id,true);
        temp_double = m_pController->getMinimumPosition(joint_id,true);
        res.isSuccessful = true;
    }

    return true;

}

void ActuatorController_ROS::populateDictionary(){

    m_mAttributeDictionary["CUR_PROPORTIONAL"] = "Description: Proportional value for current mode's "
                                                 "PID control";
    m_mAttributeDictionary["CUR_INTEGRAL"] = "Integral value for current mode's PID control";
    m_mAttributeDictionary["CUR_MAXSPEED"] = "Maximum speed allowed for current mode";
    m_mAttributeDictionary["VEL_PROPORTIONAL"] = "Proportional value for velocity mode's PID control";
    m_mAttributeDictionary["VEL_INTEGRAL"] = "Integral value for velocity mode's PID control";
    m_mAttributeDictionary["VEL_OUTPUT_LIMITATION_MINIMUM"] = "Maximum (in negative direction) ratio of the physical "
                                                              "current limit allowed in velocity mode, valid from "
                                                              "-1.0 to 0.0, if the physical current limit is 33A and "
                                                              "this value is -0.5, then the maximum current "
                                                              "(in negative direction) output in "
                                                              "velocity mode is -16.5A";
    m_mAttributeDictionary["VEL_OUTPUT_LIMITATION_MAXIMUM"] = "Maximum ratio of the physical current limit allowed "
                                                              "in velocity mode, valid from "
                                                              "0.0 to 1.0, if the physical current limit is 33A and "
                                                              "this value is 0.5, then the maximum current output in "
                                                              "velocity mode is 16.5A";
    m_mAttributeDictionary["POS_PROPORTIONAL"] = "Proportional value for position mode's PID control";
    m_mAttributeDictionary["POS_INTEGRAL"] = "Integral value for position mode's PID control";
    m_mAttributeDictionary["POS_DIFFERENTIAL"] = "Differential value for position mode's PID control";
    m_mAttributeDictionary["POS_OUTPUT_LIMITATION_MINIMUM"] = "Maximum (in negative direction) ratio of the physical "
                                                              "current limit allowed in position mode, valid from "
                                                              "-1.0 to 0.0, if the physical current limit is 33A and "
                                                              "this value is -0.5, then the maximum current "
                                                              "(in negative direction) output in "
                                                              "position mode is -16.5A";
    m_mAttributeDictionary["POS_OUTPUT_LIMITATION_MAXIMUM"] = "Maximum ratio of the physical current limit allowed "
                                                              "in position mode, valid from "
                                                              "0.0 to 1.0, if the physical current limit is 33A and "
                                                              "this value is 0.5, then the maximum current output in "
                                                              "position mode is 16.5A";
    m_mAttributeDictionary["POS_LIMITATION_MINIMUM"] = "Lower limit for position mode, any target value lower than "
                                                       "this limit will be replace by this value";
    m_mAttributeDictionary["POS_LIMITATION_MAXIMUM"] = "Upper limit for position mode, any target value higher than "
                                                       "this limit will be replace by this value";
    m_mAttributeDictionary["PROFILE_POS_MAX_SPEED"] = "The maximum speed allowed in the local planner for profile "
                                                      "position mode";
    m_mAttributeDictionary["PROFILE_POS_ACC"] = "The acceleration in the local planner for profile position mode, "
                                                "used when actuator is approaching maximum speed";
    m_mAttributeDictionary["PROFILE_POS_DEC"] = "The deceleration in the local planner for profile position mode, "
                                                "used when actuator is coming to a stop ";
    m_mAttributeDictionary["PROFILE_VEL_MAX_SPEED"] = "The maximum speed allowed in the local planner for profile "
                                                      "velocity mode";
    m_mAttributeDictionary["PROFILE_VEL_ACC"] = "The acceleration in the local planner for profile velocity mode, "
                                                "used when actuator is approaching maximum speed";
    m_mAttributeDictionary["PROFILE_VEL_DEC"] = "The deceleration in the local planner for profile velocity mode, "
                                                "used when actuator is coming to a stop ";
    m_mAttributeDictionary["FILTER_C_VALUE"] = "The filter cut-off value of the target currents";
    m_mAttributeDictionary["FILTER_V_VALUE"] = "The filter cut-off value of the target velocities";
    m_mAttributeDictionary["FILTER_P_VALUE"] = "The filter cut-off value of the target positions";
    m_mAttributeDictionary["LOCK_ENERGY"] = "The maximum amount of energy allowed in an deadlock situation, "
                                            "if the actuator had spent this much energy but cannot progress, "
                                            "the target output will be gradually lowered";
    m_mAttributeDictionary["ACTUATOR_PROTECT_TEMPERATURE"] = "The maximum actuator temperature allowed during operation,"
                                                             " should the actuator reach this temperature, "
                                                             "it will enter a lock down mode and reject most commands";
    m_mAttributeDictionary["ACTUATOR_RECOVERY_TEMPERATURE"] = "The recovery temperature of the actuator, if the "
                                                              "actuator is in lock down mode and is lowered to this "
                                                              "temperature, it will resume its functions";
    m_mAttributeDictionary["INVERTER_PROTECT_TEMPERATURE"] = "The maximum inverter temperature allowed during operation,"
                                                             " should the actuator reach this temperature, "
                                                             "it will enter a lock down mode and reject most commands";
    m_mAttributeDictionary["INVERTER_RECOVERY_TEMPERATURE"] = "The recovery temperature of the inverter, if the "
                                                              "actuator is in lock down mode and is lowered to this "
                                                              "temperature, it will resume its functions";
    m_mAttributeDictionary["CURRENT_LIMIT"] = "A software limit on the current outputs in current mode, any"
                                              " target exceeding this value will be replaced by this value";
    m_mAttributeDictionary["VELOCITY_LIMIT"] = "A software limit on the velocity outputs in velocity mode, any"
                                               " target exceeding this value will be replaced by this value";
    m_mAttributeDictionary["POS_OFFSET"] = "The leeway for velocity/current mode to react when then are approaching "
                                           "the position limits, allowing them to reduce output before exceeding the limit";



    m_mAttributeDictionary["DEVICE_ID"] = "The actuator ID, changeable";
    m_mAttributeDictionary["MODE_ID"] = "The control mode of the actuator currently in effect. "
                                        "Options include: Mode_Cur (1), Mode_Vel(2), Mode_Pos(3), Mode_Profile_Pos(4), "
                                        "Mode_Profile_Vel(5), Mode_Homing(6)";


    m_mAttributeDictionary["POS_LIMITATION_SWITCH"] = "Determine if the maximum and minimum position limits will "
                                                      "be in effect in velocity/current mode";
    m_mAttributeDictionary["FILTER_C_STATUS"] = "Determine if a first order low pass filter will be used for the "
                                                "target currents";
    m_mAttributeDictionary["FILTER_V_STATUS"] = "Determine if a first order low pass filter will be used for the "
                                                "target velocities";
    m_mAttributeDictionary["FILTER_P_STATUS"] = "Determine if a first order low pass filter will be used for the "
                                                "target positions";


    m_mAttributeDictionary["ACTUAL_CURRENT"] = "The current current of the actuator";
    m_mAttributeDictionary["ACTUAL_VELOCITY"] = "The current velocity of the actuator";
    m_mAttributeDictionary["ACTUAL_POSITION"] = "The current position of the actuator";
    m_mAttributeDictionary["VOLTAGE"] = "The current voltage of the actuator";
    m_mAttributeDictionary["CURRENT_SCALE"] = "The physical upper limit of output current";
    m_mAttributeDictionary["VELOCITY_SCALE"] = "The physical upper limit of output speed";
    m_mAttributeDictionary["ACTUATOR_TEMPERATURE"] = "The current temperature of the actuator";
    m_mAttributeDictionary["INVERTER_TEMPERATURE"] = "The current temperature of the inverter";
    m_mAttributeDictionary["ACTUATOR_SWITCH"] = "The actuator is enabled";
    m_mAttributeDictionary["ONLINE_STATUS"] = "The actuator can still be discovered by the system";
    m_mAttributeDictionary["INIT_STATE"] = "The actuator is initialized and ready to go";

    m_mAttributeDictionary["SN_ID"] = "The serial number of the actuator";
    m_mAttributeDictionary["ERROR_ID"] = "Errors currently present in the actuator";

    m_mAttributeDictionary["VERSION_430"] = "Version number of 430, for tech support";
    m_mAttributeDictionary["FRENQUENCY_430"] = "Frequency  of 430, for tech support";
    m_mAttributeDictionary["FIRMWARE_VERSION"] = "The actuator's firmware version, newer versions tend to have more "
                                                 "features";



    // Attribute type ---------------------------------------

    m_mAttributeType["CUR_PROPORTIONAL"] = "Double";
    m_mAttributeType["CUR_INTEGRAL"] = "Double";
    m_mAttributeType["CUR_MAXSPEED"] = "Double";
    m_mAttributeType["VEL_PROPORTIONAL"] = "Double";
    m_mAttributeType["VEL_INTEGRAL"] = "Double";
    m_mAttributeType["VEL_OUTPUT_LIMITATION_MINIMUM"] = "Double";
    m_mAttributeType["VEL_OUTPUT_LIMITATION_MAXIMUM"] = "Double";
    m_mAttributeType["POS_PROPORTIONAL"] = "Double";
    m_mAttributeType["POS_INTEGRAL"] = "Double";
    m_mAttributeType["POS_DIFFERENTIAL"] = "Double";
    m_mAttributeType["POS_OUTPUT_LIMITATION_MINIMUM"] = "Double";
    m_mAttributeType["POS_OUTPUT_LIMITATION_MAXIMUM"] = "Double";
    m_mAttributeType["POS_LIMITATION_MINIMUM"] = "Double";
    m_mAttributeType["POS_LIMITATION_MAXIMUM"] = "Double";
    m_mAttributeType["PROFILE_POS_MAX_SPEED"] = "Double";
    m_mAttributeType["PROFILE_POS_ACC"] = "Double";
    m_mAttributeType["PROFILE_POS_DEC"] = "Double";
    m_mAttributeType["PROFILE_VEL_MAX_SPEED"] = "Double";
    m_mAttributeType["PROFILE_VEL_ACC"] = "Double";
    m_mAttributeType["PROFILE_VEL_DEC"] = "Double";
    m_mAttributeType["FILTER_C_VALUE"] = "Double";
    m_mAttributeType["FILTER_V_VALUE"] = "Double";
    m_mAttributeType["FILTER_P_VALUE"] = "Double";
    m_mAttributeType["LOCK_ENERGY"] = "Double";
    m_mAttributeType["ACTUATOR_PROTECT_TEMPERATURE"] = "Double";
    m_mAttributeType["ACTUATOR_RECOVERY_TEMPERATURE"] = "Double";
    m_mAttributeType["INVERTER_PROTECT_TEMPERATURE"] = "Double";
    m_mAttributeType["INVERTER_RECOVERY_TEMPERATURE"] = "Double";
    m_mAttributeType["CURRENT_LIMIT"] = "Double";
    m_mAttributeType["VELOCITY_LIMIT"] = "Double";
    m_mAttributeType["POS_OFFSET"] = "Double";

    m_mAttributeType["DEVICE_ID"] = "Integer";
    m_mAttributeType["MODE_ID"] = "Integer";

    m_mAttributeType["POS_LIMITATION_SWITCH"] = "Boolean";
    m_mAttributeType["FILTER_C_STATUS"] = "Boolean";
    m_mAttributeType["FILTER_V_STATUS"] = "Boolean";
    m_mAttributeType["FILTER_P_STATUS"] = "Boolean";

    m_mAttributeType["ACTUAL_CURRENT"] =  "Double";
    m_mAttributeType["ACTUAL_VELOCITY"] = "Double";
    m_mAttributeType["ACTUAL_POSITION"] = "Double";
    m_mAttributeType["VOLTAGE"] = "Double";
    m_mAttributeType["CURRENT_SCALE"] = "Double";
    m_mAttributeType["VELOCITY_SCALE"] ="Double";
    m_mAttributeType["ACTUATOR_TEMPERATURE"] = "Double";
    m_mAttributeType["INVERTER_TEMPERATURE"] = "Double";
    m_mAttributeType["ACTUATOR_SWITCH"] = "Boolean";
    m_mAttributeType["ONLINE_STATUS"] = "Boolean";
    m_mAttributeType["INIT_STATE"] = "Boolean";

    m_mAttributeType["SN_ID"] = "Integer";
    m_mAttributeType["ERROR_ID"] = "Integer";

    m_mAttributeType["VERSION_430"] = "Integer";
    m_mAttributeType["FRENQUENCY_430"] = "Integer";
    m_mAttributeType["FIRMWARE_VERSION"] = "Integer";


    // Attribute Changeable ---------------------------------------
    m_mAttributeChangeable["CUR_PROPORTIONAL"] = true;
    m_mAttributeChangeable["CUR_INTEGRAL"] = true;
    m_mAttributeChangeable["CUR_MAXSPEED"] = true;
    m_mAttributeChangeable["VEL_PROPORTIONAL"] = true;
    m_mAttributeChangeable["VEL_INTEGRAL"] = true;
    m_mAttributeChangeable["VEL_OUTPUT_LIMITATION_MINIMUM"] = true;
    m_mAttributeChangeable["VEL_OUTPUT_LIMITATION_MAXIMUM"] = true;
    m_mAttributeChangeable["POS_PROPORTIONAL"] = true;
    m_mAttributeChangeable["POS_INTEGRAL"] = true;
    m_mAttributeChangeable["POS_DIFFERENTIAL"] = true;
    m_mAttributeChangeable["POS_OUTPUT_LIMITATION_MINIMUM"] = true;
    m_mAttributeChangeable["POS_OUTPUT_LIMITATION_MAXIMUM"] = true;
    m_mAttributeChangeable["POS_LIMITATION_MINIMUM"] = true;
    m_mAttributeChangeable["POS_LIMITATION_MAXIMUM"] = true;
    m_mAttributeChangeable["PROFILE_POS_MAX_SPEED"] = true;
    m_mAttributeChangeable["PROFILE_POS_ACC"] = true;
    m_mAttributeChangeable["PROFILE_POS_DEC"] = true;
    m_mAttributeChangeable["PROFILE_VEL_MAX_SPEED"] = true;
    m_mAttributeChangeable["PROFILE_VEL_ACC"] = true;
    m_mAttributeChangeable["PROFILE_VEL_DEC"] = true;
    m_mAttributeChangeable["FILTER_C_VALUE"] = true;
    m_mAttributeChangeable["FILTER_V_VALUE"] = true;
    m_mAttributeChangeable["FILTER_P_VALUE"] = true;
    m_mAttributeChangeable["LOCK_ENERGY"] = true;
    m_mAttributeChangeable["ACTUATOR_PROTECT_TEMPERATURE"] = true;
    m_mAttributeChangeable["ACTUATOR_RECOVERY_TEMPERATURE"] = true;
    m_mAttributeChangeable["INVERTER_PROTECT_TEMPERATURE"] = true;
    m_mAttributeChangeable["INVERTER_RECOVERY_TEMPERATURE"] = true;
    m_mAttributeChangeable["CURRENT_LIMIT"] = true;
    m_mAttributeChangeable["VELOCITY_LIMIT"] = true;
    m_mAttributeChangeable["POS_OFFSET"] = true;

    m_mAttributeChangeable["DEVICE_ID"] = true;
    m_mAttributeChangeable["MODE_ID"] = true;

    m_mAttributeChangeable["POS_LIMITATION_SWITCH"] = true;
    m_mAttributeChangeable["FILTER_C_STATUS"] = true;
    m_mAttributeChangeable["FILTER_V_STATUS"] = true;
    m_mAttributeChangeable["FILTER_P_STATUS"] = true;

    m_mAttributeChangeable["ACTUAL_CURRENT"] =  false;
    m_mAttributeChangeable["ACTUAL_VELOCITY"] = false;
    m_mAttributeChangeable["ACTUAL_POSITION"] = false;
    m_mAttributeChangeable["VOLTAGE"] = false;
    m_mAttributeChangeable["CURRENT_SCALE"] = false;
    m_mAttributeChangeable["VELOCITY_SCALE"] =false;
    m_mAttributeChangeable["ACTUATOR_TEMPERATURE"] = false;
    m_mAttributeChangeable["INVERTER_TEMPERATURE"] = false;
    m_mAttributeChangeable["ACTUATOR_SWITCH"] = false;
    m_mAttributeChangeable["ONLINE_STATUS"] = false;
    m_mAttributeChangeable["INIT_STATE"] = false;

    m_mAttributeChangeable["SN_ID"] = false;
    m_mAttributeChangeable["ERROR_ID"] = false;

    m_mAttributeChangeable["VERSION_430"] = false;
    m_mAttributeChangeable["FRENQUENCY_430"] = false;
    m_mAttributeChangeable["FIRMWARE_VERSION"] = false;

}