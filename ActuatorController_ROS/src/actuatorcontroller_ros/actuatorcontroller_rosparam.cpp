#include "actuatorcontroller_ros.h"
#include <iostream>

void ActuatorController_ROS::initializeROSParam(){

    if (no_param){

    }else {
        std::vector<uint8_t> temp_vec = m_pController->getActuatorIdArray();

        for (uint8_t act_id : temp_vec){

            initializeROSParam(act_id);

        }
    }

}


void ActuatorController_ROS::initializeROSParam(uint8_t joint_id){

    if (no_param){

    }
    else {
        if ((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized) {
            std::string temp_act_str = m_sINNFOS + m_sActuator + std::to_string(int(joint_id));

            for (std::map<std::string, ActuatorAttribute>::iterator iter = m_mChangableDoubleAttribute.begin();
                 iter != m_mChangableDoubleAttribute.end(); iter++) {

                ros::param::set(temp_act_str + "/" + iter->first,
                                m_pController->getActuatorAttribute(joint_id, iter->second));

            }

            for (std::map<std::string, ActuatorAttribute>::iterator iter = m_mChangableIntAttribute.begin();
                 iter != m_mChangableIntAttribute.end(); iter++) {

                ros::param::set(temp_act_str + "/" + iter->first,
                                int(m_pController->getActuatorAttribute(joint_id, iter->second)));

            }


        }
    }

}



void ActuatorController_ROS::clearROSParam(){
    if (no_param){

    }
    else {
        std::vector<uint8_t> temp_vec = m_pController->getActuatorIdArray();

        for (uint8_t act_id : temp_vec){

            clearROSParam(act_id);

        }
    }

}


void ActuatorController_ROS::clearROSParam(uint8_t joint_id){
    if (no_param){

    }else {
        std::string temp_act_str = m_sINNFOS + m_sActuator + std::to_string(int(joint_id));

        ros::param::del(temp_act_str);
    }


}



void ActuatorController_ROS::updateROSParam(){

    if (no_param){

    }
    else{
        std::vector<uint8_t> temp_vec = m_pController->getActuatorIdArray();

        for (uint8_t act_id : temp_vec){

            updateROSParam(act_id);

        }

    }

}


void ActuatorController_ROS::updateROSParam(uint8_t joint_id){

    if (no_param){

    }
    else {

        if ((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized) {
            std::string temp_act_str = m_sINNFOS + m_sActuator + std::to_string(int(joint_id));

            double temp_double;

            // iterate through the map of changeable attributes with double values
            for (std::map<std::string, ActuatorAttribute>::iterator iter = m_mChangableDoubleAttribute.begin();
                 iter != m_mChangableDoubleAttribute.end(); iter++) {


                if (ros::param::get(temp_act_str + "/" + iter->first,temp_double)){

                    if (temp_double == m_pController->getActuatorAttribute(joint_id , iter->second)){
                        continue;
                    }else {
                        std::string message = "Change Parameter " + iter->first + " for actuator " + std::to_string(int(joint_id)) + " to " + std::to_string(temp_double);
                        ROS_INFO("%s\n", message.c_str());
                        if (m_pController->setActuatorAttributeWithACK(m_pController->toLongId(joint_id) , iter->second , temp_double)){

                        }else{
                            ROS_INFO("Parameters setting failed! reverting...");
                            ros::param::set(temp_act_str + "/" + iter->first,m_pController->getActuatorAttribute(joint_id , iter->second ));
                        }

                    }

                }
            }



            int temp_int;

//        // iterate through the map of changeable attributes with integer values
//        for (std::map<std::string, ActuatorAttribute>::iterator iter = m_mChangableIntAttribute.begin();
//             iter != m_mChangableIntAttribute.end(); iter++) {
//
//
//            if (ros::param::get(temp_act_str + "/" + iter->first,temp_int)){
//
//                if (temp_int == int(m_pController->getActuatorAttribute(joint_id , iter->second))){
//                    continue;
//                }else {
//
//                    if (m_pController->setActuatorAttributeWithACK(m_pController->toLongId(joint_id) , iter->second , temp_int)){
//                        std::string message = "Change Parameter " + iter->first + "for actuator " + std::to_string(int(joint_id));
//                        ROS_INFO("%s\n", message.c_str());
//                    }else{
//                        ROS_INFO("Parameters setting failed! reverting...");
//                        ros::param::set(temp_act_str + "/" + iter->first, int(m_pController->getActuatorAttribute(joint_id , iter->second )));
//                    }
//
//                }
//
//
//            }
//        }


            //


        }
    }


}

void ActuatorController_ROS::populateAttributeMap(){

    m_mChangableDoubleAttribute["CUR_PROPORTIONAL"] = ActuatorAttribute::CUR_PROPORTIONAL;
    m_mChangableDoubleAttribute["CUR_INTEGRAL"] = ActuatorAttribute::CUR_INTEGRAL;
    m_mChangableDoubleAttribute["CUR_MAXSPEED"] = ActuatorAttribute::CUR_MAXSPEED;
    m_mChangableDoubleAttribute["VEL_PROPORTIONAL"] = ActuatorAttribute::VEL_PROPORTIONAL;
    m_mChangableDoubleAttribute["VEL_INTEGRAL"] = ActuatorAttribute::VEL_INTEGRAL;
    m_mChangableDoubleAttribute["VEL_OUTPUT_LIMITATION_MINIMUM"] = ActuatorAttribute::VEL_OUTPUT_LIMITATION_MINIMUM;
    m_mChangableDoubleAttribute["VEL_OUTPUT_LIMITATION_MAXIMUM"] = ActuatorAttribute::VEL_OUTPUT_LIMITATION_MAXIMUM;
    m_mChangableDoubleAttribute["POS_PROPORTIONAL"] = ActuatorAttribute::POS_PROPORTIONAL;
    m_mChangableDoubleAttribute["POS_INTEGRAL"] = ActuatorAttribute::POS_INTEGRAL;
    m_mChangableDoubleAttribute["POS_DIFFERENTIAL"] = ActuatorAttribute::POS_DIFFERENTIAL;
    m_mChangableDoubleAttribute["POS_OUTPUT_LIMITATION_MINIMUM"] = ActuatorAttribute::POS_OUTPUT_LIMITATION_MINIMUM;
    m_mChangableDoubleAttribute["POS_OUTPUT_LIMITATION_MAXIMUM"] = ActuatorAttribute::POS_OUTPUT_LIMITATION_MAXIMUM;
    m_mChangableDoubleAttribute["POS_LIMITATION_MINIMUM"] = ActuatorAttribute::POS_LIMITATION_MINIMUM;
    m_mChangableDoubleAttribute["POS_LIMITATION_MAXIMUM"] = ActuatorAttribute::POS_LIMITATION_MAXIMUM;
    m_mChangableDoubleAttribute["PROFILE_POS_MAX_SPEED"] = ActuatorAttribute::PROFILE_POS_MAX_SPEED;
    m_mChangableDoubleAttribute["PROFILE_POS_ACC"] = ActuatorAttribute::PROFILE_POS_ACC;
    m_mChangableDoubleAttribute["PROFILE_POS_DEC"] = ActuatorAttribute::PROFILE_POS_DEC;
    m_mChangableDoubleAttribute["PROFILE_VEL_MAX_SPEED"] = ActuatorAttribute::PROFILE_VEL_MAX_SPEED;
    m_mChangableDoubleAttribute["PROFILE_VEL_ACC"] = ActuatorAttribute::PROFILE_VEL_ACC;
    m_mChangableDoubleAttribute["PROFILE_VEL_DEC"] = ActuatorAttribute::PROFILE_VEL_DEC;
    m_mChangableDoubleAttribute["POS_OFFSET"] = ActuatorAttribute::POS_OFFSET;
    m_mChangableDoubleAttribute["POS_LIMITATION_SWITCH"] = ActuatorAttribute::POS_LIMITATION_SWITCH;
    m_mChangableDoubleAttribute["FILTER_C_STATUS"] = ActuatorAttribute::FILTER_C_STATUS;
    m_mChangableDoubleAttribute["FILTER_C_VALUE"] = ActuatorAttribute::FILTER_C_VALUE;
    m_mChangableDoubleAttribute["FILTER_V_STATUS"] = ActuatorAttribute::FILTER_V_STATUS;
    m_mChangableDoubleAttribute["FILTER_V_VALUE"] = ActuatorAttribute::FILTER_V_VALUE;
    m_mChangableDoubleAttribute["FILTER_P_STATUS"] = ActuatorAttribute::FILTER_P_STATUS;
    m_mChangableDoubleAttribute["FILTER_P_VALUE"] = ActuatorAttribute::FILTER_P_VALUE;
    m_mChangableDoubleAttribute["LOCK_ENERGY"] = ActuatorAttribute::LOCK_ENERGY;
    m_mChangableDoubleAttribute["ACTUATOR_PROTECT_TEMPERATURE"] = ActuatorAttribute::ACTUATOR_PROTECT_TEMPERATURE;
    m_mChangableDoubleAttribute["ACTUATOR_RECOVERY_TEMPERATURE"] = ActuatorAttribute::ACTUATOR_RECOVERY_TEMPERATURE;
    m_mChangableDoubleAttribute["INVERTER_PROTECT_TEMPERATURE"] = ActuatorAttribute::INVERTER_PROTECT_TEMPERATURE;
    m_mChangableDoubleAttribute["INVERTER_RECOVERY_TEMPERATURE"] = ActuatorAttribute::INVERTER_RECOVERY_TEMPERATURE;
    m_mChangableDoubleAttribute["CURRENT_LIMIT"] = ActuatorAttribute::CURRENT_LIMIT;
    m_mChangableDoubleAttribute["VELOCITY_LIMIT"] = ActuatorAttribute::VELOCITY_LIMIT;



//    m_mChangableIntAttribute["MODE_ID"] = ActuatorAttribute::MODE_ID;
//    m_mChangableIntAttribute["DEVICE_ID"] = ActuatorAttribute::DEVICE_ID;


}