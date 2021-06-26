cmake_minimum_required(VERSION 3.5)
project(actuatorcontroller_ros)

add_compile_options(-std=c++14)
add_compile_options(-fPIC)
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)
add_compile_options(-DIMU_ENABLE)
add_compile_options(-O3)

MESSAGE("CMAKE_SYSTEM_PROCESSOR:   ${CMAKE_SYSTEM_PROCESSOR}")

if( "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
 set(BUILD_PLATFORM
         X86)
else()
 set(BUILD_PLATFORM
         TX2)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  std_srvs
        sensor_msgs
)






## Generate messages in the 'msg' folder
 add_message_files(
   FILES
         ActuatorAttribute.msg
         ActuatorCommand.msg
         ActuatorModes.msg
         ActuatorArray.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
         AttributeLookup.srv
         AttributeQuery.srv
         GeneralQuery.srv
         AttributeDictionary.srv
         DebugQuery.srv
         TriviaQuery.srv
         IDModify.srv
         ParametersSave.srv
         ZeroReset.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

include_directories(
        include
        include/${PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/include
        ${catkin_INCLUDE_DIRS}

)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES actuatorcontroller_ros
  CATKIN_DEPENDS message_generation roscpp std_msgs std_srvs
  #DEPENDS system_lib
)

###########
## Build ##
###########







## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/actuatorcontroller_ros.cpp
         src/${PROJECT_NAME}/actuatorcontroller_rosparam.cpp
         src/${PROJECT_NAME}/actuatorcontroller_rosservice.cpp
 )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/lib/${BUILD_PLATFORM}/libActuatorController.so
)


add_executable(innfos_actuator src/innfos_actuator.cpp)
target_link_libraries(innfos_actuator  ${PROJECT_NAME})
add_dependencies(innfos_actuator ${PROJECT_NAME} )