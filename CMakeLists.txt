cmake_minimum_required(VERSION 2.8.3)
project(perceptive_mpc)

add_compile_options(-std=c++14)

set(PACKAGE_DEPENDENCIES
    roslib
    tf
    geometry_msgs
    ocs2_comm_interfaces
    ocs2_core
    ocs2_robotic_tools
    ocs2_ddp
    kindr
    kindr_ros
    voxblox
    voxblox_ros
    m3dp_msgs
    actionlib
)

find_package(catkin REQUIRED COMPONENTS
    ${PACKAGE_DEPENDENCIES}

)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  )

#add_message_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  geometry_msgs
#)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    perceptive_mpc
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    
  DEPENDS
    Boost
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_explicit_instantiations
  src/ExplicitTemplateInstantiations.cpp)

add_dependencies(${PROJECT_NAME}_explicit_instantiations ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_explicit_instantiations
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
  )

add_library(${PROJECT_NAME}
  src/SystemDynamics.cpp
  src/costs/QuadraticEndeffectorTrackingCost.cpp
  src/costs/QuadraticBaseEETrackingCost.cpp
  src/costs/QuadraticBaseTrackingCost.cpp
  src/costs/BaseAvoidanceCost.cpp
  src/costs/VoxbloxCost.cpp
  src/costs/PointsOnRobot.cpp  
  src/OperatingPoint.cpp  
  src/kinematics/KinematicsInterface.cpp
  src/kinematics/asArm/asArmKinematics.cpp
  src/kinematics/mabi/MabiKinematics.cpp
  src/kinematics/ur10/UR10Kinematics.cpp
  src/kinematics/ur5/UR5Kinematics.cpp
  src/kinematics/ur3/UR3Kinematics.cpp
  src/EsdfCachingServer.cpp
  src/AdmittanceReferenceModule.cpp
)

add_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_explicit_instantiations
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )

########################### 

add_library(${PROJECT_NAME}_armstone        
        armstone/AsPerceptiveMpcInterface.cpp
        )

add_dependencies(${PROJECT_NAME}_armstone
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_armstone
        ${PROJECT_NAME}
        ${PROJECT_NAME}_explicit_instantiations
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )

###########################

add_executable(as_pmpc_node
        armstone/as_pmpc_node.cpp
        armstone/AsPMPC.cpp
        armstone/AsPMPC_RosApi.cpp
        armstone/AsPMPC_Help.cpp)

add_dependencies(as_pmpc_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(as_pmpc_node
        ${PROJECT_NAME}
        ${PROJECT_NAME}_explicit_instantiations
        ${PROJECT_NAME}_armstone
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
