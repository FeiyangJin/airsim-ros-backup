cmake_minimum_required(VERSION 3.10.0)
project(airsim_ros_pkgs)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../AirSim/)
set (common_utility_src ${AIRSIM_ROOT}/ros/src/common)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 11)


set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# todo eigen3 in AirLib already
# find_package(Eigen3 REQUIRED) 
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)


# package add by feiyang jin
# pcl is for octomap_server
  find_package( PCL REQUIRED )
  include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  find_package(octomap REQUIRED)
  find_package(octomap_server REQUIRED)
  find_package(ompl REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  message_generation
  nav_msgs
  # nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  mav_trajectory_generation
  mav_trajectory_generation_ros
)

add_message_files(
  FILES
  GimbalAngleEulerCmd.msg
  GimbalAngleQuatCmd.msg
  GPSYaw.msg
  VelCmd.msg
  VelCmdGroup.msg
  multiDOF.msg
  multiDOF_array.msg
  BoolPlusHeader.msg
)

add_service_files(
  FILES
  SetGPSPosition.srv
  Takeoff.srv
  TakeoffGroup.srv
  Land.srv
  LandGroup.srv
  Reset.srv
  SetLocalPosition.srv
  get_trajectory.srv
  follow_trajectory_status_srv.srv
  profiling_data_srv.srv
  start_profiling_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${common_utility_src}
)

add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser ${catkin_LIBRARIES} AirLib)

add_library(pd_position_controller_simple src/pd_position_controller_simple.cpp)
add_dependencies(pd_position_controller_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pd_position_controller_simple ${catkin_EXPORTED_TARGETS})

add_library(airsim_ros src/airsim_ros_wrapper.cpp)
add_dependencies(airsim_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

  # library created by feiyang jin
  add_library(common_mav src/common_mav.cpp)
  target_link_libraries(common_mav airsim_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node airsim_ros ${catkin_LIBRARIES} AirLib)

add_executable(pd_position_controller_simple_node src/pd_position_controller_simple_node.cpp)
target_link_libraries(pd_position_controller_simple_node pd_position_controller_simple airsim_ros ${catkin_LIBRARIES} AirLib)

  # executable add by feiyang jin
  add_executable(scanning_test src/scanning_test.cpp)
  target_link_libraries(scanning_test ${catkin_LIBRARIES})

  add_executable(scanning_node src/scanning_node.cpp)
  target_link_libraries(scanning_node airsim_ros ${catkin_LIBRARIES} AirLib common_mav)

  add_executable(package_delivery_node src/package_delivery_node.cpp)
  target_link_libraries(package_delivery_node airsim_ros ${catkin_LIBRARIES} AirLib common_mav)

  add_executable(package_delivery_static src/package_delivery_withMap.cpp)
  target_link_libraries(package_delivery_static airsim_ros ${catkin_LIBRARIES} AirLib common_mav)

  add_executable(motion_planner_test src/motion_planner_test.cpp)
  target_link_libraries(motion_planner_test ${catkin_LIBRARIES} libompl.so ${OCTOMAP_LIBRARIES})

  add_executable(motion_planner src/motion_planner.cpp)
  target_link_libraries(motion_planner airsim_ros ${catkin_LIBRARIES} libompl.so ${OCTOMAP_LIBRARIES})

  add_executable(follow_trajectory src/follow_trajectory_node.cpp)
  target_link_libraries(follow_trajectory airsim_ros ${catkin_LIBRARIES} AirLib common_mav)

  add_executable(follow_trajectory_static src/follow_trajectory_static.cpp)
  target_link_libraries(follow_trajectory_static airsim_ros ${catkin_LIBRARIES} AirLib common_mav)

  add_executable(panic_pcl ${common_utility_src}/panic_pcl.cpp)
  target_link_libraries(panic_pcl ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread ${OpenCV_LIBS} common_mav ${PCL_LIBRARIES})

  add_executable(future_collision src/future_collision.cpp)
  target_link_libraries(future_collision airsim_ros ${catkin_LIBRARIES} AirLib MavLinkCom rpc pthread ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES})

  # profile manager
  # added 04/27/2020
  if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")
    include_directories(/usr/local/cuda/include)
    link_directories(/usr/lib/nvidia-384)
    add_definitions(-DUSE_INTEL -DUSE_NVML)
    add_executable(profile_manager src/profile_manager.cpp ${common_utility_src}/Profiling.cpp  ${common_utility_src}/timer.cpp)
    target_link_libraries(profile_manager  ${catkin_LIBRARIES}  AirLib MavLinkCom rpc pthread nvidia-ml airsim_ros MavLinkCom pthread ${OpenCV_LIBS} common_mav)
  else(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")
    add_executable(profile_manager src/profile_manager.cpp 
    ${common_utility_src}/Profiling.cpp ${common_utility_src}/timer.cpp)
    target_link_libraries(profile_manager  ${catkin_LIBRARIES}  AirLib MavLinkCom rpc pthread airsim_ros MavLinkCom pthread ${OpenCV_LIBS} common_mav)
  endif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")

  add_dependencies(profile_manager ${catkin_EXPORTED_TARGETS} airsim_ros_pkgs_generate_messages_cpp)


install(TARGETS
    #list of nodes
    pd_position_controller_simple_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
    #list of shared libraries
    airsim_ros
    pd_position_controller_simple
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES 
  #list of necessary files (xml...)
  README.md
  # settings.json
  # nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)