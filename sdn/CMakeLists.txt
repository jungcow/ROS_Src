cmake_minimum_required(VERSION 3.0.2)
project(sdn)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GpsInfo.msg
  )
  
  ## Generate services in the 'srv' folder
add_service_files(
  FILES
  NewConn.srv
  RegisterDest.srv
  CheckNextHop.srv
  DelConn.srv
  StateInfo.srv
  )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  VehicleCommand.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sdn
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation message_runtime roscpp std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle src/vehicle.cpp)
add_dependencies(vehicle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle
	${catkin_LIBRARIES}
)

add_executable(edge_computer1 src/edge_computer1.cpp)
add_dependencies(edge_computer1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(edge_computer1
	${catkin_LIBRARIES}
)

# add_executable(edge_computer2 src/edge_computer2.cpp)
# add_dependencies(edge_computer2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(edge_computer2
# 	${catkin_LIBRARIES}
# )

# add_executable(controller src/controller.cpp)
# add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(controller
# 	${catkin_LIBRARIES}
# )

# add_executable(app_path_planning src/app_path_planning.cpp)
# add_dependencies(app_path_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(app_path_planning
# 	${catkin_LIBRARIES}
# )

# add_executable(app_traffic src/app_traffic.cpp)
# add_dependencies(app_traffic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(app_traffic
# 	${catkin_LIBRARIES}
# )
