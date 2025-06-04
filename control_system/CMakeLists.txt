cmake_minimum_required(VERSION 3.0.2)
project(control_system)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  cmake_modules
  freyja_msgs
  control_system_msgs
  mavros_msgs
)
find_package(Eigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

if( NOT EIGEN3_FOUND )
  # resort to cmake_modules
  find_package( cmake_modules REQUIRED )
  find_package(Eigen REQUIRED)
  set( EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} )
  set( EIGEN3_LIBRARIES ${EIGEN_LIBRARIES} )
else()
  set( EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} )
endif()

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   FullState.msg
   ReferenceState.msg
   QState.msg
   ControllerDebug.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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
   freyja_msgs
   control_system_msgs
   mavros_msgs
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES control_system
   CATKIN_DEPENDS Eigen3 message_generation roscpp rospy std_msgs freyja_msgs control_system_msgs mavros_msgs message_runtime
   DEPENDS system_lib Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)




add_executable(trajectory_generator src/trajectory_generator.cpp)
add_dependencies(trajectory_generator ${trajectory_generator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_generator ${catkin_LIBRARIES})

add_executable(unscented_kalman_filter src/unscented_kalman_filter.cpp)
add_dependencies(unscented_kalman_filter ${unscented_kalman_filter_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(unscented_kalman_filter ${catkin_LIBRARIES})

add_executable(controller src/controller.cpp)
add_dependencies(controller ${controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})
