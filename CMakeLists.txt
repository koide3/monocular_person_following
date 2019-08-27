cmake_minimum_required(VERSION 2.8.3)
project(monocular_person_following)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  tf
  rospy
  roscpp
  cv_bridge
  message_filters
  image_transport
  eigen_conversions
  message_generation
  ccf_person_identification
  monocular_people_tracking
)

########################
## service generation ##
########################
add_service_files(FILES
  Imprint.srv
)

########################
## message generation ##
########################
add_message_files(FILES
  BoundingBox2D.msg
  FaceDetection.msg
  FaceDetectionArray.msg
  Target.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES monocular_person_following
    #  CATKIN_DEPENDS other_catkin_pkg
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(monocular_person_following_node
  src/monocular_person_following/state/initial_state.cpp
  src/monocular_person_following/state/initial_training_state.cpp
  src/monocular_person_following/state/tracking_state.cpp
  src/monocular_person_following/state/reid_state.cpp
  src/monocular_person_following/context.cpp
  src/monocular_person_following_node.cpp
)
add_dependencies(monocular_person_following_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(monocular_person_following_node
  ${catkin_LIBRARIES}
)
