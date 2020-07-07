cmake_minimum_required(VERSION 2.8.3)
project(vrx_gazebo)

# We need erb to process the .world.erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTOUIC ON)

find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package (Protobuf REQUIRED)
find_package (gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  aquanaute_description
  wave_gazebo
  xacro
  gazebo_dev
  geographic_msgs
  std_msgs
  rospy
  roscpp
)

########################
## Message generation ##
########################

#add_message_files(
#  FILES
#  Task.msg
#  Contact.msg
#)

#add_service_files(
#  FILES
#  ColorSequence.srv
#)

# Python scripts setup
catkin_python_setup()

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  CATKIN_DEPENDS aquanaute_description wave_gazebo xacro gazebo_dev
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}") 

include_directories(include ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${QT_USE_FILE}
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)

add_definitions(${QT_DEFINITIONS})

# Generate world files from xacro and install
xacro_add_files(
  worlds/example_course.world.xacro
  worlds/sandisland.world.xacro
  worlds/ocean.world.xacro
  INORDER INSTALL DESTINATION worlds
)

# Install all the config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(PROGRAMS scripts/spawn_wamv.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(sandisland_test
                    test/sandisland.test
                    test/sandisland.cc)
  target_link_libraries(sandisland_test ${catkin_LIBRARIES})
endif()


