cmake_minimum_required(VERSION 3.0.2)
project(raspimouse_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  roscpp
  raspimouse_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  controller_manager joint_state_controller roscpp raspimouse_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(leds src/leds.cpp)
add_executable(motors src/motors.cpp src/raspimouse_hardware.cpp)
add_executable(buttons src/buttons.cpp)
add_executable(lightsensors src/lightsensors.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(motors ${catkin_LIBRARIES})
target_link_libraries(leds ${catkin_LIBRARIES})
target_link_libraries(buttons ${catkin_LIBRARIES})
target_link_libraries(lightsensors ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS leds motors buttons lightsensors
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/buzzer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark tests for installation
install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS test/test_buzzer.py test/test_lightsensors.py test/test_motors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  find_package(rostest REQUIRED)
  add_rostest(test/raspimouse_nodes.test)
endif()