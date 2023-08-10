cmake_minimum_required(VERSION 2.8.3)
project(vrx_gazebo)

# We need erb to process the .world.erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

add_compile_options(-std=c++17)

# For Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Set policy for CMake 3.1+. Use OLD policy to let FindBoost.cmake, dependency
# of gazebo, use quoted variables in if()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()
# Use OLD policy to be able to use *_generate_services_cpp before they exist,
# without error.
if(POLICY CMP0046)
  cmake_policy(SET CMP0046 OLD)
endif()

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  geographic_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  wamv_gazebo
  wave_gazebo
  xacro
)

########################
## Message generation ##
########################

add_message_files(
  FILES
  Task.msg
  Contact.msg
)

add_service_files(
  FILES
  ColorSequence.srv
  BallShooter.srv
)

# Python scripts setup
catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS wamv_gazebo wave_gazebo xacro gazebo_dev geographic_msgs std_msgs message_runtime
  
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}") 

include_directories(include ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${QT_USE_FILE}
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_definitions(${QT_DEFINITIONS})


if(${GAZEBO_MAJOR_VERSION} GREATER 7)
  # Plugin for Task Info GUI Overlay
  add_library(gui_task_widget SHARED ${headers_MOC}
         	src/gui_task_widget.cc
  )

  target_link_libraries(gui_task_widget 
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${QT_LIBRARIES}
  )
  qt5_use_modules(gui_task_widget Widgets)
  add_dependencies(gui_task_widget
    ${catkin_EXPORTED_TARGETS}
    vrx_gazebo_generate_messages_cpp)
  install(TARGETS gui_task_widget
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()


set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

# Generate world files from xacro and install
xacro_add_files(
  worlds/example_course.world.xacro
  worlds/sydneyregatta.world.xacro
  ${XACRO_INORDER} INSTALL DESTINATION worlds
)
# Generate obstacle course
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_avoid_obstacles_buoys.py
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_avoid_obstacles_buoys.py --seed 1337 --a3 6 --a5 7 --a7 7 > ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf
)
add_custom_target(${PROJECT_NAME}_generate_obstacle_course_buoys ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf)

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

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)

install(PROGRAMS scripts/spawn_wamv.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(sandisland_test
                    test/sandisland.test
                    test/sandisland.cc)
  target_link_libraries(sandisland_test ${catkin_LIBRARIES})
endif()

# Python Scripts
catkin_install_python(PROGRAMS
    nodes/pinger_visualization.py
    nodes/set_pinger_position.py
    nodes/twist2thrust.py
    nodes/key2thrust_angle.py
    scripts/generate_worlds.py
    scripts/generate_wamv.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
