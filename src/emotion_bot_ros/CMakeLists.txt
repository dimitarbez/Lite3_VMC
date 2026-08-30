cmake_minimum_required(VERSION 3.0.2)
project(emotion_bot_ros)

find_package(catkin REQUIRED COMPONENTS
  controller_manager_msgs
  gazebo_msgs
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/chat_adapter.py
  scripts/emotion_adapter.py
  scripts/emotion_chat.py
  scripts/emotion_demo.py
  scripts/expression_mapper.py
  scripts/load_sim_controllers.py
  scripts/safety_bridge.py
  scripts/sim_ready_runner.py
  scripts/test_gazebo_e2e.py
  scripts/test_openai_live.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/emotion_bot_core.test)
endif()
