cmake_minimum_required(VERSION 3.16)
project(spacecraft_reaction_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS spacecraft_reaction_sim/reaction_experiment.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME reaction_experiment
)
install(
  PROGRAMS spacecraft_reaction_sim/attitude_hold.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME attitude_hold
)
install(
  PROGRAMS spacecraft_reaction_sim/arm_position_hold.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_position_hold
)
install(
  PROGRAMS spacecraft_reaction_sim/thruster_pulse.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME thruster_pulse
)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(reaction_experiment_test test/test_reaction_experiment.py)
  ament_add_pytest_test(attitude_hold_test test/test_attitude_hold.py)
  ament_add_pytest_test(arm_position_hold_test test/test_arm_position_hold.py)
  ament_add_pytest_test(thruster_pulse_test test/test_thruster_pulse.py)
endif()

ament_package()
