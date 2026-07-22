cmake_minimum_required(VERSION 3.16)
project(spacecraft_reaction_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_library(rcs_plume_system SHARED src/rcs_plume_system.cpp)
target_compile_features(rcs_plume_system PRIVATE cxx_std_17)
target_link_libraries(rcs_plume_system PRIVATE
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-plugin2::gz-plugin2
)
install(TARGETS rcs_plume_system DESTINATION lib)

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
