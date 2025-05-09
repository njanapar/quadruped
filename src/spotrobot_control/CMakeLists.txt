cmake_minimum_required(VERSION 3.8)
project(spotrobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/robot_controller_gazebo.py
  scripts/test_joy.py
  launch/robot_control.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/RoboticsUtilities/Transformations.py
  DESTINATION lib/${PROJECT_NAME}/RoboticsUtilities/
)

install(
  PROGRAMS
  scripts/RobotController/CrawlGaitController.py
  scripts/RobotController/GaitController.py
  scripts/RobotController/PIDController.py
  scripts/RobotController/RobotController.py
  scripts/RobotController/StandController.py
  scripts/RobotController/StateCommand.py
  scripts/RobotController/TrotGaitController.py
  DESTINATION lib/${PROJECT_NAME}/RobotController/
)

install(
  PROGRAMS
  scripts/InverseKinematics/robot_IK.py
  DESTINATION lib/${PROJECT_NAME}/InverseKinematics/
)

ament_package()
