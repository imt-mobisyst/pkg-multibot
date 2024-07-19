cmake_minimum_required(VERSION 3.8)
project(communication_test)

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

# Install launch files, config and world files
install(DIRECTORY
  launch
  config
  world
  map
  DESTINATION share/${PROJECT_NAME}/
)

# Python includes
install(DIRECTORY
  src/include
DESTINATION lib/${PROJECT_NAME})

# Python scripts
install( PROGRAMS
  # Robot controllers
  src/turtlesim_controller.py
  src/stage_controller.py
  src/kobuki_warehouse_controller.py
  src/kobuki_measure_controller.py

  # Real Robot
  src/wifi_measure.py

  # Turtlesim movement
  src/turtle_mvt.py

  # Simulation
  src/stage_fake_localization.py
  src/stage_dds_bridge.py
  src/package_dispenser.py

  # Operator nodes to control the fleet
  src/static_operator.py
  src/dynamic_operator.py
  src/warehouse_operator.py
  src/parasit.py

  src/costmap_publisher.py
DESTINATION lib/${PROJECT_NAME})

ament_package()
