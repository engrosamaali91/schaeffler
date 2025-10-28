cmake_minimum_required(VERSION 3.8)
project(agv_orchestrator)

find_package(ament_cmake REQUIRED)

# Install launch files into the standard location
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# NEW: install rviz configs
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install Python execs into lib/${PROJECT_NAME} so launch can run them
install(PROGRAMS scripts/nav2_test.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
