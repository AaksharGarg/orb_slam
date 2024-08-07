cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclpy REQUIRED)
# execute_process(COMMAND python scripts/disparity_node.py 
#     WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})


install(
  DIRECTORY 
  launch
  gazebo
  meshes
  urdf
  worlds
  config
  model
  scripts
  DESTINATION share/${PROJECT_NAME}/
  )


  install(PROGRAMS
  scripts/dispartiy_node.py
  DESTINATION lib/${PROJECT_NAME}
)

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
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.sh.in")


ament_package()
