cmake_minimum_required(VERSION 3.8)
project(imu_port_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.c
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sonia_common_cpp REQUIRED)
find_package(sonia_common_ros2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package ( Boost COMPONENTS log REQUIRED )
include_directories(include)

file(GLOB SOURCES src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} rclcpp Boost sonia_common_cpp sonia_common_ros2 sensor_msgs std_msgs std_srvs geometry_msgs)

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

ament_package()
