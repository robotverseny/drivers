cmake_minimum_required(VERSION 3.5)
project(lsn10)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost 1.65.1 REQUIRED COMPONENTS system filesystem thread)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)

add_executable(lsn10
                src/lsiosr.cpp
                src/lsn10.cpp)
include_directories(lsn10 ${Boost_INCLUDE_DIRS})
link_directories(lsn10 ${Boost_LIBRARY_DIRS})
target_link_libraries(lsn10 ${Boost_LIBRARIES})
target_link_libraries(lsn10 ${Boost_PROGRAM_OPTIONS_LIBRARY})
ament_target_dependencies(lsn10 rclcpp sensor_msgs std_srvs Boost)


install(TARGETS lsn10
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
