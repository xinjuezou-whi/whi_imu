cmake_minimum_required(VERSION 3.5)
project(whi_imu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(whi_interfaces REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

## Declare a C++ library
file(GLOB WHI_SDK_SRC
  "src/sdk/*.cpp"
)
## Define third party link directory
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/3rd_lib/controlcan/64bit # arm64
  # ${CMAKE_CURRENT_SOURCE_DIR}/3rd_lib/controlcan/64bit/x86 # amd64
)
add_library(${PROJECT_NAME} ${WHI_SDK_SRC})
## Link to the third party libraries
target_link_libraries(${PROJECT_NAME}
  controlcan
  yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  serial
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  whi_interfaces
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)
# arm64
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rd_lib/controlcan/64bit
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so"
)
# amd64
# install(
#   FILES ${CMAKE_CURRENT_SOURCE_DIR}/3rd_lib/controlcan/64bit/x86/libcontrolcan.so
#   DESTINATION lib
# )
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
