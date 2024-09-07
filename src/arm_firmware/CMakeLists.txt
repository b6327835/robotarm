cmake_minimum_required(VERSION 3.8)
project(arm_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

ament_python_install_package(${PROJECT_NAME})

add_executable(simple_serial_receiver src/simple_serial_receiver.cpp)
ament_target_dependencies(simple_serial_receiver rclcpp std_msgs)
target_include_directories(simple_serial_receiver PUBLIC
 "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(simple_serial_receiver
 "${SERIAL_LDFLAGS}")

add_executable(simple_serial_transmitter src/simple_serial_transmitter.cpp)
ament_target_dependencies(simple_serial_transmitter rclcpp std_msgs)
target_include_directories(simple_serial_transmitter PUBLIC
 "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(simple_serial_transmitter
 "${SERIAL_LDFLAGS}")

# add_executable(stepper src/stepper.cpp)
# ament_target_dependencies(stepper rclcpp std_msgs)
# target_include_directories(stepper PUBLIC
#  "${SERIAL_INCLUDE_DIRS}")
# target_link_libraries(stepper
#  "${SERIAL_LDFLAGS}")

# add_executable(stepper2 src/stepper2.cpp)
# ament_target_dependencies(stepper2 rclcpp std_msgs)
# target_include_directories(stepper2 PUBLIC
#  "${SERIAL_INCLUDE_DIRS}")
# target_link_libraries(stepper2
#  "${SERIAL_LDFLAGS}")

install(TARGETS
  simple_serial_receiver
  simple_serial_transmitter
  # stepper
  # stepper2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/simple_serial_transmitter.py
  ${PROJECT_NAME}/simple_serial_receiver.py
  ${PROJECT_NAME}/stepper.py
  ${PROJECT_NAME}/stepper2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
