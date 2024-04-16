cmake_minimum_required(VERSION 3.8)
project(Interaction_Controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create an executable
add_executable(admittance_controller
  src/admittance_controller.cpp
)
ament_target_dependencies(admittance_controller rclcpp geometry_msgs)

# Include directories
target_include_directories(admittance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(admittance_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS 
  admittance_controller
  DESTINATION lib/${PROJECT_NAME}
  )


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()