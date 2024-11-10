cmake_minimum_required(VERSION 3.8)
project(interaction_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)

find_package(interaction_msgs REQUIRED)


## Include Cpp "include" directory
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})


## Create Cpp executables and link with the dependencies 
add_executable(admittance_controller
  src/admittance_controller.cpp
)
add_executable(direct_force_controller
  src/direct_force_controller.cpp
)
ament_target_dependencies(admittance_controller rclcpp geometry_msgs px4_msgs interaction_msgs)
ament_target_dependencies(direct_force_controller rclcpp geometry_msgs px4_msgs interaction_msgs)


## Install Cpp executables in the /ros_ws2/install/...
install(TARGETS
  admittance_controller
  direct_force_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules and scripts
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/simple_interaction_setpoints.py
  scripts/multiple_point_interaction.py
  scripts/slide_interaction.py
  scripts/experiment_path.py
  scripts/tilt_path.py


  DESTINATION lib/${PROJECT_NAME}
)

# Install executables

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
