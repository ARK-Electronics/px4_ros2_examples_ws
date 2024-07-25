cmake_minimum_required(VERSION 3.5)
project(custom_mode)

# Adding compiler options for warnings and errors
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

# Finding required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Including directories for Eigen3
include_directories(${Eigen3_INCLUDE_DIRS})

# Adding executable target
add_executable(custom_mode CustomMode.cpp)

# Specifying dependencies for the target
ament_target_dependencies(custom_mode rclcpp Eigen3 px4_ros2_cpp geometry_msgs)

# Setting compile features for the target
target_compile_features(custom_mode PUBLIC c_std_99 cxx_std_17)

# Installing the target
install(TARGETS custom_mode DESTINATION lib/${PROJECT_NAME})

# Installing the launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Adding lint tests if testing is enabled
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Declaring the ament package
ament_package()
