cmake_minimum_required(VERSION 3.8)
project(fruit_models)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# This line installs your models directory so Gazebo can find it later
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
