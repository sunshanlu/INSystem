cmake_minimum_required(VERSION 3.5)
project(pre_insystem)

set(CMAKE_BUILD_TYPE Release)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(/usr/include/eigen3 ${CMAKE_CURRENT_SOURCE_DIR}/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pre_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

file(GLOB PROJECT_SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME} ${PROJECT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${Pangolin_LIBRARIES} Sophus::Sophus
                      yaml-cpp g2o_core g2o_stuff)
ament_target_dependencies(${PROJECT_NAME} rclcpp pre_interfaces sensor_msgs)

add_executable(pre_insystem_dataset example/pre_insystem_dataset.cc)
target_link_libraries(pre_insystem_dataset ${PROJECT_NAME})

add_executable(pre_insystem_ros2 example/pre_insystem_ros2.cc)
target_link_libraries(pre_insystem_ros2 ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} pre_insystem_dataset pre_insystem_ros2
        DESTINATION lib/${PROJECT_NAME})
ament_package()
