cmake_minimum_required(VERSION 3.5)
project(data_publisher)

set(CMAKE_BUILD_TYPE Release)

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

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pre_interfaces REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights uncomment
    # the line when a copyright and license is not present in all source files
    # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint
    # (only works in a git repo) uncomment the line when this package is not in
    # a git repo set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

add_library(${PROJECT_NAME} src/DataPublisher.cc)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs pre_interfaces)

add_executable(${PROJECT_NAME}_node src/main.cc)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
