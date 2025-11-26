cmake_minimum_required(VERSION 3.8)
project(wheels)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                      # if you have C++ nodes
find_package(std_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # for msg/srv generation
find_package(rosidl_default_runtime REQUIRED)      # runtime support for generated code

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
   DEPENDENCIES geometry_msgs
)
install(
    PROGRAMS src/actionserver.py src/actionclient.py
    DESTINATION lib/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
