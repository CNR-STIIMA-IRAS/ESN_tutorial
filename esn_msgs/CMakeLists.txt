cmake_minimum_required(VERSION 3.8)
project(esn_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # <— per generare msg/srv/action

# Dipendenze dei tipi usati dentro i tuoi .msg/.srv/.action
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Genera le interfacce
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectArrived.msg"
  "srv/DetectObject.srv"
  "action/PickObject.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

# Export per chi usa queste interfacce a runtime
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
