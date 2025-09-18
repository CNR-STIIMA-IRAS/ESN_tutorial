# Info here: https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.8)
project(esn_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trova le dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(esn_msgs REQUIRED)

# Dichiara una libreria C++
add_library(${PROJECT_NAME} SHARED
            src/limit_switch.cpp
 )

# Importa le cartelle "include" nella libreria
 target_include_directories(${PROJECT_NAME} PUBLIC
   $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Definisci le dipendenze della libreria per la compilazione
 ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp esn_msgs)

# Rendi la libreria visibile e importabile da altri pacchetti
 ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Definisci le dipendenze del tuo pacchetto per i pacchetti esterni che lo utilizzeranno
 ament_export_dependencies(rclcpp esn_msgs)

 # Definizione eseguibili
add_executable(limit_switch_node src/limit_switch_node.cpp)
target_link_libraries(limit_switch_node ${PROJECT_NAME})

# Installa librerei ed eseguibili
install(TARGETS ${PROJECT_NAME}
 EXPORT export_${PROJECT_NAME}
 ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin
 INCLUDES DESTINATION include
)

install(TARGETS limit_switch_node
    DESTINATION lib/${PROJECT_NAME})

install(
 DIRECTORY include/
 DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
