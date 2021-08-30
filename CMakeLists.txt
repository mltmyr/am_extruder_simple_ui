cmake_minimum_required(VERSION 3.5)
project(am_extruder_simple_ui)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake_python REQUIRED)

# define target
set(am_extruder_simple_ui_SRCS
  src/simple_extruder_ui.cpp
  src/ui_simple_extruder_ui.cpp
)

set(am_extruder_simple_ui_HDRS
  include/am_extruder_simple_ui/simple_extruder_ui.hpp
  include/am_extruder_simple_ui/ui_simple_extruder_ui.hpp
)

set(am_extruder_simple_ui_INCLUDE_DIRECTORIES
  include
  ${rclcpp_INCLUDE_DIRS}
  ${qt_gui_cpp_INCLUDE_DIRS}
  ${rqt_gui_cpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_cpp(am_extruder_simple_ui_MOCS ${am_extruder_simple_ui_HDRS})

include_directories(
  ${am_extruder_simple_ui_INCLUDE_DIRECTORIES}
)

add_library(${PROJECT_NAME} SHARED
  ${am_extruder_simple_ui_SRCS}
  ${am_extruder_simple_ui_MOCS}
)

target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${qt_gui_cpp_LIBRARIES}
  ${rqt_gui_cpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  Qt5::Widgets
)

# install target
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJCT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(PROGRAMS scripts/am_extruder_simple_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES am_extruder_simple_ui_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "am_extruder_simple_ui_plugin.xml")

#ament_target_dependencies(${PROJECT_NAME}
#  rclcpp
#  std_msgs
#  sensor_msgs
#  pluginlib
#  qt_gui
#  rqt_gui_cpp
#)

#install(DIRECTORY include/${PROJECT_NAME} DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
