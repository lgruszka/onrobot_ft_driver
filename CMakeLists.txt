cmake_minimum_required(VERSION 2.8.3)
project(onrobot_ft_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  geometry_msgs
  diagnostic_msgs
  diagnostic_updater
)

find_package(Boost REQUIRED COMPONENTS system thread program_options)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES onrobot_ft_udp_driver
  CATKIN_DEPENDS roscpp std_srvs geometry_msgs diagnostic_msgs diagnostic_updater
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Driver library
add_library(onrobot_ft_udp_driver
  src/etherdaq_driver.cpp
)
target_link_libraries(onrobot_ft_udp_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Sensor node
add_executable(ft_sensor_node src/ft_sensor_node.cpp)
target_link_libraries(ft_sensor_node
  onrobot_ft_udp_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS onrobot_ft_udp_driver ft_sensor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
