cmake_minimum_required(VERSION 2.8.3)

project(nmea_navsat_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp 
    roslint
  )
add_message_files(FILES
  nmea_msg.msg MyMsg.msg
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_runtime)

install(PROGRAMS
   scripts/nmea_serial_driver
   scripts/nmea_topic_driver
   scripts/nmea_topic_serial_reader
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

# Check package for pep8 style, add a test to fail on violations.
roslint_python()
roslint_add_test()



add_executable(GPS_test
  src/GPS_test.cpp
)
add_dependencies(GPS_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(GPS_test
  ${catkin_LIBRARIES}
)

add_executable(GPS_Subscriber
  src/GPS_Subscriber.cpp
)
add_dependencies(GPS_Subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(GPS_Subscriber
  ${catkin_LIBRARIES}
)