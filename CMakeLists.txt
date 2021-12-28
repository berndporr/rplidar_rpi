cmake_minimum_required(VERSION 3.1.0)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(RPLIDAR_A1_RaspberryPi_Library LANGUAGES CXX)
include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 11)

find_package (Threads)

include_directories(rplidarsdk rplidarsdk/src)

set(LIBSRC
  a1lidarrpi.cpp
  rplidarsdk/rplidar_driver.cpp
  rplidarsdk/arch/linux/net_socket.cpp
  rplidarsdk/arch/linux/timer.cpp
  rplidarsdk/arch/linux/net_serial.cpp
  rplidarsdk/hal/thread.cpp
  )

add_library(a1lidarrpi
  STATIC
  ${LIBSRC}
  )

set_target_properties(a1lidarrpi PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  PUBLIC_HEADER a1lidarrpi.h)

target_link_libraries(a1lidarrpi ${CMAKE_THREAD_LIBS_INIT} pigpio rt)

install(TARGETS a1lidarrpi EXPORT a1lidarrpi_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

install(DIRECTORY rplidarsdk/ DESTINATION include/rplidarsdk FILES_MATCHING PATTERN "*.h")

install(EXPORT a1lidarrpi_targets
  DESTINATION lib/cmake/A1LidarRPI
  FILE A1LidarRPIConfig.cmake
)

add_executable (printdata printdata.cpp)
target_link_libraries(printdata a1lidarrpi)

add_executable (printRPM printRPM.cpp)
target_link_libraries(printRPM a1lidarrpi)

add_executable (pwm pwm.cpp)
target_link_libraries(pwm pigpio rt ${CMAKE_THREAD_LIBS_INIT})
