cmake_minimum_required(VERSION 3.1.0)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(RPLIDAR_RaspberryPi_Library LANGUAGES CXX)
include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 11)

# add_compile_options(-Wall -Wconversion -Wextra -pedantic)

find_package (Threads)

include_directories(sdk/sdk/include sdk/sdk/src/)

set(LIBSRC
  lidarrpi.cpp
  sdk/sdk/src/rplidar_driver.cpp
  sdk/sdk/src/arch/linux/net_socket.cpp
  sdk/sdk/src/arch/linux/timer.cpp
  sdk/sdk/src/arch/linux/net_serial.cpp
  sdk/sdk/src/hal/thread.cpp
  )
set(LIBINCLUDE lidarrpi.h)

add_library(lidarrpi
  STATIC
  ${LIBSRC}
  )

set_target_properties(lidarrpi PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  PUBLIC_HEADER "${LIBINCLUDE}")

target_link_libraries(lidarrpi ${CMAKE_THREAD_LIBS_INIT} pigpio rt)

install(TARGETS lidarrpi EXPORT lidarrpi_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

add_executable (printdata printdata.cpp)
target_link_libraries(printdata lidarrpi)

add_executable (printRPM printRPM.cpp)
target_link_libraries(printRPM lidarrpi)

add_executable (pwm pwm.cpp)
target_link_libraries(pwm pigpio rt ${CMAKE_THREAD_LIBS_INIT})
