cmake_minimum_required(VERSION 3.1.0)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(RPLIDAR_RaspberryPi_Library LANGUAGES CXX)
include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 11)

find_package (Threads)

include_directories(rplidarsdk/include rplidarsdk/src)

set(LIBSRC
  a1lidarrpi.cpp
  rplidarsdk/src/rplidar_driver.cpp
  rplidarsdk/src/arch/linux/net_socket.cpp
  rplidarsdk/src/arch/linux/timer.cpp
  rplidarsdk/src/arch/linux/net_serial.cpp
  rplidarsdk/src/hal/thread.cpp
  )

set(SDKINCLUDE
  rplidarsdk/src/arch/linux/arch_linux.h
  rplidarsdk/src/arch/linux/timer.h
  rplidarsdk/src/arch/linux/net_serial.h
  rplidarsdk/src/sdkcommon.h
  rplidarsdk/src/hal/byteops.h
  rplidarsdk/src/hal/locker.h
  rplidarsdk/src/hal/assert.h
  rplidarsdk/src/hal/types.h
  rplidarsdk/src/hal/util.h
  rplidarsdk/src/hal/event.h
  rplidarsdk/src/hal/thread.h
  rplidarsdk/src/hal/abs_rxtx.h
  rplidarsdk/src/hal/socket.h
  rplidarsdk/src/rplidar_driver_TCP.h
  rplidarsdk/src/rplidar_driver_serial.h
  rplidarsdk/include/rplidar_cmd.h
  rplidarsdk/include/rplidar_driver.h
  rplidarsdk/include/rplidar.h
  rplidarsdk/include/rptypes.h
  rplidarsdk/include/rplidar_protocol.h
  )

add_library(a1lidarrpi
  STATIC
  ${LIBSRC}
  )

target_include_directories(a1lidarrpi
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>      # for a1lidarrpi.h
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rplidarsdk>  # everything else
)

set_target_properties(a1lidarrpi PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  PUBLIC_HEADER a1lidarrpi.h
  PRIVATE_HEADER "${SDKINCLUDE}")

target_link_libraries(a1lidarrpi ${CMAKE_THREAD_LIBS_INIT} pigpio rt)

install(TARGETS a1lidarrpi EXPORT a1lidarrpi_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rplidarsdk
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

install(EXPORT a1lidarrpi_targets
  DESTINATION lib/cmake/a1lidarrpi
  FILE a1lidarrpi-config.cmake
)

add_executable (printdata printdata.cpp)
target_link_libraries(printdata a1lidarrpi)

add_executable (printRPM printRPM.cpp)
target_link_libraries(printRPM a1lidarrpi)

add_executable (pwm pwm.cpp)
target_link_libraries(pwm pigpio rt ${CMAKE_THREAD_LIBS_INIT})
