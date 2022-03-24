# Install script for directory: /home/bubble/kubo_ws/src/control_box_rst/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/control_box_rst" TYPE FILE FILES "/home/bubble/kubo_ws/src/control_box_rst/src/cmake/control_box_rstConfig.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/core/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/communication/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/systems/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/plants/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/controllers/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/observers/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/numerics/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/optimization/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/optimal_control/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/tasks/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/master/cmake_install.cmake")
  include("/home/bubble/kubo_ws/src/control_box_rst/build/src/gui/cmake_install.cmake")

endif()

