# Install script for directory: /home/tim/wizzy_git/wizzy/simulation_ws/src/wizzybug_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/tim/wizzy_git/wizzy/simulation_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wizzybug_msgs/msg" TYPE FILE FILES
    "/home/tim/wizzy_git/wizzy/simulation_ws/src/wizzybug_msgs/msg/lidar_data.msg"
    "/home/tim/wizzy_git/wizzy/simulation_ws/src/wizzybug_msgs/msg/ttc.msg"
    "/home/tim/wizzy_git/wizzy/simulation_ws/src/wizzybug_msgs/msg/obstacle.msg"
    "/home/tim/wizzy_git/wizzy/simulation_ws/src/wizzybug_msgs/msg/obstacleArray.msg"
    "/home/tim/wizzy_git/wizzy/simulation_ws/src/wizzybug_msgs/msg/ChairState.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wizzybug_msgs/cmake" TYPE FILE FILES "/home/tim/wizzy_git/wizzy/simulation_ws/build/wizzybug_msgs/catkin_generated/installspace/wizzybug_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/tim/wizzy_git/wizzy/simulation_ws/devel/include/wizzybug_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/tim/wizzy_git/wizzy/simulation_ws/devel/share/roseus/ros/wizzybug_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/tim/wizzy_git/wizzy/simulation_ws/devel/share/common-lisp/ros/wizzybug_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/tim/wizzy_git/wizzy/simulation_ws/devel/share/gennodejs/ros/wizzybug_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/tim/wizzy_git/wizzy/simulation_ws/devel/lib/python2.7/dist-packages/wizzybug_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/tim/wizzy_git/wizzy/simulation_ws/devel/lib/python2.7/dist-packages/wizzybug_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tim/wizzy_git/wizzy/simulation_ws/build/wizzybug_msgs/catkin_generated/installspace/wizzybug_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wizzybug_msgs/cmake" TYPE FILE FILES "/home/tim/wizzy_git/wizzy/simulation_ws/build/wizzybug_msgs/catkin_generated/installspace/wizzybug_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wizzybug_msgs/cmake" TYPE FILE FILES
    "/home/tim/wizzy_git/wizzy/simulation_ws/build/wizzybug_msgs/catkin_generated/installspace/wizzybug_msgsConfig.cmake"
    "/home/tim/wizzy_git/wizzy/simulation_ws/build/wizzybug_msgs/catkin_generated/installspace/wizzybug_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wizzybug_msgs" TYPE FILE FILES "/home/tim/wizzy_git/wizzy/simulation_ws/src/wizzybug_msgs/package.xml")
endif()

