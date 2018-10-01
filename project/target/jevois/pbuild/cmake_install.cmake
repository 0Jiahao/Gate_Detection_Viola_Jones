# Install script for directory: /home/jiahao/dronerace2018/target/jevois

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/var/lib/jevois-build/usr")
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
  EXECUTE_PROCESS(COMMAND /bin/ls "/media/jiahao/JEVOIS/" )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xlibsx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/media/jiahao/JEVOIS/lib/MavLab/libmavpilotbase.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/media/jiahao/JEVOIS/lib/MavLab/libmavpilotbase.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/media/jiahao/JEVOIS/lib/MavLab/libmavpilotbase.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/jiahao/JEVOIS/lib/MavLab/libmavpilotbase.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/jiahao/JEVOIS/lib/MavLab" TYPE SHARED_LIBRARY FILES "/home/jiahao/dronerace2018/target/jevois/pbuild/libmavpilotbase.so")
  if(EXISTS "$ENV{DESTDIR}/media/jiahao/JEVOIS/lib/MavLab/libmavpilotbase.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/media/jiahao/JEVOIS/lib/MavLab/libmavpilotbase.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/media/jiahao/JEVOIS/lib/MavLab/libmavpilotbase.so"
         OLD_RPATH "/var/lib/jevois-build/usr/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/media/jiahao/JEVOIS/lib/MavLab/libmavpilotbase.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/jiahao/JEVOIS/modules/MavLab/CascadeDetector")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/jiahao/JEVOIS/modules/MavLab" TYPE DIRECTORY FILES "/home/jiahao/dronerace2018/target/jevois/src/Modules/CascadeDetector" REGEX "/[^/]*\\.[hHcC]$" EXCLUDE REGEX "/[^/]*\\.hpp$" EXCLUDE REGEX "/[^/]*\\.cpp$" EXCLUDE REGEX "/modinfo\\.[^/]*$" EXCLUDE REGEX "/[^/]*\\~$" EXCLUDE REGEX "/[^/]*ubyte$" EXCLUDE REGEX "/[^/]*\\.bin$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/media/jiahao/JEVOIS/modules/MavLab/CascadeDetector/CascadeDetector.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/media/jiahao/JEVOIS/modules/MavLab/CascadeDetector/CascadeDetector.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/media/jiahao/JEVOIS/modules/MavLab/CascadeDetector/CascadeDetector.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/jiahao/JEVOIS/modules/MavLab/CascadeDetector/CascadeDetector.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/jiahao/JEVOIS/modules/MavLab/CascadeDetector" TYPE SHARED_LIBRARY FILES "/home/jiahao/dronerace2018/target/jevois/pbuild/CascadeDetector.so")
  if(EXISTS "$ENV{DESTDIR}/media/jiahao/JEVOIS/modules/MavLab/CascadeDetector/CascadeDetector.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/media/jiahao/JEVOIS/modules/MavLab/CascadeDetector/CascadeDetector.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/media/jiahao/JEVOIS/modules/MavLab/CascadeDetector/CascadeDetector.so"
         OLD_RPATH "/var/lib/jevois-build/usr/lib:/home/jiahao/dronerace2018/target/jevois/pbuild:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/media/jiahao/JEVOIS/modules/MavLab/CascadeDetector/CascadeDetector.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xbinx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/jiahao/JEVOIS/share")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/jiahao/JEVOIS" TYPE DIRECTORY FILES "/home/jiahao/dronerace2018/target/jevois/share")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/jiahao/dronerace2018/target/jevois/pbuild/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
