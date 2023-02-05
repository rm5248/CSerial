#
#   Copyright 2016-2023 Robert Middleton
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
CMAKE_MINIMUM_REQUIRED(VERSION 3.3)

PROJECT(cserial VERSION 0.3.0)

INCLUDE(CheckIncludeFiles)
INCLUDE(GNUInstallDirs)

#
# CMake variables
#

#see if we are on Linux and have RS485
CHECK_INCLUDE_FILES(linux/serial.h HAVE_LINUX_SERIAL)

# Platform Defines, append to this to set various defines for the code
SET( PLATFORM_DEFINES )
SET( install_targets )

#
# library version
#

ADD_DEFINITIONS( "-DCSERIAL_VERSION=${cserial_VERSION}" )

#
# Sources for the library
#
SET( C_SERIAL_SOURCES
	c_serial.c )
SET( C_SERIAL_HEADERS 
	c_serial.h
        simplelogger_defs.h )
SET( C_SERIAL_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR} )
INCLUDE_DIRECTORIES( ${C_SERIAL_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_BINARY_DIR} )

CONFIGURE_FILE(config.h.in config.h)

#
# Library information
#
ADD_LIBRARY( cserial SHARED ${C_SERIAL_SOURCES} )
set_target_properties(cserial PROPERTIES VERSION ${cserial_VERSION} SOVERSION ${cserial_VERSION_MAJOR} )

option(CSERIAL_ENABLE_STATIC "Enable static library for CSerial" OFF)
if(CSERIAL_ENABLE_STATIC)
	ADD_LIBRARY( cserial_static STATIC ${C_SERIAL_SOURCES} )
	SET(install_targets ${install_targets} cserial_static )
endif()

#
# Install Information
#
SET(install_targets ${install_targets} cserial )
IF( UNIX )
    INSTALL( TARGETS ${install_targets}
             EXPORT cserialTargets
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
             ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" )
    INSTALL( FILES ${C_SERIAL_HEADERS} DESTINATION include/cserial )
ENDIF( UNIX )
IF( WIN32 )
    INSTALL( TARGETS ${install_targets}
             EXPORT cserialTargets
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib )
    INSTALL( FILES ${C_SERIAL_HEADERS} DESTINATION include/cserial )
ENDIF( WIN32 )

IF( UNIX )
    #
    # pkg-config information
    #
    SET(PKG_CONFIG_LIBDIR
        "\${prefix}/lib"
    )
    SET(PKG_CONFIG_INCLUDEDIR
        "\${prefix}/include"
    )
    SET(PKG_CONFIG_LIBS
        "-L\${libdir} -lcserial"
    )
    SET(PKG_CONFIG_CFLAGS
        "-I\${includedir}"
    )

    CONFIGURE_FILE(
        "${CMAKE_CURRENT_SOURCE_DIR}/cserial.pc.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
    )
    INSTALL( FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
             DESTINATION lib/pkgconfig)
ENDIF( UNIX )

#
# Enable all warnings
#
if(MSVC)
  # Force to always compile with W4
  if(CMAKE_C_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}")
  else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W4")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # Update if necessary
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-long-long -pedantic")
endif()

#
# platform-specific defines
#
IF( WIN32 )
	ADD_DEFINITIONS( "-DCSERIAL_LIB" )
	target_compile_definitions( cserial_static PUBLIC CSERIAL_STATIC )
ENDIF( WIN32 )

#
# Support for find_package
#
install(EXPORT cserialTargets
  FILE        cserialConfig.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cserial
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/cserialConfigVersion.cmake"
  VERSION       ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES   "${CMAKE_CURRENT_BINARY_DIR}/cserialConfigVersion.cmake"
  DESTINATION   ${CMAKE_INSTALL_LIBDIR}/cmake/cserial
)
