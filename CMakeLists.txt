#
#   Copyright 2016 rm5248
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
PROJECT(cserial)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#
# CMake variables
#

# Platform Defines, append to this to set various defines for the code
SET( PLATFORM_DEFINES )
SET( install_targets )

#
# library version
#
SET(CSERIAL_VERSION_MAJOR 0)
SET(CSERIAL_VERSION_MINOR 1)
SET(CSERIAL_VERSION_PATCH 0)
SET(CSERIAL_VERSION_STRING ${CSERIAL_VERSION_MAJOR}.${CSERIAL_VERSION_MINOR}.${CSERIAL_VERSION_PATCH})

ADD_DEFINITIONS( "-DCSERIAL_VERSION=${CSERIAL_VERSION_STRING}" )

#
# Sources for the library
#
SET( C_SERIAL_SOURCES
	c_serial.c )
SET( C_SERIAL_HEADERS 
	c_serial.h )
SET( C_SERIAL_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR} )
INCLUDE_DIRECTORIES( ${C_SERIAL_INCLUDE_DIRECTORIES} )

#
# Library information
#
ADD_LIBRARY( cserial SHARED ${C_SERIAL_SOURCES} )
ADD_LIBRARY( cserial_static STATIC ${C_SERIAL_SOURCES} )
set_target_properties(cserial PROPERTIES VERSION ${CSERIAL_VERSION_STRING} SOVERSION ${CSERIAL_VERSION_MAJOR} )

#
# Install Information
#
SET(install_targets ${install_targets} cserial cserial_static )
INSTALL( TARGETS ${install_targets}
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib )
INSTALL( FILES ${C_SERIAL_HEADERS} DESTINATION include/cserial )

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
    INSTALL( FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
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
