# Copyright 2012-2020 Osmocom Project
#
# This file is part of rtl-sdr
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 3.0)
project(rtlsdr C)

# CMP0075 Include file check macros honor CMAKE_REQUIRED_LIBRARIES
if(POLICY CMP0075)
    cmake_policy(SET CMP0075 NEW)
endif()

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)

if(NOT LIB_INSTALL_DIR)
   set(LIB_INSTALL_DIR lib)
endif()

OPTION(LINK_RTLTOOLS_AGAINST_STATIC_LIB "Link rtl-tools statically against librtlsdr" OFF)

########################################################################
# Compiler specific setup
########################################################################
if(CMAKE_COMPILER_IS_GNUCC)
    add_compile_options(-Wall)
    add_compile_options(-Wextra)
    add_compile_options(-Wsign-compare)
    add_compile_options(-Wdeclaration-after-statement)
    #http://gcc.gnu.org/wiki/Visibility
    add_compile_options(-fvisibility=hidden)
elseif(MSVC14)
    #pthread-w32 issue, timespec is now part of time.h
    ADD_DEFINITIONS(-D_TIMESPEC_DEFINED)
endif()

if(MINGW)
    # Fix printf %zu
    ADD_DEFINITIONS(-D__USE_MINGW_ANSI_STDIO) 
endif()
#ADD_DEFINITIONS(-DDEBUG) 

########################################################################
# Find build dependencies
########################################################################
if(NOT WIN32)
find_package(PkgConfig)
find_package(LibUSB)
if(NOT LIBUSB_FOUND)
    message(FATAL_ERROR "LibUSB 1.0 required to compile rtl-sdr")
endif()
endif()

if(WIN32 AND NOT MINGW)
    set(THREADS_USE_PTHREADS_WIN32 true)
endif()
find_package(Threads REQUIRED)

if(NOT THREADS_FOUND)
    message(FATAL_ERROR "pthreads(-win32) required to compile rtl-sdr")
endif()

########################################################################
# Setup the include and linker paths
########################################################################
include_directories(
    ${PROJECT_SOURCE_DIR}/include
if(NOT WIN32)
    ${LIBUSB_INCLUDE_DIR}
endif()
    ${THREADS_PTHREADS_INCLUDE_DIR}
)

########################################################################
# Create uninstall target
########################################################################
configure_file(
    ${PROJECT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)

########################################################################
# Install udev rules
########################################################################
option(INSTALL_UDEV_RULES "Install udev rules for RTL-SDR" OFF)
if (INSTALL_UDEV_RULES)
    install (
        FILES rtl-sdr.rules
        DESTINATION "/etc/udev/rules.d"
        COMPONENT "udev"
        )
else (INSTALL_UDEV_RULES)
    message (STATUS "Udev rules not being installed, install them with -DINSTALL_UDEV_RULES=ON")
endif (INSTALL_UDEV_RULES)

option(DETACH_KERNEL_DRIVER "Detach kernel driver if loaded" OFF)
if (DETACH_KERNEL_DRIVER)
    message (STATUS "Building with kernel driver detaching enabled")
    add_definitions(-DDETACH_KERNEL_DRIVER=1)
else (DETACH_KERNEL_DRIVER)
    message (STATUS "Building with kernel driver detaching disabled, use -DDETACH_KERNEL_DRIVER=ON to enable")
endif (DETACH_KERNEL_DRIVER)

option(ENABLE_ZEROCOPY "Enable usbfs zero-copy support" OFF)
if (ENABLE_ZEROCOPY)
    message (STATUS "Building with usbfs zero-copy support enabled")
    add_definitions(-DENABLE_ZEROCOPY=1)
else (ENABLE_ZEROCOPY)
    message (STATUS "Building with usbfs zero-copy support disabled, use -DENABLE_ZEROCOPY=ON to enable")
endif (ENABLE_ZEROCOPY)

########################################################################
# Install public header files
########################################################################
install(FILES
    include/rtl-sdr.h
    include/rtl-sdr_export.h
    DESTINATION include
)
########################################################################
# Add subdirectories
########################################################################
add_subdirectory(src)

########################################################################
# Create Pkg Config File
########################################################################
FOREACH(inc ${LIBUSB_INCLUDE_DIR})
    LIST(APPEND RTLSDR_PC_CFLAGS "-I${inc}")
ENDFOREACH(inc)

FOREACH(lib ${LIBUSB_LIBRARY_DIRS})
    LIST(APPEND RTLSDR_PC_LIBS "-L${lib}")
ENDFOREACH(lib)

# use space-separation format for the pc file
STRING(REPLACE ";" " " RTLSDR_PC_CFLAGS "${RTLSDR_PC_CFLAGS}")
STRING(REPLACE ";" " " RTLSDR_PC_LIBS "${RTLSDR_PC_LIBS}")

# unset these vars to avoid hard-coded paths to cross environment
IF(CMAKE_CROSSCOMPILING)
    UNSET(RTLSDR_PC_CFLAGS)
    UNSET(RTLSDR_PC_LIBS)
ENDIF(CMAKE_CROSSCOMPILING)

set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix \${prefix})
set(libdir \${exec_prefix}/${LIB_INSTALL_DIR})
set(includedir \${prefix}/include)

CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/librtlsdr.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/librtlsdr.pc
@ONLY)

INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/librtlsdr.pc
    DESTINATION ${LIB_INSTALL_DIR}/pkgconfig
)

########################################################################
# Print Summary
########################################################################
MESSAGE(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
