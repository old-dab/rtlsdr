# Copyright 2012 OSMOCOM Project
#
# This file is part of rtl-sdr
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
LIST(APPEND rtlsdr_srcs
    librtlsdr.c 
    tuner_e4k.c 
    tuner_fc001x.c 
    tuner_fc2580.c 
    tuner_r82xx.c
)

########################################################################
# Setup shared library variant
########################################################################
add_library(rtlsdr_shared SHARED ${rtlsdr_srcs})
target_link_libraries(rtlsdr_shared ${LIBUSB_LIBRARIES} -Wl,-s)
if(WIN32)
target_link_libraries(rtlsdr_shared -lsetupapi -lwinusb ${CMAKE_THREAD_LIBS_INIT})
endif()
set_target_properties(rtlsdr_shared PROPERTIES DEFINE_SYMBOL "rtlsdr_EXPORTS")
set_target_properties(rtlsdr_shared PROPERTIES OUTPUT_NAME rtlsdr)

########################################################################
# Setup static library variant
########################################################################
add_library(rtlsdr_static STATIC ${rtlsdr_srcs})
target_link_libraries(rtlsdr_static ${LIBUSB_LIBRARIES} -s)
if(WIN32)
target_link_libraries(rtlsdr_static -lsetupapi -lwinusb ${CMAKE_THREAD_LIBS_INIT})
endif()
set_property(TARGET rtlsdr_static APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
if(NOT WIN32)
# Force same library filename for static and shared variants of the library
set_target_properties(rtlsdr_static PROPERTIES OUTPUT_NAME rtlsdr)
endif()

########################################################################
# Set up Windows DLL resource files
########################################################################
if(WIN32)
target_sources(rtlsdr_shared PRIVATE rtlsdr.rc)
endif()

########################################################################
# Set link library
########################################################################
if(LINK_RTLTOOLS_AGAINST_STATIC_LIB)
  set(RTLSDR_TOOL_LIB rtlsdr_static)
else()
  set(RTLSDR_TOOL_LIB rtlsdr_shared)
endif()

########################################################################
# Setup libraries used in executables
########################################################################
add_library(convenience_static STATIC
    convenience/convenience.c
)

if(MSVC)
add_library(libgetopt_static STATIC
    getopt/getopt.c
)
endif()
target_link_libraries(convenience_static
    ${RTLSDR_TOOL_LIB}
)

########################################################################
# Build utility
########################################################################
add_executable(rtl_sdr rtl_sdr.c convenience/wavewrite.c)
add_executable(rtl_tcp rtl_tcp.c controlThread.c)
add_executable(rtl2_tcp rtl2_tcp.c controlThread2.c)
add_executable(rtl_udp rtl_udp.c)
add_executable(rtl_test rtl_test.c)
add_executable(rtl_fm rtl_fm.c convenience/wavewrite.c)
add_executable(rtl_ir rtl_ir.c)
add_executable(rtl_eeprom rtl_eeprom.c)
add_executable(rtl_adsb rtl_adsb.c)
add_executable(rtl_power rtl_power.c)
add_executable(rtl_file rtl_file.c)
set(INSTALL_TARGETS rtlsdr_shared rtlsdr_static 
	rtl_sdr rtl_tcp rtl2_tcp rtl_udp rtl_test rtl_fm 
	rtl_ir rtl_eeprom rtl_adsb rtl_power rtl_file
)

target_link_libraries(rtl_sdr ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_tcp ${RTLSDR_TOOL_LIB} convenience_static
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl2_tcp ${RTLSDR_TOOL_LIB} convenience_static
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_test ${RTLSDR_TOOL_LIB} convenience_static
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_fm ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_ir ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_eeprom ${RTLSDR_TOOL_LIB} convenience_static
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_adsb ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_udp ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_power ${RTLSDR_TOOL_LIB} convenience_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries(rtl_file ${RTLSDR_TOOL_LIB} 
    ${CMAKE_THREAD_LIBS_INIT}
)

if(UNIX)
target_link_libraries(rtl_fm m)
target_link_libraries(rtl_ir m)
target_link_libraries(rtl_adsb m)
target_link_libraries(rtl_power m)
target_link_libraries(rtl_tcp m)
target_link_libraries(rtl2_tcp m)
target_link_libraries(rtl_test m)
endif()

if(WIN32)
if(MSVC)
target_link_libraries(rtl_sdr libgetopt_static)
target_link_libraries(rtl_tcp ws2_32 libgetopt_static)
target_link_libraries(rtl2_tcp ws2_32 libgetopt_static)
target_link_libraries(rtl_udp ws2_32 libgetopt_static)
target_link_libraries(rtl_test libgetopt_static)
target_link_libraries(rtl_fm libgetopt_static)
target_link_libraries(rtl_ir libgetopt_static)
target_link_libraries(rtl_eeprom libgetopt_static)
target_link_libraries(rtl_adsb libgetopt_static)
target_link_libraries(rtl_power libgetopt_static)
else()
target_link_libraries(rtl_tcp ws2_32)
target_link_libraries(rtl2_tcp ws2_32)
target_link_libraries(rtl_udp ws2_32)
target_link_libraries(rtl_test)
target_link_libraries(rtl_file ws2_32)
endif()
set_property(TARGET rtl_sdr APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_tcp APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl2_tcp APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_udp APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_test APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_fm APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_ir APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_eeprom APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_adsb APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_power APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
set_property(TARGET rtl_file APPEND PROPERTY COMPILE_DEFINITIONS "rtlsdr_STATIC" )
endif()

########################################################################
# Install built library files & utilities
########################################################################
install(TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION ${LIB_INSTALL_DIR} # .so/.dylib file
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)
