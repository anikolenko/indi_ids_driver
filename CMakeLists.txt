cmake_minimum_required(VERSION 3.16)
PROJECT(ids_ccd CXX C)

LIST(APPEND CMAKE_MODULE_PATH "./cmake_modules/")

include(GNUInstallDirs)

set (GENERIC_VERSION_MAJOR 1)
set (GENERIC_VERSION_MINOR 0)

find_package(CFITSIO REQUIRED)
find_package(INDI REQUIRED)
find_package(ZLIB REQUIRED)

# uEye SDK manual setup
set(UEYE_LIBRARY "ueye_api")

#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_ids_ccd.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_ids_ccd.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${CFITSIO_INCLUDE_DIR})

include(CMakeCommon)

############# GENERIC CCD ###############
set(idsccd_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/indi_ids_ccd.cpp
)

add_executable(indi_ids_ccd ${idsccd_SRCS})

target_link_libraries(indi_ids_ccd ${INDI_LIBRARIES} ${CFITSIO_LIBRARIES} ${UEYE_LIBRARY} m ${ZLIB_LIBRARY})

install(TARGETS indi_ids_ccd RUNTIME DESTINATION bin)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_ids_ccd.xml DESTINATION ${INDI_DATA_DIR})
