#=============================================================================
#
# CMake configuration file for the FRyDoM project
#
#=============================================================================
cmake_minimum_required(VERSION 3.13)

project(acme)
set(CMAKE_CXX_STANDARD 17)

#-----------------------------------------------------------------------------
# Prevent in-source builds
#-----------------------------------------------------------------------------
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source build is not possible and not recommended. Choose an empty directory for build output.")
endif (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

#-----------------------------------------------------------------------------
# FRyDoM options
#-----------------------------------------------------------------------------
cmake_policy(SET CMP0077 NEW)

option(BUILD_SHARED_LIBS "Shared lib version" OFF)

option(ACME_BUILD_TESTS "Activate build tests" ON)
option(ACME_BUILD_DEV_TESTS "Activate build tests" ON)
option(ACME_BUILD_UNIT_TESTS "Activate build tests" ON)

#=============================================================================
# Retrieving the current Git revision
#=============================================================================
set(PRE_CONFIGURE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/acme/version.cpp.in CACHE FILEPATH "" FORCE)
set(POST_CONFIGURE_FILE ${CMAKE_CURRENT_BINARY_DIR}/src/acme/version.cpp CACHE FILEPATH "" FORCE)
set(GIT_WORKING_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE FILEPATH "" FORCE)
set(GIT_STATE_FILE ${CMAKE_CURRENT_BINARY_DIR}/git-state-hash CACHE FILEPATH "" FORCE)
include(cmake/git_watcher.cmake)

if (NOT EXISTS ${POST_CONFIGURE_FILE})
    configure_file(${PRE_CONFIGURE_FILE} ${POST_CONFIGURE_FILE})
endif ()

# =============================================================================
# FRyDoM DEPENDENCIES
# =============================================================================
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/URL.conf.cmake)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(Add_mathutils)
include(Add_json)

#=============================================================================
# Adding the ACME project sources
#=============================================================================
add_subdirectory(src/acme)

#=============================================================================
# Adding tests
#=============================================================================
if (${ACME_BUILD_TESTS})
    enable_testing()
    include(Add_googletest)
    add_subdirectory(tests)
endif ()

#=============================================================================
# Adding tools
#=============================================================================
#if (${ACME_BUILD_TOOLS})
#    include(add_cxxopts)
#    add_subdirectory(tools)
#endif ()
