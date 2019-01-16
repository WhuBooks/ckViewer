cmake_minimum_required(VERSION 3.5)
project(ckViewer)

set(CMAKE_CXX_STANDARD 11)

# output paths settings
set(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")

# work out the postfix; required where we use OUTPUT_NAME
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(EXE_POSTFIX)
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
    set(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(WIN32)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    MESSAGE(STATUS "Compiling for Windows")
elseif(ANDROID)
    MESSAGE(STATUS "Compiling for Android")
elseif(APPLE)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    MESSAGE(STATUS "Compiling for OSX")
elseif(UNIX)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-pthread")

    find_package(PCL REQUIRED HINTS /usr/lib/x86_64-linux-gnu/cmake)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    link_libraries(${PCL_LIBRARIES})

    MESSAGE(STATUS "Compiling for Unix")
else()
    MESSAGE(STATUS "Platform can't be known")
endif(WIN32)

MESSAGE(STATUS "Executable Output Path : ${EXECUTABLE_OUTPUT_PATH}")
MESSAGE(STATUS "Library Output Path : ${LIBRARY_OUTPUT_PATH}")

if(WIN32)
    set(LCM_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ThirdParty/LCM)
    set(LCM_LIB ${PROJECT_SOURCE_DIR}/ThirdParty/LCM/lcm.lib)
else()
    set(LCM_INCLUDE_DIR /usr/local/include)
    set(LCM_LIB /usr/local/lib/liblcm.so)
endif(WIN32)


option(UseTest "Compile Test" OFF)
MESSAGE(STATUS "Use Test or not : ${UseTest}")

include_directories(ThirdParty)
include_directories(${LCM_INCLUDE_DIR})
include_directories(src)
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(src)
add_subdirectory(app)
if(UseTest)
    add_subdirectory(test)
endif(UseTest)
