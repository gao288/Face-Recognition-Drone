cmake_minimum_required(VERSION 3.10)

project(t)

find_package(Qt5Core REQUIRED) 
find_package(Qt5Widgets REQUIRED) 
#find_package(Qt5Multimedia REQUIRED)
#find_package(Qt5Multimediawidgets REQUIRED)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x")

set(SOURCES
    test.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(t ${SOURCES} )
qt5_use_modules(t Core Widgets)
