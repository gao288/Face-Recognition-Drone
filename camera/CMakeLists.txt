cmake_minimum_required(VERSION 3.10)

project(camera)

#set(Qt5LOC "${HOME}/Qt/5.13.0/gcc_64")
#set(CMAKE_PREFIX_PATH Qt5LOC)

set(Qt5LOC "${HOME}/Qt/5.13.0/gcc_64/lib/cmake/")
# manually specify module location
set(Qt5Core_DIR                 "${Qt5LOC}/Qt5Core")
set(Qt5Widgets_DIR              "${Qt5LOC}/Qt5Widgets")
set(Qt5Gui_DIR                  "${Qt5LOC}/Qt5Gui")
set(Qt5Multimedia_DIR           "${Qt5LOC}/Qt5Multimedia")
set(Qt5MultimediaWidgets_DIR    "${Qt5LOC}/Qt5MultimediaWidgets")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5Core REQUIRED) 
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED) 
find_package(Qt5Multimedia REQUIRED) 
find_package(Qt5MultimediaWidgets REQUIRED) 

message(STATUS Qt5Core_dir: ${Qt5Core_DIR})
message(STATUS Qt5Version: ${Qt5Widgets_VERSION})
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")

set(src_dir .)
foreach(dir ${src_dir})
	string(TOUPPER "${dir}" dir_u)
	file(GLOB ${dir_u}_UI_FILES ${dir}/*.ui)
	file(GLOB ${dir_u}_H_FILES ${dir}/*.h)
	file(GLOB ${dir_u}_CXX_FILES ${dir}/*.cpp)
	list(APPEND UI_FILES ${${dir_u}_UI_FILES})
	list(APPEND ${PROJECT_NAME}_SRCS ${${dir_u}_H_FILES})
	list(APPEND HEADER_SRCS ${${dir_u}_CXX_FILES})
endforeach(dir ${src_dir})

#message(STATUS ${UI_FILES})
#message(STATUS ${${PROJECT_NAME}_SRCS})
#message(STATUS ${HEADER_SRCS})
qt5_wrap_ui( UIC_SRCS ${UI_FILES})

set(RESOURCE_FILES ${PROJECT_NAME}.qrc)
qt5_add_resources(RCC_SRCS ${RESOURCE_FILES}) 

include_directories(${CMAKE_BINARY_DIR})
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS}
				${HEADER_SRCS}
				${UIC_SRCS}
                ${RCC_SRCS})


target_link_libraries(${PROJECT_NAME}
                Qt5::Core
    			Qt5::Gui
    			Qt5::Widgets
                Qt5::Multimedia
                Qt5::MultimediaWidgets
				)

source_group("Form Files" FILES ${UI_FILES})
source_group("Generated Files" FILES ${UIC_SRCS} ${RCC_SRCS} )
source_group("Resources" FILES ${RESOURCE_FILES})
