# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build

# Include any dependencies generated for this target.
include CMakeFiles/camera.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camera.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera.dir/flags.make

ui_camera.h: ../camera.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ui_camera.h"
	/home/aclouditation/Qt/5.13.0/gcc_64/bin/uic -o /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/ui_camera.h /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/camera.ui

ui_imagesettings.h: ../imagesettings.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating ui_imagesettings.h"
	/home/aclouditation/Qt/5.13.0/gcc_64/bin/uic -o /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/ui_imagesettings.h /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/imagesettings.ui

ui_videosettings.h: ../videosettings.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating ui_videosettings.h"
	/home/aclouditation/Qt/5.13.0/gcc_64/bin/uic -o /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/ui_videosettings.h /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/videosettings.ui

qrc_camera.cpp: ../images/shutter.svg
qrc_camera.cpp: camera.qrc.depends
qrc_camera.cpp: ../camera.qrc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating qrc_camera.cpp"
	/home/aclouditation/Qt/5.13.0/gcc_64/bin/rcc --name camera --output /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/qrc_camera.cpp /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/camera.qrc

CMakeFiles/camera.dir/camera.cpp.o: CMakeFiles/camera.dir/flags.make
CMakeFiles/camera.dir/camera.cpp.o: ../camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/camera.dir/camera.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/camera.cpp.o -c /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/camera.cpp

CMakeFiles/camera.dir/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/camera.cpp > CMakeFiles/camera.dir/camera.cpp.i

CMakeFiles/camera.dir/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/camera.cpp -o CMakeFiles/camera.dir/camera.cpp.s

CMakeFiles/camera.dir/camera.cpp.o.requires:

.PHONY : CMakeFiles/camera.dir/camera.cpp.o.requires

CMakeFiles/camera.dir/camera.cpp.o.provides: CMakeFiles/camera.dir/camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/camera.dir/build.make CMakeFiles/camera.dir/camera.cpp.o.provides.build
.PHONY : CMakeFiles/camera.dir/camera.cpp.o.provides

CMakeFiles/camera.dir/camera.cpp.o.provides.build: CMakeFiles/camera.dir/camera.cpp.o


CMakeFiles/camera.dir/imagesettings.cpp.o: CMakeFiles/camera.dir/flags.make
CMakeFiles/camera.dir/imagesettings.cpp.o: ../imagesettings.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/camera.dir/imagesettings.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/imagesettings.cpp.o -c /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/imagesettings.cpp

CMakeFiles/camera.dir/imagesettings.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/imagesettings.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/imagesettings.cpp > CMakeFiles/camera.dir/imagesettings.cpp.i

CMakeFiles/camera.dir/imagesettings.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/imagesettings.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/imagesettings.cpp -o CMakeFiles/camera.dir/imagesettings.cpp.s

CMakeFiles/camera.dir/imagesettings.cpp.o.requires:

.PHONY : CMakeFiles/camera.dir/imagesettings.cpp.o.requires

CMakeFiles/camera.dir/imagesettings.cpp.o.provides: CMakeFiles/camera.dir/imagesettings.cpp.o.requires
	$(MAKE) -f CMakeFiles/camera.dir/build.make CMakeFiles/camera.dir/imagesettings.cpp.o.provides.build
.PHONY : CMakeFiles/camera.dir/imagesettings.cpp.o.provides

CMakeFiles/camera.dir/imagesettings.cpp.o.provides.build: CMakeFiles/camera.dir/imagesettings.cpp.o


CMakeFiles/camera.dir/main.cpp.o: CMakeFiles/camera.dir/flags.make
CMakeFiles/camera.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/camera.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/main.cpp.o -c /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/main.cpp

CMakeFiles/camera.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/main.cpp > CMakeFiles/camera.dir/main.cpp.i

CMakeFiles/camera.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/main.cpp -o CMakeFiles/camera.dir/main.cpp.s

CMakeFiles/camera.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/camera.dir/main.cpp.o.requires

CMakeFiles/camera.dir/main.cpp.o.provides: CMakeFiles/camera.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/camera.dir/build.make CMakeFiles/camera.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/camera.dir/main.cpp.o.provides

CMakeFiles/camera.dir/main.cpp.o.provides.build: CMakeFiles/camera.dir/main.cpp.o


CMakeFiles/camera.dir/videosettings.cpp.o: CMakeFiles/camera.dir/flags.make
CMakeFiles/camera.dir/videosettings.cpp.o: ../videosettings.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/camera.dir/videosettings.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/videosettings.cpp.o -c /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/videosettings.cpp

CMakeFiles/camera.dir/videosettings.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/videosettings.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/videosettings.cpp > CMakeFiles/camera.dir/videosettings.cpp.i

CMakeFiles/camera.dir/videosettings.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/videosettings.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/videosettings.cpp -o CMakeFiles/camera.dir/videosettings.cpp.s

CMakeFiles/camera.dir/videosettings.cpp.o.requires:

.PHONY : CMakeFiles/camera.dir/videosettings.cpp.o.requires

CMakeFiles/camera.dir/videosettings.cpp.o.provides: CMakeFiles/camera.dir/videosettings.cpp.o.requires
	$(MAKE) -f CMakeFiles/camera.dir/build.make CMakeFiles/camera.dir/videosettings.cpp.o.provides.build
.PHONY : CMakeFiles/camera.dir/videosettings.cpp.o.provides

CMakeFiles/camera.dir/videosettings.cpp.o.provides.build: CMakeFiles/camera.dir/videosettings.cpp.o


CMakeFiles/camera.dir/qrc_camera.cpp.o: CMakeFiles/camera.dir/flags.make
CMakeFiles/camera.dir/qrc_camera.cpp.o: qrc_camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/camera.dir/qrc_camera.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/qrc_camera.cpp.o -c /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/qrc_camera.cpp

CMakeFiles/camera.dir/qrc_camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/qrc_camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/qrc_camera.cpp > CMakeFiles/camera.dir/qrc_camera.cpp.i

CMakeFiles/camera.dir/qrc_camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/qrc_camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/qrc_camera.cpp -o CMakeFiles/camera.dir/qrc_camera.cpp.s

CMakeFiles/camera.dir/qrc_camera.cpp.o.requires:

.PHONY : CMakeFiles/camera.dir/qrc_camera.cpp.o.requires

CMakeFiles/camera.dir/qrc_camera.cpp.o.provides: CMakeFiles/camera.dir/qrc_camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/camera.dir/build.make CMakeFiles/camera.dir/qrc_camera.cpp.o.provides.build
.PHONY : CMakeFiles/camera.dir/qrc_camera.cpp.o.provides

CMakeFiles/camera.dir/qrc_camera.cpp.o.provides.build: CMakeFiles/camera.dir/qrc_camera.cpp.o


CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o: CMakeFiles/camera.dir/flags.make
CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o: camera_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o -c /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/camera_autogen/mocs_compilation.cpp

CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/camera_autogen/mocs_compilation.cpp > CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.i

CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/camera_autogen/mocs_compilation.cpp -o CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.s

CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o.requires:

.PHONY : CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o.requires

CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o.provides: CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o.requires
	$(MAKE) -f CMakeFiles/camera.dir/build.make CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o.provides.build
.PHONY : CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o.provides

CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o.provides.build: CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o


# Object files for target camera
camera_OBJECTS = \
"CMakeFiles/camera.dir/camera.cpp.o" \
"CMakeFiles/camera.dir/imagesettings.cpp.o" \
"CMakeFiles/camera.dir/main.cpp.o" \
"CMakeFiles/camera.dir/videosettings.cpp.o" \
"CMakeFiles/camera.dir/qrc_camera.cpp.o" \
"CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o"

# External object files for target camera
camera_EXTERNAL_OBJECTS =

camera: CMakeFiles/camera.dir/camera.cpp.o
camera: CMakeFiles/camera.dir/imagesettings.cpp.o
camera: CMakeFiles/camera.dir/main.cpp.o
camera: CMakeFiles/camera.dir/videosettings.cpp.o
camera: CMakeFiles/camera.dir/qrc_camera.cpp.o
camera: CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o
camera: CMakeFiles/camera.dir/build.make
camera: /home/aclouditation/Qt/5.13.0/gcc_64/lib/libQt5MultimediaWidgets.so.5.13.0
camera: /home/aclouditation/Qt/5.13.0/gcc_64/lib/libQt5Widgets.so.5.13.0
camera: /home/aclouditation/Qt/5.13.0/gcc_64/lib/libQt5Multimedia.so.5.13.0
camera: /home/aclouditation/Qt/5.13.0/gcc_64/lib/libQt5Gui.so.5.13.0
camera: /home/aclouditation/Qt/5.13.0/gcc_64/lib/libQt5Network.so.5.13.0
camera: /home/aclouditation/Qt/5.13.0/gcc_64/lib/libQt5Core.so.5.13.0
camera: CMakeFiles/camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX executable camera"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera.dir/build: camera

.PHONY : CMakeFiles/camera.dir/build

CMakeFiles/camera.dir/requires: CMakeFiles/camera.dir/camera.cpp.o.requires
CMakeFiles/camera.dir/requires: CMakeFiles/camera.dir/imagesettings.cpp.o.requires
CMakeFiles/camera.dir/requires: CMakeFiles/camera.dir/main.cpp.o.requires
CMakeFiles/camera.dir/requires: CMakeFiles/camera.dir/videosettings.cpp.o.requires
CMakeFiles/camera.dir/requires: CMakeFiles/camera.dir/qrc_camera.cpp.o.requires
CMakeFiles/camera.dir/requires: CMakeFiles/camera.dir/camera_autogen/mocs_compilation.cpp.o.requires

.PHONY : CMakeFiles/camera.dir/requires

CMakeFiles/camera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera.dir/clean

CMakeFiles/camera.dir/depend: ui_camera.h
CMakeFiles/camera.dir/depend: ui_imagesettings.h
CMakeFiles/camera.dir/depend: ui_videosettings.h
CMakeFiles/camera.dir/depend: qrc_camera.cpp
	cd /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build /home/aclouditation/COURSES/ECE477/Face-Recognition-Drone/camera/build/CMakeFiles/camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera.dir/depend

