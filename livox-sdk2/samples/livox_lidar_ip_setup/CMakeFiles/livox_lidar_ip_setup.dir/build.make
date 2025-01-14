# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /workspace/freshconsulting-livox-sdk2-eb36a4ce7115

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /workspace/freshconsulting-livox-sdk2-eb36a4ce7115

# Include any dependencies generated for this target.
include samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/depend.make

# Include the progress variables for this target.
include samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/progress.make

# Include the compile flags for this target's objects.
include samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/flags.make

samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.o: samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/flags.make
samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.o: samples/livox_lidar_ip_setup/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/workspace/freshconsulting-livox-sdk2-eb36a4ce7115/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.o"
	cd /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.o -c /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup/main.cpp

samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.i"
	cd /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup/main.cpp > CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.i

samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.s"
	cd /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup/main.cpp -o CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.s

# Object files for target livox_lidar_ip_setup
livox_lidar_ip_setup_OBJECTS = \
"CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.o"

# External object files for target livox_lidar_ip_setup
livox_lidar_ip_setup_EXTERNAL_OBJECTS =

samples/livox_lidar_ip_setup/livox_lidar_ip_setup: samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/main.cpp.o
samples/livox_lidar_ip_setup/livox_lidar_ip_setup: samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/build.make
samples/livox_lidar_ip_setup/livox_lidar_ip_setup: sdk_core/liblivox_lidar_sdk_static.a
samples/livox_lidar_ip_setup/livox_lidar_ip_setup: samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/workspace/freshconsulting-livox-sdk2-eb36a4ce7115/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable livox_lidar_ip_setup"
	cd /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/livox_lidar_ip_setup.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/build: samples/livox_lidar_ip_setup/livox_lidar_ip_setup

.PHONY : samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/build

samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/clean:
	cd /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup && $(CMAKE_COMMAND) -P CMakeFiles/livox_lidar_ip_setup.dir/cmake_clean.cmake
.PHONY : samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/clean

samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/depend:
	cd /workspace/freshconsulting-livox-sdk2-eb36a4ce7115 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /workspace/freshconsulting-livox-sdk2-eb36a4ce7115 /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup /workspace/freshconsulting-livox-sdk2-eb36a4ce7115 /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup /workspace/freshconsulting-livox-sdk2-eb36a4ce7115/samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : samples/livox_lidar_ip_setup/CMakeFiles/livox_lidar_ip_setup.dir/depend

