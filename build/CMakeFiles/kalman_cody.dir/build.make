# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/cody/codygittest/Kalman_Cody

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cody/codygittest/Kalman_Cody/build

# Include any dependencies generated for this target.
include CMakeFiles/kalman_cody.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/kalman_cody.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kalman_cody.dir/flags.make

CMakeFiles/kalman_cody.dir/FusionEKF.o: CMakeFiles/kalman_cody.dir/flags.make
CMakeFiles/kalman_cody.dir/FusionEKF.o: ../FusionEKF.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cody/codygittest/Kalman_Cody/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/kalman_cody.dir/FusionEKF.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kalman_cody.dir/FusionEKF.o -c /home/cody/codygittest/Kalman_Cody/FusionEKF.cpp

CMakeFiles/kalman_cody.dir/FusionEKF.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kalman_cody.dir/FusionEKF.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cody/codygittest/Kalman_Cody/FusionEKF.cpp > CMakeFiles/kalman_cody.dir/FusionEKF.i

CMakeFiles/kalman_cody.dir/FusionEKF.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kalman_cody.dir/FusionEKF.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cody/codygittest/Kalman_Cody/FusionEKF.cpp -o CMakeFiles/kalman_cody.dir/FusionEKF.s

CMakeFiles/kalman_cody.dir/FusionEKF.o.requires:

.PHONY : CMakeFiles/kalman_cody.dir/FusionEKF.o.requires

CMakeFiles/kalman_cody.dir/FusionEKF.o.provides: CMakeFiles/kalman_cody.dir/FusionEKF.o.requires
	$(MAKE) -f CMakeFiles/kalman_cody.dir/build.make CMakeFiles/kalman_cody.dir/FusionEKF.o.provides.build
.PHONY : CMakeFiles/kalman_cody.dir/FusionEKF.o.provides

CMakeFiles/kalman_cody.dir/FusionEKF.o.provides.build: CMakeFiles/kalman_cody.dir/FusionEKF.o


CMakeFiles/kalman_cody.dir/kalman_filter.o: CMakeFiles/kalman_cody.dir/flags.make
CMakeFiles/kalman_cody.dir/kalman_filter.o: ../kalman_filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cody/codygittest/Kalman_Cody/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/kalman_cody.dir/kalman_filter.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kalman_cody.dir/kalman_filter.o -c /home/cody/codygittest/Kalman_Cody/kalman_filter.cpp

CMakeFiles/kalman_cody.dir/kalman_filter.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kalman_cody.dir/kalman_filter.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cody/codygittest/Kalman_Cody/kalman_filter.cpp > CMakeFiles/kalman_cody.dir/kalman_filter.i

CMakeFiles/kalman_cody.dir/kalman_filter.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kalman_cody.dir/kalman_filter.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cody/codygittest/Kalman_Cody/kalman_filter.cpp -o CMakeFiles/kalman_cody.dir/kalman_filter.s

CMakeFiles/kalman_cody.dir/kalman_filter.o.requires:

.PHONY : CMakeFiles/kalman_cody.dir/kalman_filter.o.requires

CMakeFiles/kalman_cody.dir/kalman_filter.o.provides: CMakeFiles/kalman_cody.dir/kalman_filter.o.requires
	$(MAKE) -f CMakeFiles/kalman_cody.dir/build.make CMakeFiles/kalman_cody.dir/kalman_filter.o.provides.build
.PHONY : CMakeFiles/kalman_cody.dir/kalman_filter.o.provides

CMakeFiles/kalman_cody.dir/kalman_filter.o.provides.build: CMakeFiles/kalman_cody.dir/kalman_filter.o


CMakeFiles/kalman_cody.dir/main.o: CMakeFiles/kalman_cody.dir/flags.make
CMakeFiles/kalman_cody.dir/main.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cody/codygittest/Kalman_Cody/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/kalman_cody.dir/main.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kalman_cody.dir/main.o -c /home/cody/codygittest/Kalman_Cody/main.cpp

CMakeFiles/kalman_cody.dir/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kalman_cody.dir/main.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cody/codygittest/Kalman_Cody/main.cpp > CMakeFiles/kalman_cody.dir/main.i

CMakeFiles/kalman_cody.dir/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kalman_cody.dir/main.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cody/codygittest/Kalman_Cody/main.cpp -o CMakeFiles/kalman_cody.dir/main.s

CMakeFiles/kalman_cody.dir/main.o.requires:

.PHONY : CMakeFiles/kalman_cody.dir/main.o.requires

CMakeFiles/kalman_cody.dir/main.o.provides: CMakeFiles/kalman_cody.dir/main.o.requires
	$(MAKE) -f CMakeFiles/kalman_cody.dir/build.make CMakeFiles/kalman_cody.dir/main.o.provides.build
.PHONY : CMakeFiles/kalman_cody.dir/main.o.provides

CMakeFiles/kalman_cody.dir/main.o.provides.build: CMakeFiles/kalman_cody.dir/main.o


CMakeFiles/kalman_cody.dir/tools.o: CMakeFiles/kalman_cody.dir/flags.make
CMakeFiles/kalman_cody.dir/tools.o: ../tools.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cody/codygittest/Kalman_Cody/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/kalman_cody.dir/tools.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kalman_cody.dir/tools.o -c /home/cody/codygittest/Kalman_Cody/tools.cpp

CMakeFiles/kalman_cody.dir/tools.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kalman_cody.dir/tools.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cody/codygittest/Kalman_Cody/tools.cpp > CMakeFiles/kalman_cody.dir/tools.i

CMakeFiles/kalman_cody.dir/tools.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kalman_cody.dir/tools.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cody/codygittest/Kalman_Cody/tools.cpp -o CMakeFiles/kalman_cody.dir/tools.s

CMakeFiles/kalman_cody.dir/tools.o.requires:

.PHONY : CMakeFiles/kalman_cody.dir/tools.o.requires

CMakeFiles/kalman_cody.dir/tools.o.provides: CMakeFiles/kalman_cody.dir/tools.o.requires
	$(MAKE) -f CMakeFiles/kalman_cody.dir/build.make CMakeFiles/kalman_cody.dir/tools.o.provides.build
.PHONY : CMakeFiles/kalman_cody.dir/tools.o.provides

CMakeFiles/kalman_cody.dir/tools.o.provides.build: CMakeFiles/kalman_cody.dir/tools.o


# Object files for target kalman_cody
kalman_cody_OBJECTS = \
"CMakeFiles/kalman_cody.dir/FusionEKF.o" \
"CMakeFiles/kalman_cody.dir/kalman_filter.o" \
"CMakeFiles/kalman_cody.dir/main.o" \
"CMakeFiles/kalman_cody.dir/tools.o"

# External object files for target kalman_cody
kalman_cody_EXTERNAL_OBJECTS =

kalman_cody: CMakeFiles/kalman_cody.dir/FusionEKF.o
kalman_cody: CMakeFiles/kalman_cody.dir/kalman_filter.o
kalman_cody: CMakeFiles/kalman_cody.dir/main.o
kalman_cody: CMakeFiles/kalman_cody.dir/tools.o
kalman_cody: CMakeFiles/kalman_cody.dir/build.make
kalman_cody: CMakeFiles/kalman_cody.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cody/codygittest/Kalman_Cody/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable kalman_cody"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kalman_cody.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kalman_cody.dir/build: kalman_cody

.PHONY : CMakeFiles/kalman_cody.dir/build

CMakeFiles/kalman_cody.dir/requires: CMakeFiles/kalman_cody.dir/FusionEKF.o.requires
CMakeFiles/kalman_cody.dir/requires: CMakeFiles/kalman_cody.dir/kalman_filter.o.requires
CMakeFiles/kalman_cody.dir/requires: CMakeFiles/kalman_cody.dir/main.o.requires
CMakeFiles/kalman_cody.dir/requires: CMakeFiles/kalman_cody.dir/tools.o.requires

.PHONY : CMakeFiles/kalman_cody.dir/requires

CMakeFiles/kalman_cody.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kalman_cody.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kalman_cody.dir/clean

CMakeFiles/kalman_cody.dir/depend:
	cd /home/cody/codygittest/Kalman_Cody/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cody/codygittest/Kalman_Cody /home/cody/codygittest/Kalman_Cody /home/cody/codygittest/Kalman_Cody/build /home/cody/codygittest/Kalman_Cody/build /home/cody/codygittest/Kalman_Cody/build/CMakeFiles/kalman_cody.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kalman_cody.dir/depend
