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
CMAKE_SOURCE_DIR = /home/hooshang/OSSRP/cleancoded/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hooshang/OSSRP/cleancoded/build

# Utility rule file for infrastructure_gennodejs.

# Include the progress variables for this target.
include infrastructure/CMakeFiles/infrastructure_gennodejs.dir/progress.make

infrastructure_gennodejs: infrastructure/CMakeFiles/infrastructure_gennodejs.dir/build.make

.PHONY : infrastructure_gennodejs

# Rule to build all files generated by this target.
infrastructure/CMakeFiles/infrastructure_gennodejs.dir/build: infrastructure_gennodejs

.PHONY : infrastructure/CMakeFiles/infrastructure_gennodejs.dir/build

infrastructure/CMakeFiles/infrastructure_gennodejs.dir/clean:
	cd /home/hooshang/OSSRP/cleancoded/build/infrastructure && $(CMAKE_COMMAND) -P CMakeFiles/infrastructure_gennodejs.dir/cmake_clean.cmake
.PHONY : infrastructure/CMakeFiles/infrastructure_gennodejs.dir/clean

infrastructure/CMakeFiles/infrastructure_gennodejs.dir/depend:
	cd /home/hooshang/OSSRP/cleancoded/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hooshang/OSSRP/cleancoded/src /home/hooshang/OSSRP/cleancoded/src/infrastructure /home/hooshang/OSSRP/cleancoded/build /home/hooshang/OSSRP/cleancoded/build/infrastructure /home/hooshang/OSSRP/cleancoded/build/infrastructure/CMakeFiles/infrastructure_gennodejs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : infrastructure/CMakeFiles/infrastructure_gennodejs.dir/depend

