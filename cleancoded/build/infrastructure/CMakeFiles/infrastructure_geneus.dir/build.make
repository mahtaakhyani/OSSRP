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
CMAKE_SOURCE_DIR = /home/hooshang/Desktop/OSSRP/cleancoded/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hooshang/Desktop/OSSRP/cleancoded/build

# Utility rule file for infrastructure_geneus.

# Include the progress variables for this target.
include infrastructure/CMakeFiles/infrastructure_geneus.dir/progress.make

infrastructure_geneus: infrastructure/CMakeFiles/infrastructure_geneus.dir/build.make

.PHONY : infrastructure_geneus

# Rule to build all files generated by this target.
infrastructure/CMakeFiles/infrastructure_geneus.dir/build: infrastructure_geneus

.PHONY : infrastructure/CMakeFiles/infrastructure_geneus.dir/build

infrastructure/CMakeFiles/infrastructure_geneus.dir/clean:
	cd /home/hooshang/Desktop/OSSRP/cleancoded/build/infrastructure && $(CMAKE_COMMAND) -P CMakeFiles/infrastructure_geneus.dir/cmake_clean.cmake
.PHONY : infrastructure/CMakeFiles/infrastructure_geneus.dir/clean

infrastructure/CMakeFiles/infrastructure_geneus.dir/depend:
	cd /home/hooshang/Desktop/OSSRP/cleancoded/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hooshang/Desktop/OSSRP/cleancoded/src /home/hooshang/Desktop/OSSRP/cleancoded/src/infrastructure /home/hooshang/Desktop/OSSRP/cleancoded/build /home/hooshang/Desktop/OSSRP/cleancoded/build/infrastructure /home/hooshang/Desktop/OSSRP/cleancoded/build/infrastructure/CMakeFiles/infrastructure_geneus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : infrastructure/CMakeFiles/infrastructure_geneus.dir/depend

