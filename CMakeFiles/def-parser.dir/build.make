# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\fmaksimo\Documents\openmote\openwsn-fw

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\fmaksimo\Documents\openmote\openwsn-fw

# Utility rule file for def-parser.

# Include any custom commands dependencies for this target.
include CMakeFiles/def-parser.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/def-parser.dir/progress.make

CMakeFiles/def-parser:
	cd /d C:\Users\fmaksimo\Documents\openmote\openwsn-fw\cmake && C:\Users\fmaksimo\AppData\Local\Programs\Python\Python39\python.exe def_exporter.py

def-parser: CMakeFiles/def-parser
def-parser: CMakeFiles/def-parser.dir/build.make
.PHONY : def-parser

# Rule to build all files generated by this target.
CMakeFiles/def-parser.dir/build: def-parser
.PHONY : CMakeFiles/def-parser.dir/build

CMakeFiles/def-parser.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\def-parser.dir\cmake_clean.cmake
.PHONY : CMakeFiles/def-parser.dir/clean

CMakeFiles/def-parser.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\fmaksimo\Documents\openmote\openwsn-fw C:\Users\fmaksimo\Documents\openmote\openwsn-fw C:\Users\fmaksimo\Documents\openmote\openwsn-fw C:\Users\fmaksimo\Documents\openmote\openwsn-fw C:\Users\fmaksimo\Documents\openmote\openwsn-fw\CMakeFiles\def-parser.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/def-parser.dir/depend

