# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/fetch/Documents/segmenter_jordlee

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fetch/Documents/segmenter_jordlee/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/segmenter_jordlee.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/segmenter_jordlee.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/segmenter_jordlee.dir/flags.make

CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o: CMakeFiles/segmenter_jordlee.dir/flags.make
CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o: ../src/segmenter_jordlee.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/fetch/Documents/segmenter_jordlee/cmake-build-debug/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o -c /home/fetch/Documents/segmenter_jordlee/src/segmenter_jordlee.cpp

CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/fetch/Documents/segmenter_jordlee/src/segmenter_jordlee.cpp > CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.i

CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/fetch/Documents/segmenter_jordlee/src/segmenter_jordlee.cpp -o CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.s

CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o.requires:
.PHONY : CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o.requires

CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o.provides: CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o.requires
	$(MAKE) -f CMakeFiles/segmenter_jordlee.dir/build.make CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o.provides.build
.PHONY : CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o.provides

CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o.provides.build: CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o

# Object files for target segmenter_jordlee
segmenter_jordlee_OBJECTS = \
"CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o"

# External object files for target segmenter_jordlee
segmenter_jordlee_EXTERNAL_OBJECTS =

segmenter_jordlee: CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o
segmenter_jordlee: CMakeFiles/segmenter_jordlee.dir/build.make
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_system.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_thread.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libpthread.so
segmenter_jordlee: /usr/local/lib/libpcl_common.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
segmenter_jordlee: /usr/local/lib/libpcl_kdtree.so
segmenter_jordlee: /usr/local/lib/libpcl_octree.so
segmenter_jordlee: /usr/lib/libOpenNI.so
segmenter_jordlee: /usr/lib/libOpenNI2.so
segmenter_jordlee: /usr/lib/libvtkCommon.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkFiltering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkImaging.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkGraphics.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkGenericFiltering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkIO.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkRendering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkVolumeRendering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkHybrid.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkWidgets.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkParallel.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkInfovis.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkGeovis.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkViews.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkCharts.so.5.8.0
segmenter_jordlee: /usr/local/lib/libpcl_io.so
segmenter_jordlee: /usr/local/lib/libpcl_search.so
segmenter_jordlee: /usr/local/lib/libpcl_sample_consensus.so
segmenter_jordlee: /usr/local/lib/libpcl_filters.so
segmenter_jordlee: /usr/local/lib/libpcl_features.so
segmenter_jordlee: /usr/local/lib/libpcl_keypoints.so
segmenter_jordlee: /usr/local/lib/libpcl_segmentation.so
segmenter_jordlee: /usr/local/lib/libpcl_visualization.so
segmenter_jordlee: /usr/local/lib/libpcl_outofcore.so
segmenter_jordlee: /usr/local/lib/libpcl_registration.so
segmenter_jordlee: /usr/local/lib/libpcl_recognition.so
segmenter_jordlee: /usr/local/lib/libpcl_tracking.so
segmenter_jordlee: /usr/local/lib/libpcl_people.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libqhull.so
segmenter_jordlee: /usr/local/lib/libpcl_surface.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_system.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_thread.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libpthread.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libqhull.so
segmenter_jordlee: /usr/lib/libOpenNI.so
segmenter_jordlee: /usr/lib/libOpenNI2.so
segmenter_jordlee: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
segmenter_jordlee: /usr/lib/libvtkCommon.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkFiltering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkImaging.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkGraphics.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkGenericFiltering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkIO.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkRendering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkVolumeRendering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkHybrid.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkWidgets.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkParallel.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkInfovis.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkGeovis.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkViews.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkCharts.so.5.8.0
segmenter_jordlee: /usr/local/lib/libpcl_common.so
segmenter_jordlee: /usr/local/lib/libpcl_kdtree.so
segmenter_jordlee: /usr/local/lib/libpcl_octree.so
segmenter_jordlee: /usr/local/lib/libpcl_io.so
segmenter_jordlee: /usr/local/lib/libpcl_search.so
segmenter_jordlee: /usr/local/lib/libpcl_sample_consensus.so
segmenter_jordlee: /usr/local/lib/libpcl_filters.so
segmenter_jordlee: /usr/local/lib/libpcl_features.so
segmenter_jordlee: /usr/local/lib/libpcl_keypoints.so
segmenter_jordlee: /usr/local/lib/libpcl_segmentation.so
segmenter_jordlee: /usr/local/lib/libpcl_visualization.so
segmenter_jordlee: /usr/local/lib/libpcl_outofcore.so
segmenter_jordlee: /usr/local/lib/libpcl_registration.so
segmenter_jordlee: /usr/local/lib/libpcl_recognition.so
segmenter_jordlee: /usr/local/lib/libpcl_tracking.so
segmenter_jordlee: /usr/local/lib/libpcl_people.so
segmenter_jordlee: /usr/local/lib/libpcl_surface.so
segmenter_jordlee: /usr/lib/libvtkViews.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkInfovis.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkWidgets.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkVolumeRendering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkHybrid.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkParallel.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkRendering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkImaging.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkGraphics.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkIO.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkFiltering.so.5.8.0
segmenter_jordlee: /usr/lib/libvtkCommon.so.5.8.0
segmenter_jordlee: /usr/lib/libvtksys.so.5.8.0
segmenter_jordlee: CMakeFiles/segmenter_jordlee.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable segmenter_jordlee"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/segmenter_jordlee.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/segmenter_jordlee.dir/build: segmenter_jordlee
.PHONY : CMakeFiles/segmenter_jordlee.dir/build

CMakeFiles/segmenter_jordlee.dir/requires: CMakeFiles/segmenter_jordlee.dir/src/segmenter_jordlee.cpp.o.requires
.PHONY : CMakeFiles/segmenter_jordlee.dir/requires

CMakeFiles/segmenter_jordlee.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/segmenter_jordlee.dir/cmake_clean.cmake
.PHONY : CMakeFiles/segmenter_jordlee.dir/clean

CMakeFiles/segmenter_jordlee.dir/depend:
	cd /home/fetch/Documents/segmenter_jordlee/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fetch/Documents/segmenter_jordlee /home/fetch/Documents/segmenter_jordlee /home/fetch/Documents/segmenter_jordlee/cmake-build-debug /home/fetch/Documents/segmenter_jordlee/cmake-build-debug /home/fetch/Documents/segmenter_jordlee/cmake-build-debug/CMakeFiles/segmenter_jordlee.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/segmenter_jordlee.dir/depend

