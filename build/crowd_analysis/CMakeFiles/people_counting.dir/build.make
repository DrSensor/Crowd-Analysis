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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wildan/cctv_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wildan/cctv_ws/build

# Include any dependencies generated for this target.
include crowd_analysis/CMakeFiles/people_counting.dir/depend.make

# Include the progress variables for this target.
include crowd_analysis/CMakeFiles/people_counting.dir/progress.make

# Include the compile flags for this target's objects.
include crowd_analysis/CMakeFiles/people_counting.dir/flags.make

crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o: crowd_analysis/CMakeFiles/people_counting.dir/flags.make
crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o: /home/wildan/cctv_ws/src/crowd_analysis/src/people_counting.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/wildan/cctv_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o"
	cd /home/wildan/cctv_ws/build/crowd_analysis && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/people_counting.dir/src/people_counting.cpp.o -c /home/wildan/cctv_ws/src/crowd_analysis/src/people_counting.cpp

crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/people_counting.dir/src/people_counting.cpp.i"
	cd /home/wildan/cctv_ws/build/crowd_analysis && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/wildan/cctv_ws/src/crowd_analysis/src/people_counting.cpp > CMakeFiles/people_counting.dir/src/people_counting.cpp.i

crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/people_counting.dir/src/people_counting.cpp.s"
	cd /home/wildan/cctv_ws/build/crowd_analysis && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/wildan/cctv_ws/src/crowd_analysis/src/people_counting.cpp -o CMakeFiles/people_counting.dir/src/people_counting.cpp.s

crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o.requires:
.PHONY : crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o.requires

crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o.provides: crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o.requires
	$(MAKE) -f crowd_analysis/CMakeFiles/people_counting.dir/build.make crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o.provides.build
.PHONY : crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o.provides

crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o.provides.build: crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o

# Object files for target people_counting
people_counting_OBJECTS = \
"CMakeFiles/people_counting.dir/src/people_counting.cpp.o"

# External object files for target people_counting
people_counting_EXTERNAL_OBJECTS =

/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: crowd_analysis/CMakeFiles/people_counting.dir/build.make
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/libcv_bridge.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_videostab.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_superres.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_stitching.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_ocl.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_gpu.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_contrib.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/libimage_transport.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/libmessage_filters.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libtinyxml.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/libclass_loader.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/libPocoFoundation.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libdl.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/libroscpp.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libboost_signals.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libboost_filesystem.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/librosconsole.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/liblog4cxx.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libboost_regex.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/libroslib.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/librostime.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libboost_date_time.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /opt/ros/indigo/lib/libcpp_common.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libboost_system.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libboost_thread.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libpthread.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libconsole_bridge.so
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_videostab.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_superres.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_stitching.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_ocl.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_gpu.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_contrib.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting: crowd_analysis/CMakeFiles/people_counting.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting"
	cd /home/wildan/cctv_ws/build/crowd_analysis && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/people_counting.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
crowd_analysis/CMakeFiles/people_counting.dir/build: /home/wildan/cctv_ws/devel/lib/crowd_analysis/people_counting
.PHONY : crowd_analysis/CMakeFiles/people_counting.dir/build

crowd_analysis/CMakeFiles/people_counting.dir/requires: crowd_analysis/CMakeFiles/people_counting.dir/src/people_counting.cpp.o.requires
.PHONY : crowd_analysis/CMakeFiles/people_counting.dir/requires

crowd_analysis/CMakeFiles/people_counting.dir/clean:
	cd /home/wildan/cctv_ws/build/crowd_analysis && $(CMAKE_COMMAND) -P CMakeFiles/people_counting.dir/cmake_clean.cmake
.PHONY : crowd_analysis/CMakeFiles/people_counting.dir/clean

crowd_analysis/CMakeFiles/people_counting.dir/depend:
	cd /home/wildan/cctv_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wildan/cctv_ws/src /home/wildan/cctv_ws/src/crowd_analysis /home/wildan/cctv_ws/build /home/wildan/cctv_ws/build/crowd_analysis /home/wildan/cctv_ws/build/crowd_analysis/CMakeFiles/people_counting.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : crowd_analysis/CMakeFiles/people_counting.dir/depend
