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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/research/tutorialROSOpenCV

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/research/tutorialROSOpenCV/build

# Include any dependencies generated for this target.
include CMakeFiles/tutorialROSOpenCV.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tutorialROSOpenCV.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tutorialROSOpenCV.dir/flags.make

CMakeFiles/tutorialROSOpenCV.dir/src/main.o: CMakeFiles/tutorialROSOpenCV.dir/flags.make
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: ../src/main.cpp
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: ../manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/camera_umd/uvc_camera/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
CMakeFiles/tutorialROSOpenCV.dir/src/main.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /root/research/tutorialROSOpenCV/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tutorialROSOpenCV.dir/src/main.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/tutorialROSOpenCV.dir/src/main.o -c /root/research/tutorialROSOpenCV/src/main.cpp

CMakeFiles/tutorialROSOpenCV.dir/src/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tutorialROSOpenCV.dir/src/main.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /root/research/tutorialROSOpenCV/src/main.cpp > CMakeFiles/tutorialROSOpenCV.dir/src/main.i

CMakeFiles/tutorialROSOpenCV.dir/src/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tutorialROSOpenCV.dir/src/main.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /root/research/tutorialROSOpenCV/src/main.cpp -o CMakeFiles/tutorialROSOpenCV.dir/src/main.s

CMakeFiles/tutorialROSOpenCV.dir/src/main.o.requires:
.PHONY : CMakeFiles/tutorialROSOpenCV.dir/src/main.o.requires

CMakeFiles/tutorialROSOpenCV.dir/src/main.o.provides: CMakeFiles/tutorialROSOpenCV.dir/src/main.o.requires
	$(MAKE) -f CMakeFiles/tutorialROSOpenCV.dir/build.make CMakeFiles/tutorialROSOpenCV.dir/src/main.o.provides.build
.PHONY : CMakeFiles/tutorialROSOpenCV.dir/src/main.o.provides

CMakeFiles/tutorialROSOpenCV.dir/src/main.o.provides.build: CMakeFiles/tutorialROSOpenCV.dir/src/main.o

# Object files for target tutorialROSOpenCV
tutorialROSOpenCV_OBJECTS = \
"CMakeFiles/tutorialROSOpenCV.dir/src/main.o"

# External object files for target tutorialROSOpenCV
tutorialROSOpenCV_EXTERNAL_OBJECTS =

../bin/tutorialROSOpenCV: CMakeFiles/tutorialROSOpenCV.dir/src/main.o
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_calib3d.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_contrib.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_core.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_features2d.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_flann.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_gpu.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_highgui.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_imgproc.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_legacy.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_ml.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_nonfree.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_objdetect.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_photo.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_stitching.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_ts.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_video.so
../bin/tutorialROSOpenCV: /opt/ros/fuerte/lib/libopencv_videostab.so
../bin/tutorialROSOpenCV: CMakeFiles/tutorialROSOpenCV.dir/build.make
../bin/tutorialROSOpenCV: CMakeFiles/tutorialROSOpenCV.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/tutorialROSOpenCV"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tutorialROSOpenCV.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tutorialROSOpenCV.dir/build: ../bin/tutorialROSOpenCV
.PHONY : CMakeFiles/tutorialROSOpenCV.dir/build

CMakeFiles/tutorialROSOpenCV.dir/requires: CMakeFiles/tutorialROSOpenCV.dir/src/main.o.requires
.PHONY : CMakeFiles/tutorialROSOpenCV.dir/requires

CMakeFiles/tutorialROSOpenCV.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tutorialROSOpenCV.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tutorialROSOpenCV.dir/clean

CMakeFiles/tutorialROSOpenCV.dir/depend:
	cd /root/research/tutorialROSOpenCV/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/research/tutorialROSOpenCV /root/research/tutorialROSOpenCV /root/research/tutorialROSOpenCV/build /root/research/tutorialROSOpenCV/build /root/research/tutorialROSOpenCV/build/CMakeFiles/tutorialROSOpenCV.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tutorialROSOpenCV.dir/depend

