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
CMAKE_SOURCE_DIR = /root/research/PROYECTOTP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/research/PROYECTOTP/build

# Include any dependencies generated for this target.
include CMakeFiles/controlador.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/controlador.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/controlador.dir/flags.make

CMakeFiles/controlador.dir/controllers/controlador/controlador.o: CMakeFiles/controlador.dir/flags.make
CMakeFiles/controlador.dir/controllers/controlador/controlador.o: ../controllers/controlador/controlador.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /root/research/PROYECTOTP/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/controlador.dir/controllers/controlador/controlador.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/controlador.dir/controllers/controlador/controlador.o -c /root/research/PROYECTOTP/controllers/controlador/controlador.cpp

CMakeFiles/controlador.dir/controllers/controlador/controlador.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controlador.dir/controllers/controlador/controlador.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /root/research/PROYECTOTP/controllers/controlador/controlador.cpp > CMakeFiles/controlador.dir/controllers/controlador/controlador.i

CMakeFiles/controlador.dir/controllers/controlador/controlador.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controlador.dir/controllers/controlador/controlador.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /root/research/PROYECTOTP/controllers/controlador/controlador.cpp -o CMakeFiles/controlador.dir/controllers/controlador/controlador.s

CMakeFiles/controlador.dir/controllers/controlador/controlador.o.requires:
.PHONY : CMakeFiles/controlador.dir/controllers/controlador/controlador.o.requires

CMakeFiles/controlador.dir/controllers/controlador/controlador.o.provides: CMakeFiles/controlador.dir/controllers/controlador/controlador.o.requires
	$(MAKE) -f CMakeFiles/controlador.dir/build.make CMakeFiles/controlador.dir/controllers/controlador/controlador.o.provides.build
.PHONY : CMakeFiles/controlador.dir/controllers/controlador/controlador.o.provides

CMakeFiles/controlador.dir/controllers/controlador/controlador.o.provides.build: CMakeFiles/controlador.dir/controllers/controlador/controlador.o

# Object files for target controlador
controlador_OBJECTS = \
"CMakeFiles/controlador.dir/controllers/controlador/controlador.o"

# External object files for target controlador
controlador_EXTERNAL_OBJECTS =

../controllers/controlador/controlador: CMakeFiles/controlador.dir/controllers/controlador/controlador.o
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_calib3d.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_contrib.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_core.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_features2d.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_flann.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_gpu.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_highgui.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_imgproc.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_legacy.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_ml.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_nonfree.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_objdetect.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_photo.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_stitching.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_ts.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_video.so
../controllers/controlador/controlador: /opt/ros/fuerte/lib/libopencv_videostab.so
../controllers/controlador/controlador: /usr/local/webots/lib/libController.so
../controllers/controlador/controlador: CMakeFiles/controlador.dir/build.make
../controllers/controlador/controlador: CMakeFiles/controlador.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../controllers/controlador/controlador"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controlador.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/controlador.dir/build: ../controllers/controlador/controlador
.PHONY : CMakeFiles/controlador.dir/build

CMakeFiles/controlador.dir/requires: CMakeFiles/controlador.dir/controllers/controlador/controlador.o.requires
.PHONY : CMakeFiles/controlador.dir/requires

CMakeFiles/controlador.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/controlador.dir/cmake_clean.cmake
.PHONY : CMakeFiles/controlador.dir/clean

CMakeFiles/controlador.dir/depend:
	cd /root/research/PROYECTOTP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/research/PROYECTOTP /root/research/PROYECTOTP /root/research/PROYECTOTP/build /root/research/PROYECTOTP/build /root/research/PROYECTOTP/build/CMakeFiles/controlador.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/controlador.dir/depend

