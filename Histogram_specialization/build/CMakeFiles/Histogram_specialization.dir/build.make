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
CMAKE_SOURCE_DIR = /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization/build

# Include any dependencies generated for this target.
include CMakeFiles/Histogram_specialization.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Histogram_specialization.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Histogram_specialization.dir/flags.make

CMakeFiles/Histogram_specialization.dir/main.cpp.o: CMakeFiles/Histogram_specialization.dir/flags.make
CMakeFiles/Histogram_specialization.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Histogram_specialization.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Histogram_specialization.dir/main.cpp.o -c /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization/main.cpp

CMakeFiles/Histogram_specialization.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Histogram_specialization.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization/main.cpp > CMakeFiles/Histogram_specialization.dir/main.cpp.i

CMakeFiles/Histogram_specialization.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Histogram_specialization.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization/main.cpp -o CMakeFiles/Histogram_specialization.dir/main.cpp.s

CMakeFiles/Histogram_specialization.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/Histogram_specialization.dir/main.cpp.o.requires

CMakeFiles/Histogram_specialization.dir/main.cpp.o.provides: CMakeFiles/Histogram_specialization.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Histogram_specialization.dir/build.make CMakeFiles/Histogram_specialization.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Histogram_specialization.dir/main.cpp.o.provides

CMakeFiles/Histogram_specialization.dir/main.cpp.o.provides.build: CMakeFiles/Histogram_specialization.dir/main.cpp.o


# Object files for target Histogram_specialization
Histogram_specialization_OBJECTS = \
"CMakeFiles/Histogram_specialization.dir/main.cpp.o"

# External object files for target Histogram_specialization
Histogram_specialization_EXTERNAL_OBJECTS =

Histogram_specialization: CMakeFiles/Histogram_specialization.dir/main.cpp.o
Histogram_specialization: CMakeFiles/Histogram_specialization.dir/build.make
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudabgsegm.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudaobjdetect.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudastereo.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_stitching.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_superres.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_videostab.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_aruco.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_bgsegm.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_bioinspired.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_ccalib.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_dpm.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_freetype.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_fuzzy.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_hdf.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_line_descriptor.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_optflow.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_reg.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_saliency.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_stereo.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_structured_light.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_surface_matching.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_tracking.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_xfeatures2d.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_ximgproc.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_xobjdetect.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_xphoto.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudafeatures2d.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_shape.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudacodec.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudaoptflow.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudalegacy.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudawarping.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_viz.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_phase_unwrapping.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_rgbd.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_calib3d.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_video.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_datasets.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_dnn.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_face.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_plot.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_text.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_features2d.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_flann.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_objdetect.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_ml.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_highgui.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_photo.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudaimgproc.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudafilters.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudaarithm.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_videoio.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_imgcodecs.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_imgproc.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_core.so.3.2.0
Histogram_specialization: /home/yxl/opencv/build/lib/libopencv_cudev.so.3.2.0
Histogram_specialization: CMakeFiles/Histogram_specialization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Histogram_specialization"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Histogram_specialization.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Histogram_specialization.dir/build: Histogram_specialization

.PHONY : CMakeFiles/Histogram_specialization.dir/build

CMakeFiles/Histogram_specialization.dir/requires: CMakeFiles/Histogram_specialization.dir/main.cpp.o.requires

.PHONY : CMakeFiles/Histogram_specialization.dir/requires

CMakeFiles/Histogram_specialization.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Histogram_specialization.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Histogram_specialization.dir/clean

CMakeFiles/Histogram_specialization.dir/depend:
	cd /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization/build /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization/build /home/yxl/github/Visual-Localization-Percessing-Homework/Histogram_specialization/build/CMakeFiles/Histogram_specialization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Histogram_specialization.dir/depend

