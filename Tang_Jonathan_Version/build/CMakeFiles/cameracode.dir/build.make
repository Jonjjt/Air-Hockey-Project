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
CMAKE_SOURCE_DIR = "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build"

# Include any dependencies generated for this target.
include CMakeFiles/cameracode.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cameracode.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cameracode.dir/flags.make

CMakeFiles/cameracode.dir/main.cpp.o: CMakeFiles/cameracode.dir/flags.make
CMakeFiles/cameracode.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cameracode.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cameracode.dir/main.cpp.o -c "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/main.cpp"

CMakeFiles/cameracode.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cameracode.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/main.cpp" > CMakeFiles/cameracode.dir/main.cpp.i

CMakeFiles/cameracode.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cameracode.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/main.cpp" -o CMakeFiles/cameracode.dir/main.cpp.s

CMakeFiles/cameracode.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/cameracode.dir/main.cpp.o.requires

CMakeFiles/cameracode.dir/main.cpp.o.provides: CMakeFiles/cameracode.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/cameracode.dir/build.make CMakeFiles/cameracode.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/cameracode.dir/main.cpp.o.provides

CMakeFiles/cameracode.dir/main.cpp.o.provides.build: CMakeFiles/cameracode.dir/main.cpp.o


CMakeFiles/cameracode.dir/tcamimage.cpp.o: CMakeFiles/cameracode.dir/flags.make
CMakeFiles/cameracode.dir/tcamimage.cpp.o: ../tcamimage.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/cameracode.dir/tcamimage.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cameracode.dir/tcamimage.cpp.o -c "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/tcamimage.cpp"

CMakeFiles/cameracode.dir/tcamimage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cameracode.dir/tcamimage.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/tcamimage.cpp" > CMakeFiles/cameracode.dir/tcamimage.cpp.i

CMakeFiles/cameracode.dir/tcamimage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cameracode.dir/tcamimage.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/tcamimage.cpp" -o CMakeFiles/cameracode.dir/tcamimage.cpp.s

CMakeFiles/cameracode.dir/tcamimage.cpp.o.requires:

.PHONY : CMakeFiles/cameracode.dir/tcamimage.cpp.o.requires

CMakeFiles/cameracode.dir/tcamimage.cpp.o.provides: CMakeFiles/cameracode.dir/tcamimage.cpp.o.requires
	$(MAKE) -f CMakeFiles/cameracode.dir/build.make CMakeFiles/cameracode.dir/tcamimage.cpp.o.provides.build
.PHONY : CMakeFiles/cameracode.dir/tcamimage.cpp.o.provides

CMakeFiles/cameracode.dir/tcamimage.cpp.o.provides.build: CMakeFiles/cameracode.dir/tcamimage.cpp.o


CMakeFiles/cameracode.dir/tcamcamera.cpp.o: CMakeFiles/cameracode.dir/flags.make
CMakeFiles/cameracode.dir/tcamcamera.cpp.o: ../tcamcamera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/cameracode.dir/tcamcamera.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cameracode.dir/tcamcamera.cpp.o -c "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/tcamcamera.cpp"

CMakeFiles/cameracode.dir/tcamcamera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cameracode.dir/tcamcamera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/tcamcamera.cpp" > CMakeFiles/cameracode.dir/tcamcamera.cpp.i

CMakeFiles/cameracode.dir/tcamcamera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cameracode.dir/tcamcamera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/tcamcamera.cpp" -o CMakeFiles/cameracode.dir/tcamcamera.cpp.s

CMakeFiles/cameracode.dir/tcamcamera.cpp.o.requires:

.PHONY : CMakeFiles/cameracode.dir/tcamcamera.cpp.o.requires

CMakeFiles/cameracode.dir/tcamcamera.cpp.o.provides: CMakeFiles/cameracode.dir/tcamcamera.cpp.o.requires
	$(MAKE) -f CMakeFiles/cameracode.dir/build.make CMakeFiles/cameracode.dir/tcamcamera.cpp.o.provides.build
.PHONY : CMakeFiles/cameracode.dir/tcamcamera.cpp.o.provides

CMakeFiles/cameracode.dir/tcamcamera.cpp.o.provides.build: CMakeFiles/cameracode.dir/tcamcamera.cpp.o


CMakeFiles/cameracode.dir/Socket.cpp.o: CMakeFiles/cameracode.dir/flags.make
CMakeFiles/cameracode.dir/Socket.cpp.o: ../Socket.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/cameracode.dir/Socket.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cameracode.dir/Socket.cpp.o -c "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/Socket.cpp"

CMakeFiles/cameracode.dir/Socket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cameracode.dir/Socket.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/Socket.cpp" > CMakeFiles/cameracode.dir/Socket.cpp.i

CMakeFiles/cameracode.dir/Socket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cameracode.dir/Socket.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/Socket.cpp" -o CMakeFiles/cameracode.dir/Socket.cpp.s

CMakeFiles/cameracode.dir/Socket.cpp.o.requires:

.PHONY : CMakeFiles/cameracode.dir/Socket.cpp.o.requires

CMakeFiles/cameracode.dir/Socket.cpp.o.provides: CMakeFiles/cameracode.dir/Socket.cpp.o.requires
	$(MAKE) -f CMakeFiles/cameracode.dir/build.make CMakeFiles/cameracode.dir/Socket.cpp.o.provides.build
.PHONY : CMakeFiles/cameracode.dir/Socket.cpp.o.provides

CMakeFiles/cameracode.dir/Socket.cpp.o.provides.build: CMakeFiles/cameracode.dir/Socket.cpp.o


CMakeFiles/cameracode.dir/ClientSocket.cpp.o: CMakeFiles/cameracode.dir/flags.make
CMakeFiles/cameracode.dir/ClientSocket.cpp.o: ../ClientSocket.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/cameracode.dir/ClientSocket.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cameracode.dir/ClientSocket.cpp.o -c "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/ClientSocket.cpp"

CMakeFiles/cameracode.dir/ClientSocket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cameracode.dir/ClientSocket.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/ClientSocket.cpp" > CMakeFiles/cameracode.dir/ClientSocket.cpp.i

CMakeFiles/cameracode.dir/ClientSocket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cameracode.dir/ClientSocket.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/ClientSocket.cpp" -o CMakeFiles/cameracode.dir/ClientSocket.cpp.s

CMakeFiles/cameracode.dir/ClientSocket.cpp.o.requires:

.PHONY : CMakeFiles/cameracode.dir/ClientSocket.cpp.o.requires

CMakeFiles/cameracode.dir/ClientSocket.cpp.o.provides: CMakeFiles/cameracode.dir/ClientSocket.cpp.o.requires
	$(MAKE) -f CMakeFiles/cameracode.dir/build.make CMakeFiles/cameracode.dir/ClientSocket.cpp.o.provides.build
.PHONY : CMakeFiles/cameracode.dir/ClientSocket.cpp.o.provides

CMakeFiles/cameracode.dir/ClientSocket.cpp.o.provides.build: CMakeFiles/cameracode.dir/ClientSocket.cpp.o


CMakeFiles/cameracode.dir/udp_client_server.cpp.o: CMakeFiles/cameracode.dir/flags.make
CMakeFiles/cameracode.dir/udp_client_server.cpp.o: ../udp_client_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/cameracode.dir/udp_client_server.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cameracode.dir/udp_client_server.cpp.o -c "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/udp_client_server.cpp"

CMakeFiles/cameracode.dir/udp_client_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cameracode.dir/udp_client_server.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/udp_client_server.cpp" > CMakeFiles/cameracode.dir/udp_client_server.cpp.i

CMakeFiles/cameracode.dir/udp_client_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cameracode.dir/udp_client_server.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/udp_client_server.cpp" -o CMakeFiles/cameracode.dir/udp_client_server.cpp.s

CMakeFiles/cameracode.dir/udp_client_server.cpp.o.requires:

.PHONY : CMakeFiles/cameracode.dir/udp_client_server.cpp.o.requires

CMakeFiles/cameracode.dir/udp_client_server.cpp.o.provides: CMakeFiles/cameracode.dir/udp_client_server.cpp.o.requires
	$(MAKE) -f CMakeFiles/cameracode.dir/build.make CMakeFiles/cameracode.dir/udp_client_server.cpp.o.provides.build
.PHONY : CMakeFiles/cameracode.dir/udp_client_server.cpp.o.provides

CMakeFiles/cameracode.dir/udp_client_server.cpp.o.provides.build: CMakeFiles/cameracode.dir/udp_client_server.cpp.o


# Object files for target cameracode
cameracode_OBJECTS = \
"CMakeFiles/cameracode.dir/main.cpp.o" \
"CMakeFiles/cameracode.dir/tcamimage.cpp.o" \
"CMakeFiles/cameracode.dir/tcamcamera.cpp.o" \
"CMakeFiles/cameracode.dir/Socket.cpp.o" \
"CMakeFiles/cameracode.dir/ClientSocket.cpp.o" \
"CMakeFiles/cameracode.dir/udp_client_server.cpp.o"

# External object files for target cameracode
cameracode_EXTERNAL_OBJECTS =

cameracode: CMakeFiles/cameracode.dir/main.cpp.o
cameracode: CMakeFiles/cameracode.dir/tcamimage.cpp.o
cameracode: CMakeFiles/cameracode.dir/tcamcamera.cpp.o
cameracode: CMakeFiles/cameracode.dir/Socket.cpp.o
cameracode: CMakeFiles/cameracode.dir/ClientSocket.cpp.o
cameracode: CMakeFiles/cameracode.dir/udp_client_server.cpp.o
cameracode: CMakeFiles/cameracode.dir/build.make
cameracode: /usr/local/lib/libopencv_gapi.so.4.5.2
cameracode: /usr/local/lib/libopencv_stitching.so.4.5.2
cameracode: /usr/local/lib/libopencv_alphamat.so.4.5.2
cameracode: /usr/local/lib/libopencv_aruco.so.4.5.2
cameracode: /usr/local/lib/libopencv_barcode.so.4.5.2
cameracode: /usr/local/lib/libopencv_bgsegm.so.4.5.2
cameracode: /usr/local/lib/libopencv_bioinspired.so.4.5.2
cameracode: /usr/local/lib/libopencv_ccalib.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudabgsegm.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudafeatures2d.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudaobjdetect.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudastereo.so.4.5.2
cameracode: /usr/local/lib/libopencv_dnn_objdetect.so.4.5.2
cameracode: /usr/local/lib/libopencv_dnn_superres.so.4.5.2
cameracode: /usr/local/lib/libopencv_dpm.so.4.5.2
cameracode: /usr/local/lib/libopencv_face.so.4.5.2
cameracode: /usr/local/lib/libopencv_freetype.so.4.5.2
cameracode: /usr/local/lib/libopencv_fuzzy.so.4.5.2
cameracode: /usr/local/lib/libopencv_hdf.so.4.5.2
cameracode: /usr/local/lib/libopencv_hfs.so.4.5.2
cameracode: /usr/local/lib/libopencv_img_hash.so.4.5.2
cameracode: /usr/local/lib/libopencv_intensity_transform.so.4.5.2
cameracode: /usr/local/lib/libopencv_line_descriptor.so.4.5.2
cameracode: /usr/local/lib/libopencv_mcc.so.4.5.2
cameracode: /usr/local/lib/libopencv_quality.so.4.5.2
cameracode: /usr/local/lib/libopencv_rapid.so.4.5.2
cameracode: /usr/local/lib/libopencv_reg.so.4.5.2
cameracode: /usr/local/lib/libopencv_rgbd.so.4.5.2
cameracode: /usr/local/lib/libopencv_saliency.so.4.5.2
cameracode: /usr/local/lib/libopencv_sfm.so.4.5.2
cameracode: /usr/local/lib/libopencv_stereo.so.4.5.2
cameracode: /usr/local/lib/libopencv_structured_light.so.4.5.2
cameracode: /usr/local/lib/libopencv_superres.so.4.5.2
cameracode: /usr/local/lib/libopencv_surface_matching.so.4.5.2
cameracode: /usr/local/lib/libopencv_tracking.so.4.5.2
cameracode: /usr/local/lib/libopencv_videostab.so.4.5.2
cameracode: /usr/local/lib/libopencv_wechat_qrcode.so.4.5.2
cameracode: /usr/local/lib/libopencv_xfeatures2d.so.4.5.2
cameracode: /usr/local/lib/libopencv_xobjdetect.so.4.5.2
cameracode: /usr/local/lib/libopencv_xphoto.so.4.5.2
cameracode: /usr/local/lib/libopencv_shape.so.4.5.2
cameracode: /usr/local/lib/libopencv_highgui.so.4.5.2
cameracode: /usr/local/lib/libopencv_datasets.so.4.5.2
cameracode: /usr/local/lib/libopencv_plot.so.4.5.2
cameracode: /usr/local/lib/libopencv_text.so.4.5.2
cameracode: /usr/local/lib/libopencv_ml.so.4.5.2
cameracode: /usr/local/lib/libopencv_phase_unwrapping.so.4.5.2
cameracode: /usr/local/lib/libopencv_videoio.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudaoptflow.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudalegacy.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudawarping.so.4.5.2
cameracode: /usr/local/lib/libopencv_optflow.so.4.5.2
cameracode: /usr/local/lib/libopencv_ximgproc.so.4.5.2
cameracode: /usr/local/lib/libopencv_video.so.4.5.2
cameracode: /usr/local/lib/libopencv_dnn.so.4.5.2
cameracode: /usr/local/lib/libopencv_imgcodecs.so.4.5.2
cameracode: /usr/local/lib/libopencv_objdetect.so.4.5.2
cameracode: /usr/local/lib/libopencv_calib3d.so.4.5.2
cameracode: /usr/local/lib/libopencv_features2d.so.4.5.2
cameracode: /usr/local/lib/libopencv_flann.so.4.5.2
cameracode: /usr/local/lib/libopencv_photo.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudaimgproc.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudafilters.so.4.5.2
cameracode: /usr/local/lib/libopencv_imgproc.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudaarithm.so.4.5.2
cameracode: /usr/local/lib/libopencv_core.so.4.5.2
cameracode: /usr/local/lib/libopencv_cudev.so.4.5.2
cameracode: CMakeFiles/cameracode.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable cameracode"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cameracode.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cameracode.dir/build: cameracode

.PHONY : CMakeFiles/cameracode.dir/build

CMakeFiles/cameracode.dir/requires: CMakeFiles/cameracode.dir/main.cpp.o.requires
CMakeFiles/cameracode.dir/requires: CMakeFiles/cameracode.dir/tcamimage.cpp.o.requires
CMakeFiles/cameracode.dir/requires: CMakeFiles/cameracode.dir/tcamcamera.cpp.o.requires
CMakeFiles/cameracode.dir/requires: CMakeFiles/cameracode.dir/Socket.cpp.o.requires
CMakeFiles/cameracode.dir/requires: CMakeFiles/cameracode.dir/ClientSocket.cpp.o.requires
CMakeFiles/cameracode.dir/requires: CMakeFiles/cameracode.dir/udp_client_server.cpp.o.requires

.PHONY : CMakeFiles/cameracode.dir/requires

CMakeFiles/cameracode.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cameracode.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cameracode.dir/clean

CMakeFiles/cameracode.dir/depend:
	cd "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version" "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version" "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build" "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build" "/home/mark/Desktop/Air Hockey Project/Machine Vision and Control Logic/Tang_Jonathan_Version/build/CMakeFiles/cameracode.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/cameracode.dir/depend
