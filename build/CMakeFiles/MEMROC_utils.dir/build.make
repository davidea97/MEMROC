# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

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

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/davide/.local/lib/python3.10/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/davide/.local/lib/python3.10/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/davide/davide_ws/src/MEMROC

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/davide/davide_ws/src/MEMROC/build

# Include any dependencies generated for this target.
include CMakeFiles/MEMROC_utils.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/MEMROC_utils.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/MEMROC_utils.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MEMROC_utils.dir/flags.make

CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.o: CMakeFiles/MEMROC_utils.dir/flags.make
CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.o: /home/davide/davide_ws/src/MEMROC/src/Calibrator.cpp
CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.o: CMakeFiles/MEMROC_utils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/davide/davide_ws/src/MEMROC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.o -MF CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.o.d -o CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.o -c /home/davide/davide_ws/src/MEMROC/src/Calibrator.cpp

CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davide/davide_ws/src/MEMROC/src/Calibrator.cpp > CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.i

CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davide/davide_ws/src/MEMROC/src/Calibrator.cpp -o CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.s

CMakeFiles/MEMROC_utils.dir/src/utils.cpp.o: CMakeFiles/MEMROC_utils.dir/flags.make
CMakeFiles/MEMROC_utils.dir/src/utils.cpp.o: /home/davide/davide_ws/src/MEMROC/src/utils.cpp
CMakeFiles/MEMROC_utils.dir/src/utils.cpp.o: CMakeFiles/MEMROC_utils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/davide/davide_ws/src/MEMROC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/MEMROC_utils.dir/src/utils.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MEMROC_utils.dir/src/utils.cpp.o -MF CMakeFiles/MEMROC_utils.dir/src/utils.cpp.o.d -o CMakeFiles/MEMROC_utils.dir/src/utils.cpp.o -c /home/davide/davide_ws/src/MEMROC/src/utils.cpp

CMakeFiles/MEMROC_utils.dir/src/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MEMROC_utils.dir/src/utils.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davide/davide_ws/src/MEMROC/src/utils.cpp > CMakeFiles/MEMROC_utils.dir/src/utils.cpp.i

CMakeFiles/MEMROC_utils.dir/src/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MEMROC_utils.dir/src/utils.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davide/davide_ws/src/MEMROC/src/utils.cpp -o CMakeFiles/MEMROC_utils.dir/src/utils.cpp.s

CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.o: CMakeFiles/MEMROC_utils.dir/flags.make
CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.o: /home/davide/davide_ws/src/MEMROC/src/CameraInfo.cpp
CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.o: CMakeFiles/MEMROC_utils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/davide/davide_ws/src/MEMROC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.o -MF CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.o.d -o CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.o -c /home/davide/davide_ws/src/MEMROC/src/CameraInfo.cpp

CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davide/davide_ws/src/MEMROC/src/CameraInfo.cpp > CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.i

CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davide/davide_ws/src/MEMROC/src/CameraInfo.cpp -o CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.s

CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.o: CMakeFiles/MEMROC_utils.dir/flags.make
CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.o: /home/davide/davide_ws/src/MEMROC/src/Detector.cpp
CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.o: CMakeFiles/MEMROC_utils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/davide/davide_ws/src/MEMROC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.o -MF CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.o.d -o CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.o -c /home/davide/davide_ws/src/MEMROC/src/Detector.cpp

CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davide/davide_ws/src/MEMROC/src/Detector.cpp > CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.i

CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davide/davide_ws/src/MEMROC/src/Detector.cpp -o CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.s

CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.o: CMakeFiles/MEMROC_utils.dir/flags.make
CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.o: /home/davide/davide_ws/src/MEMROC/src/display_utils.cpp
CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.o: CMakeFiles/MEMROC_utils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/davide/davide_ws/src/MEMROC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.o -MF CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.o.d -o CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.o -c /home/davide/davide_ws/src/MEMROC/src/display_utils.cpp

CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davide/davide_ws/src/MEMROC/src/display_utils.cpp > CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.i

CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davide/davide_ws/src/MEMROC/src/display_utils.cpp -o CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.s

CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.o: CMakeFiles/MEMROC_utils.dir/flags.make
CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.o: /home/davide/davide_ws/src/MEMROC/src/PinholeCameraModel.cpp
CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.o: CMakeFiles/MEMROC_utils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/davide/davide_ws/src/MEMROC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.o -MF CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.o.d -o CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.o -c /home/davide/davide_ws/src/MEMROC/src/PinholeCameraModel.cpp

CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davide/davide_ws/src/MEMROC/src/PinholeCameraModel.cpp > CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.i

CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davide/davide_ws/src/MEMROC/src/PinholeCameraModel.cpp -o CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.s

CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.o: CMakeFiles/MEMROC_utils.dir/flags.make
CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.o: /home/davide/davide_ws/src/MEMROC/src/Reader.cpp
CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.o: CMakeFiles/MEMROC_utils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/davide/davide_ws/src/MEMROC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.o -MF CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.o.d -o CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.o -c /home/davide/davide_ws/src/MEMROC/src/Reader.cpp

CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davide/davide_ws/src/MEMROC/src/Reader.cpp > CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.i

CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davide/davide_ws/src/MEMROC/src/Reader.cpp -o CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.s

CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.o: CMakeFiles/MEMROC_utils.dir/flags.make
CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.o: /home/davide/davide_ws/src/MEMROC/src/MobileHandEyeCalibrator.cpp
CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.o: CMakeFiles/MEMROC_utils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/davide/davide_ws/src/MEMROC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.o -MF CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.o.d -o CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.o -c /home/davide/davide_ws/src/MEMROC/src/MobileHandEyeCalibrator.cpp

CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davide/davide_ws/src/MEMROC/src/MobileHandEyeCalibrator.cpp > CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.i

CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davide/davide_ws/src/MEMROC/src/MobileHandEyeCalibrator.cpp -o CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.s

CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.o: CMakeFiles/MEMROC_utils.dir/flags.make
CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.o: /home/davide/davide_ws/src/MEMROC/src/CalibrationInfo.cpp
CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.o: CMakeFiles/MEMROC_utils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/davide/davide_ws/src/MEMROC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.o -MF CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.o.d -o CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.o -c /home/davide/davide_ws/src/MEMROC/src/CalibrationInfo.cpp

CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davide/davide_ws/src/MEMROC/src/CalibrationInfo.cpp > CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.i

CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davide/davide_ws/src/MEMROC/src/CalibrationInfo.cpp -o CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.s

# Object files for target MEMROC_utils
MEMROC_utils_OBJECTS = \
"CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.o" \
"CMakeFiles/MEMROC_utils.dir/src/utils.cpp.o" \
"CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.o" \
"CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.o" \
"CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.o" \
"CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.o" \
"CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.o" \
"CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.o" \
"CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.o"

# External object files for target MEMROC_utils
MEMROC_utils_EXTERNAL_OBJECTS =

libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/src/Calibrator.cpp.o
libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/src/utils.cpp.o
libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/src/CameraInfo.cpp.o
libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/src/Detector.cpp.o
libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/src/display_utils.cpp.o
libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/src/PinholeCameraModel.cpp.o
libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/src/Reader.cpp.o
libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/src/MobileHandEyeCalibrator.cpp.o
libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/src/CalibrationInfo.cpp.o
libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/build.make
libMEMROC_utils.a: CMakeFiles/MEMROC_utils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/davide/davide_ws/src/MEMROC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX static library libMEMROC_utils.a"
	$(CMAKE_COMMAND) -P CMakeFiles/MEMROC_utils.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MEMROC_utils.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MEMROC_utils.dir/build: libMEMROC_utils.a
.PHONY : CMakeFiles/MEMROC_utils.dir/build

CMakeFiles/MEMROC_utils.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MEMROC_utils.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MEMROC_utils.dir/clean

CMakeFiles/MEMROC_utils.dir/depend:
	cd /home/davide/davide_ws/src/MEMROC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/davide/davide_ws/src/MEMROC /home/davide/davide_ws/src/MEMROC /home/davide/davide_ws/src/MEMROC/build /home/davide/davide_ws/src/MEMROC/build /home/davide/davide_ws/src/MEMROC/build/CMakeFiles/MEMROC_utils.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/MEMROC_utils.dir/depend

