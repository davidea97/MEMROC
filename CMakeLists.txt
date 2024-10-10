cmake_minimum_required(VERSION 3.16.3)

project(MEMROC)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

# Find packages
#set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-11.3")
#set(CUDA_CUDA_LIBRARY "/usr/local/cuda-11.3/targets/x86_64-linux/lib/libcudart_static.a")

#find_package(CUDA REQUIRED)

#include_directories(${CUDA_INCLUDE_DIRS})
#link_directories(${CUDA_LIBRARIES})


find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# Include directories
include_directories(
  include
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Add an executable target
add_executable(MEMROC src/main.cpp)

# Add a library
add_library(${PROJECT_NAME}_utils 
  src/Calibrator.cpp
  include/Calibrator.h
  src/utils.cpp
  include/utils.h
  src/CameraInfo.cpp
  include/CameraInfo.h
  src/Detector.cpp
  include/Detector.h
  src/display_utils.cpp
  include/display_utils.h
  src/PinholeCameraModel.cpp
  include/PinholeCameraModel.h
  src/Reader.cpp
  include/Reader.h
  include/handeye_calibrator.h
  include/CalibrationInfo.h
  #src/Metrics.cpp
  #include/Metrics.h
  src/MobileHandEyeCalibrator.cpp
  include/MobileHandEyeCalibrator.h
  src/CalibrationInfo.cpp
)

# Link libraries
target_link_libraries(MEMROC
  ${PROJECT_NAME}_utils
  ${OpenCV_LIBS}
  Eigen3::Eigen
  yaml-cpp
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES}
)



