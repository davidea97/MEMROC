cmake_minimum_required(VERSION 3.16.3)

project(MEMROC)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# Include directories
include_directories(
  include
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add an executable target
add_executable(MEMROC src/main.cpp)

# Add a library
add_library(${PROJECT_NAME}_utils 
  src/Calibrator.cpp
  src/utils.cpp
  src/CameraInfo.cpp
  src/Detector.cpp
  src/display_utils.cpp
  src/Reader.cpp
  src/MobileHandEyeCalibrator.cpp
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

