# MEMROC: Multi-Eye to Mobile RObot Calibration
#### Abstract
This work presents MEMROC (Multi-Eye to Mobile RObot Calibration), a novel motion-based calibration method that simplifies the process of accurately calibrating multiple cameras relative to a mobile robot’s reference frame. MEMROC utilizes a known calibration pattern to facilitate accurate calibration with a lower number of images during the optimization process. Additionally, it leverages robust ground plane detection for comprehensive 6-DoF extrinsic calibration, overcoming a critical limitation of many existing methods that struggle to estimate the complete camera pose. The proposed method addresses the need for frequent recalibration in dy- namic environments, where cameras may shift slightly or alter their positions due to daily usage, operational adjustments, or vibrations from mobile robot movements. MEMROC ex- hibits remarkable robustness to noisy odometry data, requiring minimal calibration input data. This combination makes it highly suitable for daily operations involving mobile robots. A comprehensive set of experiments on both synthetic and real data proves MEMROC’s efficiency, surpassing existing state- of-the-art methods in terms of accuracy, robustness, and ease of use. 

[[Arxiv]]([URL](https://arxiv.org/abs/2410.08805))

![MEMROC Overview](images/MEMROC_block.png)

## Installation
Download the repository
```
git clone https://github.com/davidea97/MEMROC.git
```
and move forward with the compilation

```
mkdir build
cd build
cmake ..
make -j8
```

## Dataset download
This work introduces a comprehensive dataset comprising both synthetic and real images.
![MEMROC Overview](images/dataset.png)
_Dataset soon avaialble_: You can download the dataset directly from the following [link](URL) or use your own data.


It is essential that the structure of your data is as follows:

* [README.md](./README.md)
* [src](./src)
* [data](./data)
   * [<name_of_your_data_folder>](./data/exp1)
        * [camera1](./data/exp1/camera1)
          *
          *
          *
        * [camera2](./data/exp1/camera1)
        * [cameraN](./data/exp1/camera1)
        * [CalibrationInfo.yaml]
* [include](./include)
* [build](build)
* [CMakeLists.txt](./CMakeLists.txt)


In particular the camera folders has to include:

The CalibrationInfo.yaml must have the following structure:
```
# Number of cameras you want to calibrate
number_of_cameras: 3
# Camera folder name where pose and image subfolders are located
camera_folder_prefix: camera
# Pattern type used
pattern_type: checkerboard

# Number of rows
number_of_rows: 3
# Number of columns
number_of_columns: 4
# Pattern size
size: 0.10

# Resize factor
resize_factor: 1
# Store reprojected corners: 0 to remove visualization, 1 if you want to store them
visual_error: 1
# Evaluation with ground truth: 0 if the GT is not available, 1 if the ground truth is provided in GT folder
gt: 0
# Calibration type: 0 eye-in-hand, 1 eye-on-base
calibration_setup: 2
# Calibration info: 0 not needed, 1 perform it
intrinsic_calibration: 0
# Metric AX=ZB
metric: 1
``` 

## Calibration process usage
![MEMROC Overview](images/method.png)

To execute the following code, the ceres-solver library (link), opencv>4.2, eigen>3.3.7 and PCL libraries must be correctly installed. 

```
cd build
./MEMROC ../data/<name_of_your_data_folder>/
```

It will provide the resulting rototranslation describing the relative position of each camera mounted on the mobile robot with respect to the mobile robot reference frame.

## Citation

## License
The MEMROC dataset is released under the CC BY-NC-SA 4.0 license. 

Authors reserve the right to change the license in the future.

Authors of the MEMROC dataset are not responsible for any misuse of the dataset or any potential harm caused by the use of the dataset.

