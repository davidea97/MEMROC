# MEMROC: Multi-Eye to Mobile RObot Calibration
#### Abstract
This work presents MEMROC (Multi-Eye to Mobile RObot Calibration), a novel motion-based calibration method that simplifies the process of accurately calibrating multiple cameras relative to a mobile robot’s reference frame. MEMROC utilizes a known calibration pattern to facilitate accurate calibration with a lower number of images during the optimization process. Additionally, it leverages robust ground plane detection for comprehensive 6-DoF extrinsic calibration, overcoming a critical limitation of many existing methods that struggle to estimate the complete camera pose. The proposed method addresses the need for frequent recalibration in dy- namic environments, where cameras may shift slightly or alter their positions due to daily usage, operational adjustments, or vibrations from mobile robot movements. MEMROC ex- hibits remarkable robustness to noisy odometry data, requiring minimal calibration input data. This combination makes it highly suitable for daily operations involving mobile robots. A comprehensive set of experiments on both synthetic and real data proves MEMROC’s efficiency, surpassing existing state- of-the-art methods in terms of accuracy, robustness, and ease of use. 

[[Arxiv]](URL)

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
This work proposes a dataset containing synthetic and real images. 
![MEMROC Overview](images/dataset.png)
You can download the dataset directly from the following [link](URL) or use your own data.
It is essential that the structure of your data is as follows:
Repository Structure:

* [README.md](./README.md)
* [src](./src)
* [data](./data)
   * [exp1](./data/exp1)
        * [camera1](./data/exp1/camera1)
        * [camera2](./data/exp1/camera1)
        * [cameraN](./data/exp1/camera1)
        * [config.yaml]
   * [exp2](./data/exp2)
* [include](./include)
* [build](build)
* [CMakeLists.txt](./CMakeLists.txt)

## Calibration process usage
![MEMROC Overview](images/method.png)

## Citation

## License
The MEMROC dataset is released under the CC BY-NC-SA 4.0 license. 

Authors reserve the right to change the license in the future.

Authors of the MEMROC dataset are not responsible for any misuse of the dataset or any potential harm caused by the use of the dataset.

