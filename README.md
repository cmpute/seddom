# Real-time semantic dense occupancy mapping
Fast semantic dense occupancy mapping based on random free space sampling and spherical rtree.

# Getting Started

## Prerequisites

This program requires the following libraries (besides ROS) to be installed aforehead.

- msgpack: `sudo apt install libmsgpack-dev`
- grid-map: `sudo apt-get install ros-$ROS_DISTRO-grid-map`
- (Optional) easy_profiler: https://github.com/yse/easy_profiler#build

## Building with catkin

```bash
catkin_ws/src$ git clone https://github.com/catkin/catkin_simple.git
catkin_ws/src$ git clone <this repository> --recurse-submodules
catkin_ws/src$ cd ..

catkin_ws$ catkin_make -DCMAKE_BUILD_TYPE=Release
# or
catkin_ws$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

## Running the Demo

```bash
$ roslaunch seddom toy_example_node.launch
```

<details>

<summary> Semantic Mapping using KITTI dataset </summary>

<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/kitti_05.png" width=320><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/kitti_15.png" width=540>

### Download Data
Please download [data_kitti_15](https://drive.google.com/file/d/1dIHRrsA7rZSRJ6M9Uz_75ZxcHHY96Gmb/view?usp=sharing) and uncompress it into the data folder.

### Running
```bash
$ roslaunch seddom kitti_node.launch
```
This includes a visualization of the mapping result.

</details>

<details>

<summary> Semantic Mapping using SemanticKITTI dataset </summary>

<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/semantic_kitti_seq05.png" width=430><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/semantic_kitti_seq04.png" width=430>

### Download Data
Please download [semantickitti_04](https://drive.google.com/file/d/19Dv1jQqf-VGKS2qvbygFlUzQoSvu17E5/view?usp=sharing) and uncompress it into the data folder.

### Running
```bash
$ roslaunch seddom semantickitti_node.launch
```
This includes a visualization of the mapping result.

</details>

## Online mapping

```bash
# build the occupancy map from the topic /annotation_3dpoints
$ roslaunch seddom clapper.launch  

# start mapping with data from rosbag in CLAPPER format
$ roslaunch seddom clapper.launch data_path:=... 

# dump map into a serialized format
$ rosservice call /octomap_server/dump_map ~/test.bin
```

# Reference

This work is published as [**Real-time Semantic 3D Dense Occupancy Mapping with Efficient Free Space Representations**](https://arxiv.org/abs/2107.02981), we will appreciate it if you cite it at
```
@article{zhong2021real,
    title={Real-time Semantic 3D Dense Occupancy Mapping with Efficient Free Space Representations},
    author={Zhong, Yuanxin and Peng, Huei},
    journal={2022 IEEE International Intelligent Transportation Systems Conference (ITSC)},
    year={2022}
}
```

This project is based on [BKISemanticMapping](https://github.com/ganlumomo/BKISemanticMapping.git). Cite their work at
```
@article{gan2019bayesian,
    author={L. {Gan} and R. {Zhang} and J. W. {Grizzle} and R. M. {Eustice} and M. {Ghaffari}},
    journal={IEEE Robotics and Automation Letters},
    title={Bayesian Spatial Kernel Smoothing for Scalable Dense Semantic Mapping},
    year={2020},
    volume={5},
    number={2},
    pages={790-797},
    keywords={Mapping;semantic scene understanding;range sensing;RGB-D perception},
    doi={10.1109/LRA.2020.2965390},
    ISSN={2377-3774},
    month={April},
}
```
