# Real-time semantic dense occupancy mapping
Fast semantic dense occupancy mapping based on random free space sampling and spherical rtree

## Getting Started

### Building with catkin

```bash
catkin_ws/src$ git clone https://github.com/catkin/catkin_simple.git
catkin_ws/src$ git clone <this repository>
catkin_ws/src$ cd ..

catkin_ws$ catkin_make -DCMAKE_BUILD_TYPE=Release
# or
catkin_ws$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Running the Demo

```bash
$ roslaunch seddom toy_example_node.launch
```

## Semantic Mapping using KITTI dataset

<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/kitti_05.png" width=320><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/kitti_15.png" width=540>

### Download Data
Please download [data_kitti_15](https://drive.google.com/file/d/1dIHRrsA7rZSRJ6M9Uz_75ZxcHHY96Gmb/view?usp=sharing) and uncompress it into the data folder.

### Running
```bash
$ roslaunch seddom kitti_node.launch
```
You will see semantic map in RViz. It also projects 3D grid onto 2D image for evaluation, stored at data/data_kitti_05/reproj_img.

### Evaluation
Evaluation code is provided in kitti_evaluation.ipynb. You may modify the directory names to run it.


## Semantic Mapping using SemanticKITTI dataset

<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/semantic_kitti_seq05.png" width=430><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/semantic_kitti_seq04.png" width=430>

### Download Data
Please download [semantickitti_04](https://drive.google.com/file/d/19Dv1jQqf-VGKS2qvbygFlUzQoSvu17E5/view?usp=sharing) and uncompress it into the data folder.

### Running
```bash
$ roslaunch seddom semantickitti_node.launch
```
You will see semantic map in RViz. It also query each ground truth point for evaluation, stored at data/semantickitti_04/evaluations.

## Reference
Codebase of this project is based on [BKISemanticMapping](https://github.com/ganlumomo/BKISemanticMapping.git). Thanks for their great work!

## Online mapping

```bash
$ roslaunch seddom clapper.launch data_path:=...  # start mapping with data from rosbag in CLAPPER format
$ rosservice call /octomap_server/dump_map ~/test.bin  # dump map into a serialized format
```
