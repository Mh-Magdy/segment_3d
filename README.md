# segment_3d

## Description

ROS package for computing objects 3D bounding boxes from image segmentation and a registered point coud.

### License

Apache License, Version 2.0

**Maintainer: Mohamed Magdy<br />**

## Table of contents

* [Prerequisites](#Prerequisites)
* [Usage](#Usage)
* [FutureWork](#Work)

  
## Prerequisites

* ultralytics python3-module
 
## Usage

Edit config.yaml file with your working topics and output tf frame.

```
roslaunch segmentation_3d segment_3d.launch
```
## Work

* ROS 2 Humble migration
* Supporting LIDAR point cloud.

