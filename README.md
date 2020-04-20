# Awesome LIDAR List [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

<img src="img/lidar.svg" align="right" width="100">

> A curated list of awesome LIDAR sensors and its applications.

LIDAR is a remote sensing sensor that uses laser light to measure the surroundings in ~cm accuracy.

Contributions are welcome! Please [check out](contributing.md) our guidelines.

## Contents

- [Conventions](#conventions)
- [Manufacturers](#manufacturers)
- [Datasets](#datasets)
- [Libraries](#libraries)
- [Algorithms](#algorithms)
  - [LIDAR-based odometry and or mapping (LOAM)](#lidar-based-odometry-and-or-mapping-loam)
  - [Simultaneous localization and mapping (SLAM)](#simultaneous-localization-and-mapping-slam)
  - [Object detection and object tracking](#object-detection-and-object-tracking)
- [Simulators](#simulators)
- [Others](#others)

## Conventions

- Any list item with an OctoCat :octocat: has a GitHub repo or organization
- Any list item with a RedCircle :red_circle: has YouTube videos or channel

## Manufacturers

- [Velodyne](https://velodynelidar.com/) - LIDAR manufacturer.
  - [YouTube channel :red_circle:](https://www.youtube.com/user/VelodyneLiDAR)
  - [ROS driver :octocat:](https://github.com/ros-drivers/velodyne)
- [Ouster](https://ouster.com/) - LIDAR manufacturer.
  - [YouTube channel :red_circle:](https://www.youtube.com/channel/UCRB5JpLey3BA-1P9XyrErTA/)
  - [GitHub organization :octocat:](https://github.com/ouster-lidar)
- [SICK](https://www.sick.com/ag/en/) - Sensor and automation manufacturer.
  - [YouTube channel :red_circle:](https://www.youtube.com/user/SICKSensors)
  - [GitHub organization :octocat:](https://github.com/ouster-lidar)
- [Hokuyo](https://www.hokuyo-aut.jp/) - Sensor and automation manufacturer.


## Datasets
- [Oxford RobotCar](https://robotcar-dataset.robots.ox.ac.uk/) - The Oxford RobotCar Dataset contains over 100 repetitions of a consistent route through Oxford, UK, captured over a period of over a year. 
- [EU Long-term Dataset](https://epan-utbm.github.io/utbm_robocar_dataset/) - This dataset was collected with our robocar (in human driving mode of course), equipped up to eleven heterogeneous sensors, in the downtown (for long-term data) and a suburb (for roundabout data) of Montbéliard in France. The vehicle speed was limited to 50 km/h following the French traffic rules.
- [NuScenes](https://www.nuscenes.org/) - Public large-scale dataset for autonomous driving.
- [Lyft](https://level5.lyft.com/dataset/) - Public dataset collected by a fleet of Ford Fusion vehicles equipped with LIDAR and camera.
- [KITTI](http://www.cvlibs.net/datasets/kitti/raw_data.php) - Widespread public dataset, pirmarily focusing on computer vision applications, but also contains LIDAR point cloud.

## Libraries
- [Point Cloud Library (PCL)](http://www.pointclouds.org/) - popular highly parallel programming library, with numerous industrial and research use-cases. 

## Algorithms

### Matching
- [Iterative closest point :red_circle:](https://www.youtube.com/watch?v=uzOCS_gdZuM) - a must-have algorithm in feature matching.
- [Normal distributions transform :red_circle:](https://www.youtube.com/watch?v=0YV4a2asb8Y) - a new approach to feature matching.

### LIDAR-based odometry and or mapping (LOAM)
- [LOAM J. Zhang and S. Singh :red_circle:](https://youtu.be/8ezyhTAEyHs) - LOAM: Lidar Odometry and Mapping in Real-time.
- [LeGO-LOAM :octocat:](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) - A lightweight and ground optimized lidar odometry and mapping (LeGO-LOAM) system for ROS compatible UGVs. 
  - [YouTube :red_circle:](https://www.youtube.com/watch?v=7uCxLUs9fwQ)
### Simultaneous localization and mapping (SLAM)
- [Cartographer :octocat:](https://github.com/cartographer-project/cartographer) - Cartographer is ROS compatible system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.
  - [YouTube :red_circle:](https://www.youtube.com/watch?v=29Knm-phAyI)

### Object detection and object tracking

## Simulators
- CoppeliaSim (formerly V-REP)
- [OSRF Gazebo](http://gazebosim.org/) (OGRE-based, :octocat: https://bitbucket.org/osrf/gazebo/src/gazebo11/)
- [CARLA](https://carla.org/) (Unreal Engine based, :octocat: https://github.com/carla-simulator/carla)
- [LGSVL](https://www.lgsvlsimulator.com/) (Unity Engine based, :octocat: https://github.com/lgsvl/simulator)

## Others
- [Pointcloudprinter](https://github.com/marian42/pointcloudprinter) - A tool to turn pointcloud data from aerial lidar scans into solid meshes for 3D printing.
