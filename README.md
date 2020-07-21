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
- [Frameworks](#frameworks)
- [Algorithms](#algorithms)
  - [Basic matching algorithms](#basic-matching-algorithms)
  - [LIDAR-based odometry and or mapping (LOAM)](#lidar-based-odometry-and-or-mapping-loam)
  - [Semantic segmentation](#semantic-segmentation)
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
- [Ouster](https://ouster.com/) - LIDAR manufacturer, specializing in digital-spinning LiDARs.
  - [YouTube channel :red_circle:](https://www.youtube.com/channel/UCRB5JpLey3BA-1P9XyrErTA/)
  - [GitHub organization :octocat:](https://github.com/ouster-lidar)
- [Livox](https://www.livoxtech.com/) - LIDAR manufacturer.
  - [YouTube channel :red_circle:](https://www.youtube.com/channel/UCnLpB5QxlQUexi40vM12mNQ)
  - [GitHub organization :octocat:](https://github.com/Livox-SDK)
- [SICK](https://www.sick.com/ag/en/) - Sensor and automation manufacturer.
  - [YouTube channel :red_circle:](https://www.youtube.com/user/SICKSensors)
  - [GitHub organization :octocat:](https://github.com/ouster-lidar)
- [Hokuyo](https://www.hokuyo-aut.jp/) - Sensor and automation manufacturer.
  - [YouTube channel :red_circle:](https://www.youtube.com/channel/UCYzJXC82IEy-h-io2REin5g)
- [Pioneer](http://autonomousdriving.pioneer/en/3d-lidar/) - LIDAR manufacturer, specializing in MEMS mirror-based raster scanning LiDARs (3D-LiDAR).
  - [YouTube channel :red_circle:](https://www.youtube.com/channel/UC1VGM4uK3xo7896iOpiMzJA)
- [Luminar](https://www.luminartech.com/) - LIDAR manufacturer focusing on compact, auto-grade sensors.
  - [Vimeo channel :red_circle:](https://vimeo.com/luminartech)
  - [GitHub organization :octocat:](https://github.com/luminartech)
- [Hesai](https://www.hesaitech.com/) - Hesai Technology is a LIDAR manufacturer, founded in Shanghai.
  - [YouTube channel :red_circle:](https://www.youtube.com/channel/UCG2_ffm6sdMsK-FX8yOLNYQ/videos)
  - [GitHub organization :octocat:](https://github.com/HesaiTechnology)
- [Robosense](http://www.robosense.ai/) - RoboSense (Suteng Innovation Technology Co., Ltd.) is a LIDAR sensor, AI algorithm and IC chipset maufactuirer based in Shenzhen, Beijing.
  - [YouTube channel :red_circle:](https://www.youtube.com/channel/UCYCK8j678N6d_ayWE_8F3rQ)
  - [GitHub organization :octocat:](https://github.com/RoboSense-LiDAR)


## Datasets
- [Ford Dataset](https://avdata.ford.com/) - The dataset is time-stamped and contains raw data from all the sensors, calibration values, pose trajectory, ground truth pose, and 3D maps. The data is Robot Operating System (ROS) compatible.
- [Audi A2D2 Dataset](https://www.a2d2.audi) - The dataset features 2D semantic segmentation, 3D point clouds, 3D bounding boxes, and vehicle bus data.
- [Waymo Open Dataset](https://waymo.com/open/) - The dataset contains independently-generated labels for lidar and camera data, not simply projections.
- [Oxford RobotCar](https://robotcar-dataset.robots.ox.ac.uk/) - The Oxford RobotCar Dataset contains over 100 repetitions of a consistent route through Oxford, UK, captured over a period of over a year. 
- [EU Long-term Dataset](https://epan-utbm.github.io/utbm_robocar_dataset/) - This dataset was collected with our robocar (in human driving mode of course), equipped up to eleven heterogeneous sensors, in the downtown (for long-term data) and a suburb (for roundabout data) of Montbéliard in France. The vehicle speed was limited to 50 km/h following the French traffic rules.
- [NuScenes](https://www.nuscenes.org/) - Public large-scale dataset for autonomous driving.
- [Lyft](https://level5.lyft.com/dataset/) - Public dataset collected by a fleet of Ford Fusion vehicles equipped with LIDAR and camera.
- [KITTI](http://www.cvlibs.net/datasets/kitti/raw_data.php) - Widespread public dataset, pirmarily focusing on computer vision applications, but also contains LIDAR point cloud.
- [Semantic KITTI](http://semantic-kitti.org/) - Dataset for semantic and panoptic scene segmentation.
- [CADC - Canadian Adverse Driving Conditions Dataset](http://cadcd.uwaterloo.ca/) - Public large-scale dataset for autonomous driving in adverse weather conditions (snowy weather).
- [UofTPed50 Dataset](https://www.autodrive.utoronto.ca/uoftped50) - University of Toronto, aUToronto's self-driving car dataset, which contains GPS/IMU, 3D LIDAR, and Monocular camera data. It can be used for 3D pedestrian detection.
- [PandaSet Open Dataset](https://scale.com/open-datasets/pandaset) - Public large-scale dataset for autonomous driving provided by Hesai & Scale. It enables researchers to study challenging urban driving situations using the full sensor suit of a real self-driving-car.

## Libraries
- [Point Cloud Library (PCL)](http://www.pointclouds.org/) - Popular highly parallel programming library, with numerous industrial and research use-cases. 

## Frameworks
- [Autoware](https://www.autoware.ai/) - Popular framework in academic and research applications of autonomous vehicles.
  - [GitLab repository :octocat:](https://gitlab.com/autowarefoundation/autoware.ai)

## Algorithms

### Basic matching algorithms
- [Iterative closest point :red_circle:](https://www.youtube.com/watch?v=uzOCS_gdZuM) - The must-have algorithm for feature matching applications.
- [Normal distributions transform :red_circle:](https://www.youtube.com/watch?v=0YV4a2asb8Y) - More recent massively-parallel approach to feature matching.

### LIDAR-based odometry and or mapping (LOAM)
- [LOAM J. Zhang and S. Singh :red_circle:](https://youtu.be/8ezyhTAEyHs) - LOAM: Lidar Odometry and Mapping in Real-time.
- [LeGO-LOAM :octocat:](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) - A lightweight and ground optimized lidar odometry and mapping (LeGO-LOAM) system for ROS compatible UGVs. 
  - [YouTube :red_circle:](https://www.youtube.com/watch?v=7uCxLUs9fwQ)
### Semantic segmentation
- [RangeNet++](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/milioto2019iros.pdf) - Fast and Accurate LiDAR Sematnic Segmentation with fully convolutional network.
  - [GitHub :octocat:](https://github.com/PRBonn/rangenet_lib)
### Simultaneous localization and mapping (SLAM)
- [Cartographer :octocat:](https://github.com/cartographer-project/cartographer) - Cartographer is ROS compatible system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.
  - [YouTube :red_circle:](https://www.youtube.com/watch?v=29Knm-phAyI)
- [SuMa++](http://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/chen2019iros.pdf) - LiDAR-based Semantic SLAM.
  - [GitHub :octocat:](https://github.com/PRBonn/semantic_suma/)
- [OverlapNet](http://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/chen2020rss.pdf) -  Loop Closing for LiDAR-based SLAM.
  - [GitHub :octocat:](https://github.com/PRBonn/OverlapNet)


### Object detection and object tracking

## Simulators
- CoppeliaSim (formerly V-REP)
- [OSRF Gazebo](http://gazebosim.org/) - OGRE-based general-purpose robotic simulator, ROS/ROS2 compatible.
  - [GitHub repository :octocat:](https://github.com/osrf/gazebo)
- [CARLA](https://carla.org/) - Unreal Engine based simulator for automotive applications. Compatible with Autoware, Baidu Apollo and ROS/ROS2.
  - [GitHub repository :octocat:](https://github.com/carla-simulator/carla)
- [LGSVL](https://www.lgsvlsimulator.com/) - Unity Engine based simulator for automotive applications. Compatible with Autoware, Baidu Apollo and ROS/ROS2.
  - [GitHub repository :octocat:](https://github.com/lgsvl/simulator)

## Others
- [Pointcloudprinter](https://github.com/marian42/pointcloudprinter) - A tool to turn pointcloud data from aerial lidar scans into solid meshes for 3D printing.
