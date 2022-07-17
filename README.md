# Awesome ROS Tools 
[![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

The list of tools and packages for Robot Operating System development!

## Table of Contents

* [Best practices](#best_practices)
* [Development tools](#development-tools)
  * [Documentation](#documentation)  
* [Bag files](#bag-files)
* [Visualization](#visualization)
* [Code testing](#code-testing)
* [Simulation](#simulation)
* [Hardware](#hardware)
  * [Sensors](#sensors)
* [Web tools](#web-tools)
* [Other](#other)

## Best Practices

* [ros_best_practices](https://github.com/leggedrobotics/ros_best_practices) - loose collection of best practices, conventions, and tricks for using the Robot Operating System (ROS). It builds up on the official ROS documentation and other resources and is meant as summary and overview ![ros_best_practices](https://img.shields.io/github/stars/leggedrobotics/ros_best_practices.svg?style=flat&label=Star&maxAge=86400)
* [ROS Hacks Repo](https://github.com/yossioo/ROS-Hacks) - the repository is designed to make ROS developer's life easier. After the installation usefull aliases and functions will be added to the terminal.  ![ROS Hacks Repo](https://img.shields.io/github/stars/yossioo/ROS-Hacks.svg?style=flat&label=Star&maxAge=86400)

## Development Tools

* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html) - command line tools for working with catkin [github](https://github.com/catkin/catkin_tools). ![catkin_tools](https://img.shields.io/github/stars/catkin/catkin_tools.svg?style=flat&label=Star&maxAge=86400)
* [ROSCpp Code Format](https://github.com/davetcoleman/roscpp_code_format) - the repo contains an auto formatting script for the [ROS C++ Style Guidelines](http://wiki.ros.org/CppStyleGuide). ![roscpp_code_format](https://img.shields.io/github/stars/davetcoleman/roscpp_code_format.svg?style=flat&label=Star&maxAge=86400)
* [swri_profiler](https://github.com/swri-robotics/swri_profiler) - is a lightweight profiling framework for C++ ROS nodes. It allows you to selectively measure how much time is spent in various scopes. Profiling data is generated and published to a ROS topic where it can be recorded or monitored in real time. The profiler was designed to be lightweight enough that it can be left in during normal operation so that performance data can be monitored at any time. ![swri_profiler](https://img.shields.io/github/stars/swri-robotics/swri_profiler.svg?style=flat&label=Star&maxAge=86400)
* [pal_statistics](https://github.com/pal-robotics/pal_statistics) - provides a way of gathering, aggregating, storing and visualizing statistics from arbitrary sources in a flexible and real-time safe way in ROS. From internal variables values to high level statistics about, but not limited to, robot performance [pal_statistics Wiki](http://wiki.ros.org/pal_statistics). ![pal_statistics](https://img.shields.io/github/stars/pal-robotics/pal_statistics.svg?style=flat&label=Star&maxAge=86400)
* [ROS Logs & ELK Stack](https://github.com/karadalex/roslogs-elk-docker) - demo of collecting ROS Logs (from ROS Containers) with Filebeat which are then sent to Logstash indexed by Elasticsearch and can then be viewed and visualized at Kibana. All logs are stored in the roslogs volume. ![ROS Logs & ELK Stack](https://img.shields.io/github/stars/karadalex/roslogs-elk-docker.svg?style=flat&label=Star&maxAge=86400)
* [ros2-migration-tools](https://github.com/awslabs/ros2-migration-tools) - contains a set of tools for migrating a ROS1 package to a ROS2 package. The C++ source code migration uses libclang8 and its corresponding python bindings. ![ros2-migration-tools](https://img.shields.io/github/stars/awslabs/ros2-migration-tools.svg?style=flat&label=Star&maxAge=86400)
* [ ROS Qt Creator Plug-in](https://ros-qtc-plugin.readthedocs.io/en/latest/index.html) - plug-in for Qt Creator to work with ROS workspaces, analyse performance and debug code. 


### Documentation

* [rosdoc_lite](http://wiki.ros.org/rosdoc_lite) - ROS package wraps documentation tools like doxygen, sphinx, and epydoc, making it convenient to generate ROS package documentation. It also generates online documentation for the ROS wiki. [github](https://github.com/ros-infrastructure/rosdoc_lite). ![rosdoc_lite](https://img.shields.io/github/stars/ros-infrastructure/rosdoc_lite.svg?style=flat&label=Star&maxAge=86400)
* [rosautodoc](https://github.com/bponsler/rosautodoc) - the rosautodoc project provides a python executable that can automatically generate documentation for ROS nodes that are running on the system. ![rosautodoc](https://img.shields.io/github/stars/bponsler/rosautodoc.svg?style=flat&label=Star&maxAge=86400)
* [roslaunch_to_dot](https://github.com/bponsler/roslaunch_to_dot) - convert a roslaunch XML file into a dot file containing a graph of the launch tree. ![roslaunch_to_dot](https://img.shields.io/github/stars/bponsler/roslaunch_to_dot.svg?style=flat&label=Star&maxAge=86400)

## Bag files

* [rosbag_fancy](https://github.com/xqms/rosbag_fancy) - fancy terminal UI frontend for the venerable rosbag tool ![rosbag_fancy](https://img.shields.io/github/stars/xqms/rosbag_fancy.svg?style=flat&label=Star&maxAge=86400)
* [rosbag-sliding-windows-annotator](https://github.com/ewerlopes/rosbag-sliding-windows-annotator) - Providing a way to annotate rosbag files by using the method of sliding windows (use a video image topic as a guide for tagging numerical data) ![rosbag-sliding-windows-annotator](https://img.shields.io/github/stars/ewerlopes/rosbag-sliding-windows-annotator.svg?style=flat&label=Star&maxAge=86400)
* [rosbag_editor](https://github.com/facontidavide/rosbag_editor) - GUI-app to remove one or more topics from a rosbag, change the duration of the rosbag, rename a topic, change the compression type etc. ![rosbag_editor](https://img.shields.io/github/stars/facontidavide/rosbag_editor.svg?style=flat&label=Star&maxAge=86400)
* [bag-database](https://github.com/swri-robotics/bag-database) - web-based application that monitors a directory for ROS bag files, parses their metadata, and provides a friendly web interface for searching for bags and downloading them. Its goal is to make it easy to catalog thousands of bag files, search through them for relevant data such as topic names and message types, view information about them, and download them. ![bag-database](https://img.shields.io/github/stars/swri-robotics/bag-database.svg?style=flat&label=Star&maxAge=86400)
* [rbb_core](https://github.com/AMZ-Driverless/rbb_core) - tool to index/visualize/manage rosbags on remote storage systems. Additionally it provides a web interface and framework for automated simulations. ![rbb_core-database](https://img.shields.io/github/stars/AMZ-Driverless/rbb_core.svg?style=flat&label=Star&maxAge=86400)
* [marv-robotics](https://github.com/KITcar-Team/marv-robotics) - MARV Robotics is an extensible data management platform for robot logs. New robot logs are found by a scanner and configured nodes are run to extract, filter and process data from them. The robot logs are visualized in a web-based application that features a listing view with filters and summary, and detail views of individual log files. ![marv-robotics](https://img.shields.io/github/stars/KITcar-Team/marv-robotics.svg?style=flat&label=Star&maxAge=86400)
* [RosbagPandas](https://github.com/aktaylor08/RosbagPandas) - create Python pandas data frame from a ros bag file. ![RosbagPandas](https://img.shields.io/github/stars/aktaylor08/RosbagPandas.svg?style=flat&label=Star&maxAge=86400)
* [rosbag_metadata](https://github.com/hordurk/rosbag_metadata) - tool for collecting and writing metadata to ROS bagfiles or to accompanying yaml files. ![rosbag_metadata](https://img.shields.io/github/stars/hordurk/rosbag_metadata.svg?style=flat&label=Star&maxAge=86400)
* [bag_tools](https://github.com/srv/srv_tools) - set of useful bag processing tools (make_video from topic, change_frame_id, change_camera_info, extract_stereo_images etc.). [bag_tools Wiki](https://wiki.ros.org/bag_tools?distro=kinetic#make_video.py). ![bag_tools](https://img.shields.io/github/stars/srv/srv_tools.svg?style=flat&label=Star&maxAge=86400)
* [rosbag_compress](https://github.com/AtsushiSakai/rosbag_compress) - a python command line tool for compression or decompression of multiple ROS bag files. This tool searchs bag files recrusively,compress or compress them at same time. It is executed in parallel process, so the task is done fastly. ![rosbag_compress](https://img.shields.io/github/stars/AtsushiSakai/rosbag_compress.svg?style=flat&label=Star&maxAge=86400)
* [rosbag_snapshot](https://github.com/ros/rosbag_snapshot) - subscribes to topics and maintains a buffer of recent messages like a dash cam. This is useful in live testing where unexpected events can occur which would be useful to have data on but the opportunity is missed if rosbag record was not running (disk space limits make always running rosbag record impracticable). Instead, users may run snapshot in the background and save data from the recent past to disk as needed. ![rosbag_snapshot](https://img.shields.io/github/stars/ros/rosbag_snapshot.svg?style=flat&label=Star&maxAge=86400)
* [batch_ros](https://github.com/lrse/batch_ros) - provides a framework for batch execution under ROS, to ensure a node consumes every message in a rosbag, allowing for repeatable results and ensure no message dropping. This not only allows for rigorous testing of a ROS node but also running tests on Continuous Integration (CI) based on rosbag data. ![batch_ros](https://img.shields.io/github/stars/lrse/batch_ros.svg?style=flat&label=Star&maxAge=86400)


## Visualization

* [rosshow](https://github.com/dheera/rosshow) - Displays various sensor messages in a useful fashion using Unicode Braille art in the terminal ![rosshow](https://img.shields.io/github/stars/dheera/rosshow.svg?style=flat&label=Star&maxAge=86400)
* [webviz](https://github.com/cruise-automation/webviz) - Web-based application for playback and visualization of ROS bag files. This repository also contains some libraries that can be used independently to build web-based visualization tools ![webviz](https://img.shields.io/github/stars/cruise-automation/webviz.svg?style=flat&label=Star&maxAge=86400)
* [mapviz](https://github.com/swri-robotics/mapviz) - ROS based visualization tool with a plug-in system similar to RVIZ focused on visualizing 2D data ![mapviz](https://img.shields.io/github/stars/swri-robotics/mapviz.svg?style=flat&label=Star&maxAge=86400)
* [rqt_multiplot_plugin](https://github.com/anybotics/rqt_multiplot_plugin) - GUI rqt plugin for visualizing numeric values in multiple 2D plots using the Qwt plotting backend ![rqt_multiplot_plugin](https://img.shields.io/github/stars/ANYbotics/rqt_multiplot_plugin.svg?style=flat&label=Star&maxAge=86400)
* [PlotJuggler](https://github.com/facontidavide/PlotJuggler) - QT5 based application to display time series in plots, using an intuitive "drag and drop" interface It can be used either to load static data from file or connect to live streaming of data ![PlotJuggler](https://img.shields.io/github/stars/facontidavide/PlotJuggler.svg?style=flat&label=Star&maxAge=86400)
* [rviz_satellite](https://github.com/gareth-cross/rviz_satellite) - Plugin for rviz for displaying satellite maps loaded from the internet. ![rviz_satellite](https://img.shields.io/github/stars/gareth-cross/rviz_satellite.svg?style=flat&label=Star&maxAge=86400)


## Code Testing

* [hypothesis-ros](https://github.com/fkromer/hypothesis-ros) - data generators for Property Based Testing and Fuzzy Testing of ROS nodes (Unmantained!) ![hypothesis-ros](https://img.shields.io/github/stars/fkromer/hypothesis-ros.svg?style=flat&label=Star&maxAge=86400)
* [ros1_fuzzer](https://github.com/aliasrobotics/ros1_fuzzer) - Fuzzer aims to help developers and researchers to find bugs and vulnerabilities in ROS nodes by performing fuzz tests over topics that the target nodes process ![ros1_fuzzer](https://img.shields.io/github/stars/aliasrobotics/ros1_fuzzer.svg?style=flat&label=Star&maxAge=86400) 
* [roschaos](https://github.com/fkromer/roschaos) - functionality for process reliability/fault recovery testing in ROS ![hypothesis-ros](https://img.shields.io/github/stars/fkromer/roschaos.svg?style=flat&label=Star&maxAge=86400)
* [code_coverage](https://github.com/mikeferguson/code_coverage) - ROS package to run coverage testing. [Introduction.](http://www.robotandchisel.com/2020/04/07/code-coverage-for-ros/) ![code_coverage](https://img.shields.io/github/stars/mikeferguson/code_coverage.svg?style=flat&label=Star&maxAge=86400)
* [Ros-Test-Example](https://github.com/steup/Ros-Test-Example) - a ROS Workspace containing an example car simulation to show GTest and Rostest. ![Ros-Test-Example](https://img.shields.io/github/stars/steup/Ros-Test-Example.svg?style=flat&label=Star&maxAge=86400)

## Simulation

* [CARLA](https://carla.org/) - an open-source simulator for autonomous driving research. [github](https://github.com/carla-simulator/carla) ![carla](https://img.shields.io/github/stars/carla-simulator/carla.svg?style=flat&label=Star&maxAge=86400)
* [World Construction Tool](https://gitlab.com/LIRS_Projects/LIRS-WCT) - automatic tool for gazebo world construction: from a grayscale image to a 3d solid model.
* [Dataset of Gazebo Worlds Models and Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps) - a set of Gazebo worlds models and maps. ![Dataset of Gazebo Worlds Models and Maps](https://img.shields.io/github/stars/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps.svg?style=flat&label=Star&maxAge=86400)
* [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) - central repository for tools, tutorials, resources, and documentation for robotic simulation in Unity including integration with ROS. ![Unity Robotics Hub](https://img.shields.io/github/stars/Unity-Technologies/Unity-Robotics-Hub.svg?style=flat&label=Star&maxAge=86400)
* [xmacro](https://github.com/gezp/xmacro) - looks like a simplified version of ros/xacro, it's simpler, but it works well both for urdf and sdf. In addition it's flexible, and also easy to use. ![xmacro](https://img.shields.io/github/stars/gezp/xmacro.svg?style=flat&label=Star&maxAge=86400)


## Hardware

* [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR/) - Open Source Control Module for ROS. OpenCR is developed for ROS embedded systems to provide completely open-source hardware and software. ![OpenCR](https://img.shields.io/github/stars/ROBOTIS-GIT/OpenCR.svg?style=flat&label=Star&maxAge=86400)
* [DRV8825_ROS](https://github.com/jstiefel/DRV8825_ROS) - uses an Arduino Uno Rev.3 as a ROS node to control a stepper motor by using DRV8825 motor driver. ![DRV8825_ROS](https://img.shields.io/github/stars/jstiefel/DRV8825_ROS.svg?style=flat&label=Star&maxAge=86400)

### Sensors

* [imu_tools](https://github.com/ccny-ros-pkg/imu_tools) - IMU-related filters and visualizers including Madgwick filter, Complementary filter and rviz IMU plugin for visualizing `sensor_msgs::Imu` messages. ![imu_tools](https://img.shields.io/github/stars/ccny-ros-pkg/imu_tools.svg?style=flat&label=Star&maxAge=86400)
* [ros_imu_covariance_calculator](https://github.com/Myzhar/ros_imu_covariance_calculator) - ROS Package to estimate the variance of the inertial data from an IMU to be used to populate the error covariance matrix. ![ros_imu_covariance_calculator](https://img.shields.io/github/stars/Myzhar/ros_imu_covariance_calculator.svg?style=flat&label=Star&maxAge=86400)
* [kalibr](https://github.com/ethz-asl/kalibr) - Kalibr is a toolbox that solves the following calibration problems:
     1. **Multiple camera calibration**: 
         intrinsic and extrinsic calibration of a camera-systems with non-globally shared overlapping fields of view
     1. **Visual-inertial calibration calibration (camera-IMU)**:
         spatial and temporal calibration of an IMU w.r.t a camera-system
     1. **Rolling Shutter Camera calibration**:
         full intrinsic calibration (projection, distortion and shutter parameters) of rolling shutter cameras. ![kalibr](https://img.shields.io/github/stars/ethz-asl/kalibr.svg?style=flat&label=Star&maxAge=86400)

## Web Tools

* [Robot Web Tools](http://robotwebtools.org/index.html) - robot web tools is a collection of open-source modules and tools for building web-based robot apps.
* [web_video_server](https://github.com/RobotWebTools/web_video_server) - HTTP Streaming of ROS Image Topics in Multiple Formats. ![web_video_server](https://img.shields.io/github/stars/RobotWebTools/web_video_server.svg?style=flat&label=Star&maxAge=86400)
* [ros_rtsp](https://github.com/CircusMonkey/ros_rtsp) - ROS package to subscribe to an ROS Image topic (and as many other video sources as you want) and serve it up as a RTSP video feed with different mount points. Should provide a real-time video feed (or as close as possible). ![ros_rtsp](https://img.shields.io/github/stars/CircusMonkey/ros_rtsp.svg?style=flat&label=Star&maxAge=86400)

## Other

* [multimaster_fkie](https://github.com/fkie/multimaster_fkie) - the ROS stack of fkie_multimaster offers a complete solution for using ROS with multicores. In addition, Node Manager with a daemon provide a GUI-based management environment that is very useful to manage ROS-launch configurations and control running nodes, also in a single-core system. ![multimaster_fkie](https://img.shields.io/github/stars/fkie/multimaster_fkie.svg?style=flat&label=Star&maxAge=86400)
* [rosmon](https://github.com/xqms/rosmon) - rosmon is a drop-in replacement for the standard roslaunch tool. Rather unlike roslaunch, rosmon is focused on (remote) process monitoring. [rosmon Wiki](http://wiki.ros.org/rosmon). ![rosmon](https://img.shields.io/github/stars/xqms/rosmon.svg?style=flat&label=Star&maxAge=86400)
* [aruco_pose](https://github.com/CopterExpress/clover/tree/master/aruco_pose) - aruco_pose package consists of two nodelets: `aruco_detect` detects individual ArUco-markers and estimates their poses, `aruco_map` detects maps of markers using aruco_detect output. ![aruco_pose](https://img.shields.io/github/stars/master/aruco_pose.svg?style=flat&label=Star&maxAge=86400)

## License

[![CC0](http://i.creativecommons.org/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
