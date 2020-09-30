# Awesome ROS Tools 
[![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

The list of tools and packages for Robot Operating System development!

## Table of Contents

* [Best practices](#best_practices)
* [Development tools](#development-tools)
* [Bag files](#bag-files)
* [Visualization](#visualization)
* [Code testing](#code-testing)
* [Simulation](#simulation)
* [Sensors](#sensors)
* [Web tools](#web-tools)

## Best Practices

* [ros_best_practices](https://github.com/leggedrobotics/ros_best_practices) - loose collection of best practices, conventions, and tricks for using the Robot Operating System (ROS). It builds up on the official ROS documentation and other resources and is meant as summary and overview ![ros_best_practices](https://img.shields.io/github/stars/leggedrobotics/ros_best_practices.svg?style=flat&label=Star&maxAge=86400)

## Development Tools

* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html) - command line tools for working with catkin [github](https://github.com/catkin/catkin_tools) ![catkin_tools](https://img.shields.io/github/stars/catkin/catkin_tools.svg?style=flat&label=Star&maxAge=86400)
* [ROSCpp Code Format](https://github.com/davetcoleman/roscpp_code_format) - the repo contains an auto formatting script for the [ROS C++ Style Guidelines](http://wiki.ros.org/CppStyleGuide) ![roscpp_code_format](https://img.shields.io/github/stars/davetcoleman/roscpp_code_format.svg?style=flat&label=Star&maxAge=86400)
* [swri_profiler](https://github.com/swri-robotics/swri_profiler) - is a lightweight profiling framework for C++ ROS nodes. It allows you to selectively measure how much time is spent in various scopes. Profiling data is generated and published to a ROS topic where it can be recorded or monitored in real time. The profiler was designed to be lightweight enough that it can be left in during normal operation so that performance data can be monitored at any time.(http://wiki.ros.org/CppStyleGuide) ![swri_profiler](https://img.shields.io/github/stars/swri-robotics/swri_profiler.svg?style=flat&label=Star&maxAge=86400)

## Bag files

* [rosbag_fancy](https://github.com/xqms/rosbag_fancy) - Fancy terminal UI frontend for the venerable rosbag tool ![rosbag_fancy](https://img.shields.io/github/stars/xqms/rosbag_fancy.svg?style=flat&label=Star&maxAge=86400)
* [rosbag-sliding-windows-annotator](https://github.com/ewerlopes/rosbag-sliding-windows-annotator) - Providing a way to annotate rosbag files by using the method of sliding windows (use a video image topic as a guide for tagging numerical data) ![rosbag-sliding-windows-annotator](https://img.shields.io/github/stars/ewerlopes/rosbag-sliding-windows-annotator.svg?style=flat&label=Star&maxAge=86400)
* [rosbag_editor](https://github.com/facontidavide/rosbag_editor) - GUI-app to remove one or more topics from a rosbag, change the duration of the rosbag, rename a topic, change the compression type etc. ![rosbag_editor](https://img.shields.io/github/stars/facontidavide/rosbag_editor.svg?style=flat&label=Star&maxAge=86400)
* [bag-database](https://github.com/swri-robotics/bag-database) - web-based application that monitors a directory for ROS bag files, parses their metadata, and provides a friendly web interface for searching for bags and downloading them. Its goal is to make it easy to catalog thousands of bag files, search through them for relevant data such as topic names and message types, view information about them, and download them. ![bag-database](https://img.shields.io/github/stars/swri-robotics/bag-database.svg?style=flat&label=Star&maxAge=86400)
* [rbb_core](https://github.com/AMZ-Driverless/rbb_core) - a tool to index/visualize/manage rosbags on remote storage systems. Additionally it provides a web interface and framework for automated simulations. ![rbb_core-database](https://img.shields.io/github/stars/AMZ-Driverless/rbb_core.svg?style=flat&label=Star&maxAge=86400)
* [marv-robotics](https://github.com/KITcar-Team/marv-robotics) - MARV Robotics is an extensible data management platform for robot logs. New robot logs are found by a scanner and configured nodes are run to extract, filter and process data from them. The robot logs are visualized in a web-based application that features a listing view with filters and summary, and detail views of individual log files. ![marv-robotics](https://img.shields.io/github/stars/KITcar-Team/marv-robotics.svg?style=flat&label=Star&maxAge=86400)
* [RosbagPandas](https://github.com/aktaylor08/RosbagPandas) - create Python pandas data frame from a ros bag file. ![RosbagPandas](https://img.shields.io/github/stars/aktaylor08/RosbagPandas.svg?style=flat&label=Star&maxAge=86400)
* [rosbag_metadata](https://github.com/hordurk/rosbag_metadata) - tool for collecting and writing metadata to ROS bagfiles or to accompanying yaml files. ![rosbag_metadata](https://img.shields.io/github/stars/hordurk/rosbag_metadata.svg?style=flat&label=Star&maxAge=86400)

## Visualization

* [rosshow](https://github.com/dheera/rosshow) - Displays various sensor messages in a useful fashion using Unicode Braille art in the terminal ![rosshow](https://img.shields.io/github/stars/dheera/rosshow.svg?style=flat&label=Star&maxAge=86400)
* [webviz](https://github.com/cruise-automation/webviz) - Web-based application for playback and visualization of ROS bag files. This repository also contains some libraries that can be used independently to build web-based visualization tools ![webviz](https://img.shields.io/github/stars/cruise-automation/webviz.svg?style=flat&label=Star&maxAge=86400)
* [mapviz](https://github.com/swri-robotics/mapviz) - ROS based visualization tool with a plug-in system similar to RVIZ focused on visualizing 2D data ![mapviz](https://img.shields.io/github/stars/swri-robotics/mapviz.svg?style=flat&label=Star&maxAge=86400)
* [rqt_multiplot_plugin](https://github.com/anybotics/rqt_multiplot_plugin) - GUI rqt plugin for visualizing numeric values in multiple 2D plots using the Qwt plotting backend ![rqt_multiplot_plugin](https://img.shields.io/github/stars/ANYbotics/rqt_multiplot_plugin.svg?style=flat&label=Star&maxAge=86400)
* [PlotJuggler](https://github.com/facontidavide/PlotJuggler) - QT5 based application to display time series in plots, using an intuitive "drag and drop" interface It can be used either to load static data from file or connect to live streaming of data ![PlotJuggler](https://img.shields.io/github/stars/facontidavide/PlotJuggler.svg?style=flat&label=Star&maxAge=86400)

## Code Testing

* [hypothesis-ros](https://github.com/fkromer/hypothesis-ros) - data generators for Property Based Testing and Fuzzy Testing of ROS nodes (Unmantained!) ![hypothesis-ros](https://img.shields.io/github/stars/fkromer/hypothesis-ros.svg?style=flat&label=Star&maxAge=86400)
* [ros1_fuzzer](https://github.com/aliasrobotics/ros1_fuzzer) - Fuzzer aims to help developers and researchers to find bugs and vulnerabilities in ROS nodes by performing fuzz tests over topics that the target nodes process ![ros1_fuzzer](https://img.shields.io/github/stars/aliasrobotics/ros1_fuzzer.svg?style=flat&label=Star&maxAge=86400) 
* [roschaos](https://github.com/fkromer/roschaos) - functionality for process reliability/fault recovery testing in ROS ![hypothesis-ros](https://img.shields.io/github/stars/fkromer/roschaos.svg?style=flat&label=Star&maxAge=86400)

## Simulation

* [CARLA](https://carla.org/) - an open-source simulator for autonomous driving research [github](https://github.com/carla-simulator/carla) ![carla](https://img.shields.io/github/stars/carla-simulator/carla.svg?style=flat&label=Star&maxAge=86400)
* [World Construction Tool](https://gitlab.com/LIRS_Projects/LIRS-WCT) - Automatic tool for gazebo world construction: from a grayscale image to a 3d solid model

## Sensors

* [imu_tools](https://github.com/ccny-ros-pkg/imu_tools) - IMU-related filters and visualizers including Madgwick filter, Complementary filter and rviz IMU plugin for visualizing `sensor_msgs::Imu` messages ![imu_tools](https://img.shields.io/github/stars/ccny-ros-pkg/imu_tools.svg?style=flat&label=Star&maxAge=86400)

## Web Tools
* [Robot Web Tools](http://robotwebtools.org/index.html) - robot web tools is a collection of open-source modules and tools for building web-based robot apps.

## License

[![License: CC0-1.0](https://img.shields.io/badge/License-CC0%201.0-lightgrey.svg)](http://creativecommons.org/publicdomain/zero/1.0/)
