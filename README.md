# Turtlebot orientation to Polaris
This is a ROS package to orientate a turtlebot3 to Polaris (North Start) automatically in python.

## Description
For this task I proposed the use of a GPS plugin in Gazebo [hector gps gazebo plugin](http://wiki.ros.org/hector_gazebo_plugins) to simulate gps data (latitude,longitude). The main idea is to localize the robot using GPS data and then, find the bearing/heading angle between two points (see image below).


<img src="./examples/example.jpg" align="left"><br> <img src="./examples/example2.png" align="right"><br>

Given an initial pose, the turtlebot will generate (latitude,longitude) data which will be use to estime the bearing angle. According to GISMAP (https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/), the process to calculate the bearing angle is the following:

Bearing from point A to B, can be calculated as:

```
β = atan2(X,Y),
```
where, `X` and `Y` are two quantities and can be calculated as:

```
X = cos θb * sin ∆L

Y = cos θa * sin θb – sin θa * cos θb * cos ∆L
```

For this task, 

`L` be the longitude,
`θ` be latitude,
`β` be Bearing.


## Implementation
In this step, I created 3 ROS packages:

1.	hector_gazebo_plugins:
This ROS package contain the gps plugin module that was used to simulate (latitude,longitude) data in gazebo.

2.	turtlebot3_gazebo
This is the ROS package for the turtlebot3 model. In this package there is an urdf decription file for the robot, where I added the "hector gps" gazebo plugin. 


3.	turtlebot3_motion_planning
This is the ROS/Python package to estimate the orientation for the robot based in the gps coordinates from the Polaris north start and the gps coordinates from the robot.

## Bearing method calculation
For this process, I assumed polaris north start latitude=90 and longitud=90, according to the image below.

<img src="./examples/example4.png" align="center"><br>

where polaris is the true north with lat,lon = 90 degrees.

