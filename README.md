# Path Planning Project
In this project, I used C++ to write a program that uses prediction, behavior, and trajectory to navigate around a simulated test track. 

## Project Info
For a great introduction to the topic, check out [paper](https://arxiv.org/pdf/1604.07446.pdf).

To see the implementation please visit the following file in the 'src' folder:

1. main.cpp
2. spline.h

## Setup instructions
FYI, I ran my code on a Macbook Pro. Please ensure you have downloaded the Udacity SDCND Term 3 Simulator [here](https://github.com/udacity/self-driving-car-sim/releases/) and have installed cmake (3.5), make (4.1), and gcc/gcc+ (5.4).

1. Open Terminal
2. `git clone https://github.com/tlapinsk/CarND-Path-Planning-Project.git`
3. `cd CarND-Path-Planning-Project`
4. `sh install-mac.sh`
5. `mkdir build && cd build`
6. `cmake`
7. `make`
8. `./path-planning`
9. Run the term3_sim application, select Project 5, and click 'Start'

## Overview
By using data sent by the simulator (car's position and velocity) and sensor fusion data about the rest of the traffic, we can direct the car to perform safe lane changes and speed up/slow down when necessary. More detailed explanation will be given of the code implemented below.

## Reflection
The project provided the majority of the code needed to complete this project. The algorithm I used to complete this project can be found between lines 247-347 in src/main.cpp. I have loosely broken the algorithm into three parts: prediction, behavior, and trajectory.

### Prediction
This section of the code deals with the sensor fusion data that we are collecting about other vehicles on the track. There are three main components that we must be aware of to make safe decisions:

1. Is a car in front of us obstructing our path forward?
2. Is a car to the right of us blocking a safe lane change?
3. Is a car to the left of us blocking a safe lane change?

```
// Prediction: Gather and sift through traffic data
for (int i = 0; i < sensor_fusion.size(); i++) {
  // Get traffic data
  float d = sensor_fusion[i][6];
  double vx = sensor_fusion[i][3];
  double vy = sensor_fusion[i][4];
  double check_speed = sqrt(vx*vx+vy*vy);
  double check_car_s = sensor_fusion[i][5];
  
  // Analyze lane positioning
  int check_lane = -1;

  if (d > 0 && d < 4) 
  {
    check_lane = 0;
  } 
  else if (d > 4 && d < 8) 
  {
    check_lane = 1;
  } 
  else if (d > 8 && d < 12) 
  {
    check_lane = 2;
  }
  if (check_lane < 0) 
  {
    continue;
  }

  // Estimate s position
  check_car_s += ((double)prev_size*.02*check_speed);

  // Keep 30m between my car and car in front of me
  if (check_lane == lane && (check_car_s > car_s) && ((check_car_s - car_s) < 30)) 
  {
    too_close = true;
  }

  // Check if car in front of me is going faster or blocking me
  if (check_speed > car_speed || ((check_car_s - car_s > -10) && (check_car_s - car_s < 30))) 
  {
    empty_lane[check_lane] = false;
  }
}
```

This chunk of code (lines 261-304) helps us determine which lane we are currently in and if there is a car in front of us.

### Behavior
This section helps determine if we should do the following:

1. Speed up or slow down
2. Change lanes

Check out the code below.

```
// Behavior: Give car instructions
if (too_close) 
{
  // If there is a car in front of me, try to change lanes. If not, slow down
  if (lane + 1 <= 2 && empty_lane[lane + 1]) 
  {
    lane++;
  } 
  else if (lane - 1 >= 0 && empty_lane[lane - 1]) 
  {
    lane--;
  } 
  else 
  {
    ref_vel -= 0.224;
  }
}
// Speed back up
else if (ref_vel < 49.5) 
{
  ref_vel += 0.224;
}
```

This code lives between ines 306-327. You can see that the car checks if a lane is empty before moving right/left. In the worst case scenario, it does a speed check to see if the car can increase velocity.

### Trajectory
Most of the trajectory code was built by David and Aaron in the project walk through. This code resides between lines 329-427 and bases trajectory on the car's speed, lane, and path points.

By using past points and three future points (30, 60, and 90m ahead), we can initialize a spline calculation. There are some small snippets of code to handle changing coordinates as well. We then take the past points and calculate how to breakup the spline points (lines 409-414) and then fill up the rest of the points (always to 50 points).

The final piece is to rotate the coordinates back and then push the points to the car (lines 428-437). 

## Resources
Shoutout to the tutorials provided by Udacity on Path Planning and the Path Planning Walkthrough. The Path Planning Walkthrough was especially helpful to get me started. It didn't take much to complete the project after watching the Walkthrough a few times. Below are resources and helpful links that I used to complete this project:

- [A Survey of Motion Planning and Control
Techniques for Self-driving Urban Vehicles](https://arxiv.org/pdf/1604.07446.pdf)
- [Spline library](http://kluge.in-chemnitz.de/opensource/spline/)
- [Path Planning Walkthrough](https://www.youtube.com/watch?time_continue=1628&v=7sI3VHFPP0w)