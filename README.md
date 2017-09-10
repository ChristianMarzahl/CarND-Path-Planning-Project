# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

In the following I will adress the [Rubic](https://review.udacity.com/#!/rubrics/1020/view) points of this Path Planning Project

## Introduction
The target of this project was to drive a car safely around a circular track which has a length of 4.32 miles. To be successful the car was not allowed to:
1. Collide with another car
2. Leave the right three lines
3. The car has to stay within its current line, except for reasonable overtaking maneuvers
4. Do not drive over 50mph
5. Do not make movements that have a too high acceleration or jerk

My final submission code was able to drive the car safely around the track multiple times. The following video shows the first round with multiple line chances and an emergency brake after a car in front of me changed into my line.

<a href="https://youtu.be/TWGzA-gpgVI" target="_blank"><img src="http://img.youtube.com/vi/TWGzA-gpgVI/0.jpg"  alt="First Track" width="720" height="360" border="10" /></a>

## Path planning

### Speed Limit
To drive the car within the speed limit of 50mph I set the speed limit for the car slightly below 50 MPH to 49.75 MPH. I tested the code with higher velocity and up to 60mph it worked fine, except for the message that my car was violating the speed limit.  
```cpp
  // maximal velocity!
  double max_vel = 49.75;
``` 

### Max Acceleration and Jerk
Like the speed limit, the max acceleration and jerk are handled by the spline approximation technique which is exceptionally good explained in the walkthrough video of the course. After following the provided code hints from the video and using the provided spline technique from this [source](http://kluge.in-chemnitz.de/opensource/spline/) the  the acceleration and jerk was in range at all times .

### Staying in line and line changes for overtaking maneuvers
My approach to decide when to stay in line and when to overtake was the following:

1. Calculate the distance to my car for each car in each line. This was done by using the car's [s] position in frenet coordinates
2. For performance reasons I saved only the distance to the car right in front of me and behind my car for each line. If there is no car on the lane the distance is set to 999. That value is used for the cost function again later. 
3. If the distance to the car in front of me is closer that 30. The car will be flagged as to close.
```cpp
   // count car´s per lane 
   vector<int> lane_car_count={0,0,0};
   // for lane 0,1,2 save distance to nearest car in front of the one car
   vector<int> lane_car_distlist_front = {999, 999, 999};
   // for lane 0,1,2 save distance to nearest car in the back of the one car
   vector<int> lane_car_distlist_back = {-999, -999, -999};
   
....

   // check if the car is in front of me and distance is smaler that the smalest none for that lane
   if (car_distance > 0 && lane_car_distlist_front[other_lane] > car_distance)
   {
      lane_car_distlist_front[other_lane] = car_distance
   }
   else if (car_distance < 0 && lane_car_distlist_back[other_lane] < car_distance)
      lane_car_distlist_back[other_lane] = car_distance;
``` 
After these three steps the debug output shows the following values:
current lane: 2 // the lane of my car
too close! // the car in front of me, on the same lane, is to close 
5	 	 4	 	3 // on lane zero are 5 cars, on lane one are 4 cars and on lane 2 are 3 cars.
7 -89	 | 3 -51	 | 29 -75 // On lane 0 the next car is 7 in front of mine and 89 behind, etc...

4. If the "to close" flag rises I check two options: 
4.1. Is it save to overtake? First, I check if if the gap between the car in front and behind my car on the neighbouring lanes. If the gap on the other lane is big enough and the leading car on the other lane is further away that the car on my lane the car will change lines. If I am on the center lane I check both lanes and choose the lane on which the next car in front is further away. If both criteria are fulfilled I change to the calculated best lane.
```cpp
          if (too_close) {
            vector<int> possible_lanes;
            if (lane == 0)
              possible_lanes = {1};
            else if (lane == 1)
              possible_lanes ={0,2};
            else
              possible_lanes ={1};

            int best_lane=lane;
            double best_cost=numeric_limits<double>::max();
            for (int check_lane: possible_lanes) {
              double cost = numeric_limits<double>::max();

              // check if gap is big enough
              if (lane_car_distlist_front[check_lane] > 70
                  && lane_car_distlist_back[check_lane] < -30)
                cost = 999 - lane_car_distlist_front[check_lane];

              if (cost < best_cost) {
                best_lane = check_lane;
                best_cost = cost;
              }
            }

            // if we are close and stay on this line reduce speed
            if (best_lane == lane)
              ref_vel -= .224;
            else // change to best lane
            {
              cout << "change lane from " << lane << "to " <<  best_lane << endl;
              lane = best_lane;
            }
          } else if (ref_vel < max_vel)  // if the lane is free accelerate speed
          {
            ref_vel += .224;
          }
``` 
4.2. If it is not save to overtake I stay on the current lane and reduce speed. 

### Further improvments
1) The model is too simple for the real world. It cannot handle complex maneuvers like falling behind and then start an overtaking maneuver
2) If the car is behind another car and cannot overtake it is not set to the speed of the car in front. Instead the car decelerate if it is to close and accelerate if it is too far behind. It would be more efficient to set the speed to speed of the car in front. 

   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

