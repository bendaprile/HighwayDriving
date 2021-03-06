# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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

## Model Documentation

1. Prediction: This is the first step of my path planner and it discovers the cars that are around it. 

	a. First I loop through the cars in the sensor fusion vector and figure out which lane they are in. 
    
    b. I then grab the x, y, speed, and s location of each car. These variables will help us determine where the are and will be in the 	future.
    
    c. I then go through some if statements checking what lane the car is in. If it is in the lane in front of us and within 30m of us  	then I set the car_ahead bool to true. If the car is in the lane to the right or left of us and within 30m ahead or 15m behind us   	then I set that repsective boolean to true. These will help us during the decision during lane and speed changes.
    
2. Behavior Planning: This is the step where I decide what the car should do next with the given information.

	a. I have a simplified if structure to decide what the car should do next. This could be added onto in addition to the prediction   	step to drive more efficiently.
    
    b. If there is no car ahead and we are still under the speed limit then the car will increase its speed else...
    
    c. If there is a car ahead then we will start to look for a lane change.
    
    d. If there is no car in the left lane then we will lane change left, else if there is no car in the right lane then we will lane   	change right. If there happens to be a car in both lanes then we will keep lane and reduce our speed to stay behind the car in 		front of us. The car will reduce speed more and more as it gets closer to the car in front.
    
3. Trajectory Generation: This is the step where we actually generate trajectories for the car to follow.

	a. Start by appending the previous points onto our new trajectory to continue our last trajectory. If there are less than two 		points then we just append them without worrying about which direction we were headed. If there were more than two points then we   	can assume which direction we were headed so we compute that.
    
    b. We then add evenly spaced points every 30m out so at 30m, 60m, and 90m from the starting reference points. This is done in 		Frenet.
    
    c. Then we shift the reference angle by 45 degrees so it seems that the car is facing 0 degrees or forward. This shifts the frame   	of reference for our car.
    
    d. Now we start to construct our spline using the spline library. We start by appending all of the previous points from the 		previous path to the spline. Then we calculate how to space the points along the spline to stay at our target speed and append our  	new generated points.
    
    e. Finally we convert our local coordinates back into global coordinates and send the points to the car to drive along.