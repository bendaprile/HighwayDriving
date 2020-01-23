# Model Documentation

1. Prediction: This is the first step of my path planner and it discovers the cars that are around it. 

	a. First I loop through the cars in the sensor fusion vector and figure out which lane they are in. 
    
    b. I then grab the x, y, speed, and s location of each car. These variables will help us determine where the are and will be in the 	future.
    
    c. I then go through some if statements checking what lane the car is in. If it is in the lane in front of us and within 30m of us  	then I set the car_ahead bool to true. If the car is in the lane to the right or left of us and within 30m ahead or 15m behind us   	then I set that repsective boolean to true. These will help us during the decision during lane and speed changes.
    
2. Behavior Planning: This is the step where I decide what the car should do next with the given information.

	a. I have a simplified if structure to decide what the car should do next. This could be added onto in addition to the prediction   	step to drive more efficiently.
    
    b. If there is no car ahead and we are still under the speed limit then the car will increase its speed else...
    
    c. If there is a car ahead then we will start to look for a lane change.
    
    d. If there is no car in the left lane then we will lane change left, else if there is no car in the right lane then we will lane   	change right. If there happens to be a car in both lanes then we will keep lane and reduce our speed to stay behind the car in 			front of us. The car will reduce speed more and more as it gets closer to the car in front.
    
3. Trajectory Generation: This is the step where we actually generate trajectories for the car to follow.

	a. Start by appending the previous points onto our new trajectory to continue our last trajectory. If there are less than two 			points then we just append them without worrying about which direction we were headed. If there were more than two points then we   	can assume which direction we were headed so we compute that.
    
    b. We then add evenly spaced points every 30m out so at 30m, 60m, and 90m from the starting reference points. This is done in 			Frenet.
    
    c. Then we shift the reference angle by 45 degrees so it seems that the car is facing 0 degrees or forward. This shifts the frame   	of reference for our car.
    
    d. Now we start to construct our spline using the spline library. We start by appending all of the previous points from the 			previous path to the spline. Then we calculate how to space the points along the spline to stay at our target speed and append our  	new generated points.
    
    e. Finally we convert our local coordinates back into global coordinates and send the points to the car to drive along.