#                                                  Stop Behavior 
## What is the Stopping behavior node?

- A new package is to be created that will handle stopping behavior.  It will be code that is implemented when the car is told to stop due to an incoming stop sign or object. 
- This code will be part of a larger node in the navigation stack that decides whether the car should stop or continue its normal pursuit based on incoming data about the surroundings.  We will refer to the larger node as the *‘arbiter’*.  
When the arbiter decides that the car needs to stop, this stop behavior code wil be implemented

## How will it work?

- The code will tell the car to start slowing down beggining at a certain distance and so that the car comes to a rest at the required point.  
- The entry action for starting this code will be a stop sign detected
- If a stop sign or stop point is detected, this code will run
- We will start off simple by creating a stop point and defining it beforehand in the code.  
- In the future, the car will see a stop sign or other object and then detect the distance of the stop point to the car using camera/lidar sensors.  
- We might be able to take code from the pure pursuit and modify it so the car slows to a stop at the correct point.  

## How will it fit in with the rest off the navigation stack?

- The arbiter will go in-between the pure pursuit and the Carla-ROS bridge/the actual car
- If there is no need to stop, the arbiter will pass through the pure pursuit commands to the Carla-ROS bridge/Car
- If there is a need to stop, the arbiter will figure this out based on incoming information about the surroundings like lidar/machine learning with camera 
- When it is decided that the car needs to stop, the stop behavior code is implemented
- After the arbiter determines the car has stopped for the proper amount of time, it will switch the behavior from stop behavior back to pure pursuit and the car will start driving again.  


## Resources

- This [github page](https://github.com/qiaoxu123/Self-Driving-Cars/blob/master/Part4-Motion_Planning_for_Self-Driving_Cars/Module5-Principles_of_Behaviour_Planning/Module5-Principles_of_Behaviour_Planning.md) on Behavioral Planning and finite state machines
- - A finite state machine is what the arbiter will be like

## Ideas

- Custom message type for stop behavior
- Be able to determine a certain distance away from the stop point where the car will begin slowing down
- The distance will be bigger or smaller depending on the speed of the car
- Have a separate node that tells the car to stay stopped and to start back up when safe
- Include option for emergency stop
- May need separate state machines for different stop instances or intersection types


## Questions/Stuff to figure out
- How about the behavior when the car transitions from stop to pure pursuit?
- What should be my workspace? And should I do the Carla-Ros bridge thing?
- How to deal with environmental noise or the car not being exactly stopped or at stop point
- - Having a tolerance
- How to deal with edge cases that require emergency stops
- Determining the proper/safest way for the car to accelerate or decelerate
accelerometer?
