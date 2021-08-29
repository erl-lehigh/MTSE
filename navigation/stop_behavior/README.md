#                                                  Stop Behavior 
## What is the Stopping behavior node?

- A new package (MTSE/navigation/stop_behavior) is to be created that will handle stopping behavior.  It will be code that is implemented when the car is told to stop due to an incoming stop sign or object. 
- The first node in this package will be called stop_behavior_node.py
    - It will be called upon when the car needs to stop at a certain point
- I think this branch will probably contain all of this stuff even though the name of the branch is stop_behavior
- Check out [this Diagram](https://docs.google.com/drawings/d/1sFHjYEnZVSR-rMrqaq3gTCAwTR4kKjkosON6w0FDwM4/edit)

## How will it work?

- The code will tell the car to start slowing down beggining at a certain distance so that the car safely comes to a rest at the required point.  
    - The entry action for starting this code will be a stop sign detected
- We will start off simple by creating a stop point and defining it beforehand in the code.  
    - In the future, the car will see a stop sign or other object and then detect the distance of the stop point to the car using camera/lidar sensors.  
- We might be able to take code from the pure pursuit and modify the speed commands to slow the car down 

## How will it fit in with the rest off the navigation stack?

- The Pure pursuit and Stopping behavior package will be parallel to each other and both flow into the “multiplexor”
    - The multiplexor is a thing that takes in multiple different commands and outputs only the one command that needs to be executed
- If there is no need to stop, the multiplexor will pass through the pure pursuit commands to the Carla/Car
- If there is a need to stop, the multiplexor will know this based on incoming information
    - When it is decided that the car needs to stop, the stop behavior code is implemented
- After the car has stopped for the proper amount of time and it is safe, it will switch the behavior from stop behavior back to pure pursuit and the car will start driving again.  


## Resources

- This [github page](https://github.com/qiaoxu123/Self-Driving-Cars/blob/master/Part4-Motion_Planning_for_Self-Driving_Cars/Module5-Principles_of_Behaviour_Planning/Module5-Principles_of_Behaviour_Planning.md) on Behavioral Planning and finite state machines
    - A finite state machine is what the multiplexor will be like

## Ideas

- Custom message type for stop behavior
- Be able to determine a certain distance away from the stop point where the car will begin slowing down
    - The distance will be bigger or smaller depending on the speed of the car
- Have a separate node that tells the car to stay stopped and to start back up when safe
- Include option for emergency stop
- May need separate state machines for different stop instances or intersection types


## Questions/Stuff to figure out
- How about the behavior when the car transitions from stop to pure pursuit?
- How to deal with environmental noise or the car not being exactly stopped or at stop point
    - Having a tolerance
- How to deal with edge cases that require emergency stops
- Determining the proper/safest way for the car to accelerate or decelerate
 
# See the multiplexor packaage readme for running the multiplexor and the stop behavior along side it