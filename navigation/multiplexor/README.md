# The multiplexor

 - takes in commands from pure puresuit and stop
 - outputs one of them depending on whether there is a sign or not
 - needs to know when stop behavior is done so it can go back to publishing pure pursuit

# In order to test run the multiplexor in its current state:
1. Switch to stop behavior branch in our github repository 
2. In terminal, Enter → `roslaunch multiplexor testmultiplexor.launch`
3. In a new terminal window, publish a custom message to the /traffic_sign topic:
    - Enter → `rostopic pub /traffic_sign stop_behavior/TrafficSignStamped "h`      

    then press tab and you should get: 

    rostopic pub /traffic_sign stop_behavioTrafficSignStamped "header:
    seq: 0  
    stamp:  
        secs: 0  
        nsecs: 0  
    frame_id: ''  
    traffic_sign: ''   
    distance: "

- For now there is only a stop sign so enter ‘stop’ for the traffic_sign:  and a number for the distance ex: 15.0
- You can keep entering this command to test consecutive signs
- If you look in rvis you should see the car stopping when you publish to the /traffic_sign topic.  The terminal should also tell you what commands are being sent to the car
