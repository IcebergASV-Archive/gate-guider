<img src="https://user-images.githubusercontent.com/92492605/201941889-f4a18508-506d-4b2e-bd12-ac9e4553c2b9.png" width="200" height="200" />

# gate-guider

This repo will contain a ROS package and depancies to detect markers for the Panama Canal task and and output a GPS coordinate between the markers. The Panama Canal task requires that the ASV is autonomous at least 6 ft in front of the first set of gates. The ASV must then pass through the two sets of gates autonomously to successfully complete the task.

### Task Diagram

<img width="250" alt="name" src="https://user-images.githubusercontent.com/90921913/218113264-a1950a63-6b9e-47cd-97b8-9a09e15f3a64.png">

### Our Approach to Navgigating Through a Gate
<img width="168" alt="Gate guider (2)" src="https://user-images.githubusercontent.com/90921913/218120101-fbbb201d-221b-476f-aaef-461a2740ebe4.png">

### ASV View
<img width="450" alt="Gate guider (5)" src="https://user-images.githubusercontent.com/90921913/218119700-2aeac8f8-c393-4f90-a469-4437ca8de756.png">


## Prop Finder

![Gate guider (4)](https://user-images.githubusercontent.com/90921913/218108712-39d3538a-af41-429b-a17b-d92df176a49f.png) 

Preconditions
- ASV is more than 6 ft away from the markers, but close enough to classify and identify distance.
- Sensors are working and readings are valid

Post Conditions
- The GPS coordinate for the mid-point between the red and green marker has been identified.

### Object Detection

We are using YOLOv5 for object detection. We are creating a custom dataset with CVAT to train our model by following [this](https://www.youtube.com/watch?v=OMgQ2JzOAWA) YouTube tutorial. Training data was taken from the [RoboBoat website](https://roboboat.org) and past RoboBoat competition YouTube videos. 

We trained our model to recognize the following objects:
- black buoy
- red buoy 
- green buoy
- blue buoy
- yellow buoy
- white marker
- red marker
- green marker

## Design Considerations and Notes

This should be developed with other tasks in mind. We want to be able to resuse most of this to accomplish the speed run task. 

This method of navigation does not have a crash prevention system if another object or vehicle were to obstruct the robot's path. We should implement some sort of interput using an ultrasonic sensor that halts movement if something is detected close in front of the robot.  
