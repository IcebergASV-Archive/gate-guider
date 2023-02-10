<img src="https://user-images.githubusercontent.com/92492605/201941889-f4a18508-506d-4b2e-bd12-ac9e4553c2b9.png" width="200" height="200" />

# gate-guider

This repo will contain a ROS package and depancies to detect markers for the Panama Canal task and and output a GPS coordinate between the markers. 

<img width="250" alt="Gate guider (2)" src="https://user-images.githubusercontent.com/90921913/218099834-93d54166-97d3-4dd4-9d11-b6267585c7c8.png">


## Prop Finder

![Gate guider (4)](https://user-images.githubusercontent.com/90921913/218108712-39d3538a-af41-429b-a17b-d92df176a49f.png)

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
