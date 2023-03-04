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

Prop Finder is inside the navigation package. It is launched with prop_finder.launch and contains 3 nodes: angle_finder, distance_finder, and coord_finder. It also includes nodes for publishing fake sensor data for testing. 

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



## Dataset Creation for You Only Look Once (YOLO)

1. Collect the images that you want to include in your dataset 
2. Go to the Computer Vision Annotation Tool (CVAT) website https://www.cvat.ai/ and select the try for free option in the top menu and create an account. Link: https://www.cvat.ai/ 
3. Once, you have entered create a new project. Use a descriptive name for the project and in the constructor section click add labels. Labels are the  names of the objects that you will be annotating. After you are finished select Submit and Open. **Note: If more labels want to be added afterwards it is possible.**
![Screenshot from 2023-02-11 13-56-22](https://user-images.githubusercontent.com/92492748/218272160-e0644138-3512-49bd-9d3e-ee2578bcd6e2.png)
4. Create a new task. Name the task and upload the images that you would like to annotate. **Note: Before you click submit and continue be aware that images can't be uploaded anymore. A new project a task must be created.** After click submit and continue.
![CreateTask](https://user-images.githubusercontent.com/92492748/218272508-407e444b-0430-4ed6-9503-e93212909beb.png)
5. In the left hand menu search for a square button and start annotating the objects you want to annotate. If you want to annotate a different object a drop down list will appear with the other object labels you created. **Click the save button after completing the annotations of one picture.**
6. To move to the next picture select go next.
7. After all the images are annotated. Go back to task and select actions. This will open a drop down list and select export task dataset. The format that you should select is Pascal VOC 1.1. Add a custom name to the zip file and click ok. The first time you run this it will take a couple minutes. The zip file should be available in your downloads.
![ExportTask](https://user-images.githubusercontent.com/92492748/218272866-1e55c90d-7caf-40a4-9cc7-58c7fc2c525b.png)
8. After extracting the zip file there are two folders: Annotations and ImageSets. In the Anotations folders there is an xml file that corresponds every image annotated. This files will be used to train the data. 
![ZipFileFolderBreakDown](https://user-images.githubusercontent.com/92492748/218273002-60398ab4-12a6-4c2f-a2c6-9764931c44c6.png)


## Computer Vision Annotation Tool (CVAT)
### CVAT Description
CVAT allows you to annotate the images for your data set



## Resources
The following video was used to create the Dataset: https://www.youtube.com/watch?v=OMgQ2JzOAWA
This video explains how to use CVAT to annotate images and how to export the annotations into your computer.

