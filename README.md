# Gimbal Tracking
Gimbal state machine for TAC mission, including visual tracking

![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/55540484/121034057-027dca80-c7ad-11eb-9780-2156c4ea7074.gif)

This is a ROS repolsitory for controlling a three axis gimbal and track a visual target. 

### Dependencies
* Three axis Alexmos32 gimbal (fw. 2.63b0) (Copterlab FLIR Boson was used)
* Detections as Detection2DArray ROS msg. 

### Current States
- Home
- Tracking
- Return_To_Home


#### Home
Default position of the gimbal

#### Tracking 
When a detection arrives the gimbal will track the target

#### Return_To_Home
Gimbal will go to home position


### Usage
1. Clone into your catkin workspace
2. Build and source
3. Run with rosrun
