# Object Detection Auto Landing with Yolo v4
This project involves recognizing a landing pad from a long distance and autonomously landing on it.
![image](https://github.com/hoa0202/object_detection_landing/assets/67028730/b17315fe-f0da-441a-8f4a-321cb3f39131)

- The initial camera angle starts at 60 degrees.
- Upon reaching a specific altitude, the drone moves forward by a calculated distance and then adjusts the camera angle to 90 degrees.
- After making fine adjustments, it lands.

![image](https://github.com/hoa0202/object_detection_landing/assets/67028730/21e9604e-d43e-43f4-9fc0-50d735aec000)

- The drone's position is adjusted using YOLO v4 to center the landing pad in the camera's view.

# Environment
Host Ubuntu 18.04 \
jetson Ubuntu 18.04 \
darknet ros yolo v4 tiny (https://github.com/Tossy0423/darknet_ros.git)

# Installation
```
mkdir -p catkin_ws/src && catkin_ws/src \
git clone https://github.com/hoa0202/object_detection_landing.git \

source ~/catkin_ws/devel/setup.bash \

roslaunch object_detection_landing drone_ctrl.launch \
```
# Command
q : exit \
t : takeoff \
g : land \
u : move linear up \
j : move linear down \
w : move linear foward \
a : move linear laft side \
s : move_linear back side \
d : move_linear right side \
1 : move angular unclock arrow \
3 : move angular clock arrow \
c : mode change (offboard mode) \
m : mode change (mission mode) \
0 : tracking mode (on/off) \
8 : gimbal pitch up side \
9 : gimbal pitch down side

# Paper
- 강호현, and 신수용. "짐벌 카메라 각도 제어와 객체 인식을 사용한 UAV 자동착륙 시스템." 한국통신학회논문지 48.2 (2023): 241-248.
