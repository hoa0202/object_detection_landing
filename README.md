# object detection auto landing with yolo v4
object detection landing


# Environment
Host Ubuntu 18.04 \
jetson Ubuntu 18.04 \
darknet ros yolo v4 tiny (https://github.com/Tossy0423/darknet_ros.git)

# Installation
mkdir -p catkin_ws/src && catkin_ws/src \
git clone https://github.com/hoa0202/object_detection_landing.git \

source ~/catkin_ws/devel/setup.bash \

roslaunch object_detection_landing drone_ctrl.launch \

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
