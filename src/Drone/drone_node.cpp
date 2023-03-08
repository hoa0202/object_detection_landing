#include <drone.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "keyboard_ctrl");
    ros::NodeHandle nodehandle;

    ROS_INFO("Node start");
    Drone drone(nodehandle);

    drone._control();

    ros::spin();

    return 0;
}