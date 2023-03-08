#include <drone.h>

Drone::Drone(const ros::NodeHandle& _nodeHandle):
    nh(_nodeHandle),
    state_sub(nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Drone::_cb_state, this)),
    pose_sub(nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Drone::_cb_pose, this)),
    gps_sub(nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, &Drone::_cb_gps, this)),
    home_sub(nh.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 10, &Drone::_cb_home, this)),

    setpoint_local_pub(nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10)),

    arming_client(nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming")),
    set_mode_client(nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode")),

    set_home_client(nh.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home")),

    mount_control_pub_(nh.advertise<mavros_msgs::MountControl>("mavros/mount_control/command", 10)), //태풍 짐벌 제어테스트
    actuator_setpoint_pub_(nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 10)),

    pub_pwmGimbal(nh.advertise<std_msgs::Float32>("drone/gimbal_control", 10)), // In Jetson, gimbal control


    odom_pub(nh.advertise<nav_msgs::Odometry>("odom", 50)),


    sub_Aruco_pose(nh.subscribe<geometry_msgs::Vector3Stamped>("/aruco_single/position", 10, &Drone::cb_Aruco_pose, this)),




    sub_BoundingBox(nh.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 10, &Drone::cb_Box, this)),
    sub_LocalPosition(nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Drone::cb_LocalPosition, this)),
    sub_SetpointLocal(nh.subscribe<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10, &Drone::cb_SetpointLocal, this)),


    _frameMiddleX(640/2), //gazebo iris camera : 320 x 240
    _frameMiddleY(480/2),

    
    rate(ros::Rate(10.0))   //rate default = 10hz
{
    setpoint_pose.pose.position.x = 0;
    setpoint_pose.pose.position.y = 0;
    setpoint_pose.pose.position.z = 0;


    theta = degree_to_radian(60);
    radian_drone = 0;

    prev_ObjectDetection.isZero();

    gimbal_pitch_Jetson = 7;

    isTakeoff = false;
    isTracking = false;

    cnt_speed = 1.0;

    
    nh.setParam("/drone/isTracking", isTracking);

}