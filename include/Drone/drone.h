#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandHome.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <std_msgs/Header.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>


#include <Convert.h>

#include <mavros_msgs/MountControl.h> //태풍 짐벌 제어 테스트
#include <mavros_msgs/ActuatorControl.h>

#include <nav_msgs/Odometry.h> // 경로


#include <darknet_ros_msgs/BoundingBoxes.h>

#include <aruco_msgs/Marker.h>


class Drone{
    private:
        ros::NodeHandle nh;

        ros::Subscriber state_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber home_sub;
        ros::Subscriber objectDetection_sub;

        ros::Publisher setpoint_local_pub;

        ros::Publisher mount_control_pub_; //태풍 짐벌 제어테스트
        ros::Publisher actuator_setpoint_pub_;

        ros::Publisher pub_pwmGimbal;

        ros::Publisher odom_pub;

        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient set_home_client;

        geometry_msgs::PoseStamped setpoint_pose;
        geometry_msgs::PoseStamped current_pose;

        double radian_drone;    //drone 의 yaw 값 (radian)
        double theta;           //카메라의 각도 (radian)

        mavros_msgs::State current_state;
        mavros_msgs::State prev_state;

        sensor_msgs::NavSatFix current_gps;
        mavros_msgs::CommandHome cmd_setHome;
        mavros_msgs::HomePosition current_home;

        
        ros::Time prev_request_hoveringMsg;
        ros::Time prev_ObjectDetection;
        ros::Time prev_request;
        ros::Time now;

        ros::Time land_detect_check_request;

        ros::Rate rate = ros::Rate(10.0);
        
        bool isTakeoff = false;
        bool isTracking = false;
        bool isMoving = false;
  
        bool success;

        bool _connect();
        void _arm(const char* mode);
        void _change_mode(const char* mode);
        void _takeoff(float height);
        void _land();
        void _hover();

        void SetCurrentPose();
        void SetCurrentYaw();
        void SetDefaultHeight();

        void ControlGimbal_Jetson(float Gimbal_pitch, bool cal_type);
        
        void _set_currentPose();
        void _move_linear(double pose_x, double pose_y, double pose_z);
        void _move_linear_fix_z(double pose_x, double pose_y, double pose_z); // 지정한 고도로 고정
        void _move_angular(double radian);
        bool _check_moveComplete(const char* select);

        void _cb_state(const mavros_msgs::State::ConstPtr& state_msg);
        void _cb_pose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
        void _cb_gps(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
        void _cb_home(const mavros_msgs::HomePosition::ConstPtr& home_msg);


        void PublishMountControl(float gimbal_pitch, float gimbal_yaw); //태풍 짐벌 제어테스트
        void PublishActuatorSetpoints(float gimbal_pitch_Actuator, float gimbal_yaw_Actuator);


        ros::Subscriber sub_Aruco_pose;

        ros::Subscriber sub_BoundingBox;

        ros::Subscriber sub_SetpointLocal;
        ros::Subscriber sub_LocalPosition;

        ros::Publisher pub_MoveDrone;

        geometry_msgs::PoseStamped current_setpoint;

        ros::Time box_request;

        int _frameMiddleX; // 카메라 X 중심
        int _frameMiddleY; // 카메라 Y 중심

        double x_probability_max; // 고도에 따른 확률 최대
        double x_probability_min; // 고도에 따른 확률 최소

        double y_probability_max; // 고도에 따른 확률 최대
        double y_probability_min; // 고도에 따른 확률 최소

        int z_probability_angular; // 고도에 따른 yaw 각도
        double z_percentage; // object 를 x축 중앙에 위치 시킨 뒤에(오차 z_percentage %) object 쪽으로 이동하여 착륙


        double speed_1 = 0.1; // 기본 0.1
        double speed_2 = 0.2; // 기본 0.1
        double box_probability = 0.3;


        int gimbal_check = 0;

        float gimbal_pitch_Gazebo;

        float gimbal_pitch_Jetson;

        double cnt_speed;

        void cb_Aruco_pose(const geometry_msgs::Vector3Stamped::ConstPtr& aruco_m);

        void cb_Box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg);

        void cb_LocalPosition(const geometry_msgs::PoseStamped::ConstPtr& localPosition_msg);
        void cb_SetpointLocal(const geometry_msgs::PoseStamped::ConstPtr& setpointLocal_msg);
        

    public:
        explicit Drone(const ros::NodeHandle& _nodeHandle);
        void _control();
};
