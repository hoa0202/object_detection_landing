#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <chrono>
#include <thread>

#include <JetsonGPIO.h> // https://github.com/pjueon/JetsonGPIO

static int trigger = false;
static float gimbal_pitch = 0;

void Callback_gimbalControl(const std_msgs::Float32::ConstPtr& gimbal_msg);
void Callback_shutdown(const std_msgs::Bool::ConstPtr& shutdown_msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Gimbal_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_gimbalControl = nh.subscribe<std_msgs::Float32>("drone/gimbal_control", 10, &Callback_gimbalControl);
    ros::Subscriber sub_shutdown = nh.subscribe<std_msgs::Bool>("drone/shutdown", 10, &Callback_shutdown);
    

    ROS_INFO("Gimbal node start");

    const int GIMBAL_PIN = 33; // Jetson Xavier nx board
    float value = 0;

    GPIO::setwarnings(false);
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(GIMBAL_PIN, GPIO::OUT);

    GPIO::PWM pwm(GIMBAL_PIN, 50);
    pwm.start(0.0);

    while(ros::ok())
    {
        if (trigger)
        {
            pwm.ChangeDutyCycle(gimbal_pitch);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            trigger = false;

            ROS_INFO("Gimbal Move [%.2f]", gimbal_pitch);
        }

        ros::spinOnce();
        ros::Rate(10.0).sleep();
    }

    return 0; 
}

void Callback_shutdown(const std_msgs::Bool::ConstPtr& shutdown_msg)
{
    ROS_INFO("Gimbal node exit ...");
    ros::shutdown();
}

void Callback_gimbalControl(const std_msgs::Float32::ConstPtr& gimbal_msg)
{
    ROS_INFO("cb_gimbalControl");

    if (trigger){
        ROS_INFO("Gimbal is already Moving");
        return;
    }

    trigger = true;   
    gimbal_pitch = gimbal_msg->data;
    ROS_INFO("Trigger On [%.2f]", gimbal_pitch);
}
