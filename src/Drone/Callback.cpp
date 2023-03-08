#include <drone.h>
void Drone::_cb_state(const mavros_msgs::State::ConstPtr& state_msg)
{
    current_state = *state_msg;

    if (current_state.mode != state_msg->mode){
        success = true;

        if (state_msg->mode == "OFFBOARD")
        {   
            // Flying & Arming+offboard => set current Position & set default height for takeoff
            if (current_state.armed){
                SetCurrentPose();
                SetCurrentYaw();
                
                // // Check height and Set height
                // if (current_pose.pose.position.z <= 0.4)
                //     SetDefaultHeight();
            }
            else{
                // 
                SetCurrentPose();
                SetCurrentYaw();
                //SetDefaultHeight();
            }
        }
    }
}

void Drone::_cb_pose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    current_pose = *pose_msg;
}

void Drone::_cb_gps(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    current_gps = *gps_msg;
}

void Drone::_cb_home(const mavros_msgs::HomePosition::ConstPtr& home_msg)
{
    current_home = *home_msg; 
}

void Drone::cb_LocalPosition(const geometry_msgs::PoseStamped::ConstPtr &localPosition_msg)
{
    current_pose = *localPosition_msg;
}

void Drone::cb_SetpointLocal(const geometry_msgs::PoseStamped::ConstPtr &setpointLocal_msg)
{
    current_setpoint = *setpointLocal_msg;
}



/*
void Drone::_cb_objectDetection(const offboard::Move::ConstPtr& move_msg){
    
    if (isMoving)
        return;

    double move_value[3] = {0,};
    now.now();
    
    if (now - prev_ObjectDetection > ros::Duration(0.1)){
        prev_ObjectDetection = now;
        ROS_INFO("Get Data from ObjectDetection node [Angular|Linear|gimbal : %d, %d, %d]", move_msg->linear, move_msg->angular, move_msg->gimbal_roll);
        if (current_state.mode != "OFFBOARD"){
            _change_mode("OFFBOARD");
        }
    }else{
        return;
    }
    */

/*
    PublishActuatorSetpoints(-(move_msg->gimbal_roll)/10, 0.0);
    PublishMountControl(-12-(move_msg->gimbal_roll), 0.0);
    */
    //theta = degree_to_radian(89.9-(60-(3.75*(move_msg->gimbal_roll))));//16단계로 나누어지기때문에 60/16=3.75

    //10 미터 이상이면 5도 씩 움직이고
    //5 미터 이상이면 2도 씩 움직이고
    //5 미터 이하이면 1도 씩 움직이기

    /*
    _move_angular(degree_to_radian(move_msg->angular));
*/
    
/*
    if (move_msg->angular == 1){
        _move_angular(degree_to_radian(1));
    }
    else if(move_msg->angular == -1){
        _move_angular(degree_to_radian(-1));
    }
*/

/*
    if (move_msg->linear == -1){
        _land();
    } 
    else if(move_msg->linear == 1){
        move_value[0] = setpoint_pose.pose.position.z/tan(theta)/10;
        move_value[2] = -(setpoint_pose.pose.position.z/10);

        _move_linear(move_value[0], move_value[1], move_value[2]);
    }
    else if(move_msg->linear == 2){
        move_value[0] = setpoint_pose.pose.position.z/tan(theta)/7;

        _move_linear(move_value[0], move_value[1], move_value[2]);
    }
    else if(move_msg->linear == 3){
        move_value[2] = -setpoint_pose.pose.position.z/10;

        _move_linear(move_value[0], move_value[1], move_value[2]);
    }
    else if(move_msg->linear == 4){
        move_value[1] = 0.1;
        _move_linear(move_value[0], move_value[1], move_value[2]);
    }
    else if(move_msg->linear == 5){
        move_value[1] = -0.1;
        _move_linear(move_value[0], move_value[1], move_value[2]);
    }
    else if(move_msg->linear == 6){
        move_value[0] = 0.1;
        _move_linear(move_value[0], move_value[1], move_value[2]);
    }
    else if(move_msg->linear == 7){
        move_value[0] = -0.1;
        _move_linear(move_value[0], move_value[1], move_value[2]);
    }
    else if(move_msg->linear == 8){
        move_value[0] = current_pose.pose.position.z*sqrt(3);
        _move_linear(move_value[0], move_value[1], move_value[2]);

        PublishMountControl(-28, 0.0);
        PublishActuatorSetpoints(-1.6, 0.0);
        ROS_INFO("============go andpad============");
    }
    else if(move_msg->linear == 9){
        move_value[1] = 0.1;
        move_value[2] =2.9;
        _move_linear_fix_z(move_value[0], move_value[1], move_value[2]);
    }
    else if(move_msg->linear == 10){
        move_value[1] = -0.1;
        move_value[2] = 2.9;
        _move_linear_fix_z(move_value[0], move_value[1], move_value[2]);
    }
    else if(move_msg->linear == 11){
        move_value[0] = 0.1;
        move_value[2] = 2.9;
        _move_linear_fix_z(move_value[0], move_value[1], move_value[2]);
    }
    else if(move_msg->linear == 12){
        move_value[0] = -0.1;
        move_value[2] = 2.9;
        _move_linear_fix_z(move_value[0], move_value[1], move_value[2]);
    }
}
*/