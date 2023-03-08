#include <drone.h>
#include <cmath>

bool Drone::_connect()
{
    ROS_INFO("Connect...");

    int count = 0;

    now = ros::Time::now();
    prev_request.isZero();
    while (!current_state.connected)
    {
        now = ros::Time::now();
        if (now - prev_request > ros::Duration(5.0))
        {
            ROS_INFO("Waiting connect... %d", current_state.connected);
            count++;
            prev_request = now;
        }

        if (current_state.connected)
        {
            ROS_INFO("Connect success : %d", current_state.connected);
            return true;
        }

        if (count > 2)
        {
            ROS_ERROR("Connect Failed : %d", current_state.connected);
            return false;
        }
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connect success : %d", current_state.connected);
    return true;
}

void Drone::_arm(const char *mode)
{
    ROS_INFO("%s mode Arm", mode);

    if (current_state.armed)
    {
        ROS_ERROR("Already armed : %d", current_state.armed);
        return;
    }

    prev_request.isZero();
    prev_state = current_state;

    mavros_msgs::CommandBool cmd_arm;
    cmd_arm.request.value = true;

    mavros_msgs::SetMode cmd_mode;
    cmd_mode.request.base_mode = 0;
    cmd_mode.request.custom_mode = mode;

    while (ros::ok())
    {
        now = ros::Time::now();

        if (current_state.mode != mode && (now - prev_request > ros::Duration(2.0)))
        {
            ROS_INFO("Call mode change");
            set_mode_client.call(cmd_mode);
            prev_request = now;
        }

        else if (!current_state.armed && (now - prev_request > ros::Duration(2.0)))
        {
            ROS_INFO("Trying arm...");
            cmd_arm.request.value = true;
            arming_client.call(cmd_arm);
            prev_request = now;
        }

        //이전과 변경된 내용이 있을 경우 변경된 내용 출력
        if (prev_state.armed != current_state.armed)
        {
            ROS_INFO("Vehicle armed: %d", current_state.armed);
            prev_state.armed = current_state.armed;
            prev_request.isZero();
        }

        if (prev_state.mode != current_state.mode)
        {
            ROS_INFO("Current mode: %s", current_state.mode.c_str());
            prev_state.mode = current_state.mode;
            prev_request.isZero();
        }

        // arming 성공 -> 함수 종료
        if (current_state.armed)
        {
            ROS_INFO("Arm success [%s]", mode);
            return;
        }

        setpoint_local_pub.publish(setpoint_pose);

        ros::spinOnce();
        rate.sleep();
    }
}

void Drone::_change_mode(const char* mode){

    mavros_msgs::SetMode cmd_mode;
    cmd_mode.request.base_mode = 0;
    cmd_mode.request.custom_mode = mode;

    now = ros::Time::now();

    if (set_mode_client.call(cmd_mode) && cmd_mode.response.mode_sent)
    {
        ROS_INFO("Mode Changed %s", mode);
    }
    else
    {
        ROS_ERROR("Mode change Fail");
    }

    if (current_state.armed){
        isTakeoff = true;
    }
    ROS_INFO("Takeoff : %d", isTakeoff );
}

void Drone::_set_currentPose(){
    setpoint_pose.pose.position.x = current_pose.pose.position.x;
    setpoint_pose.pose.position.y = current_pose.pose.position.y;
    setpoint_pose.pose.position.z = current_pose.pose.position.z;
    setpoint_pose.pose.orientation.x = current_pose.pose.orientation.x;
    setpoint_pose.pose.orientation.y = current_pose.pose.orientation.y;
    setpoint_pose.pose.orientation.z = current_pose.pose.orientation.z;
    setpoint_pose.pose.orientation.w = current_pose.pose.orientation.w;

    double *current_RPY = quaternion_to_euler(setpoint_pose.pose.orientation.x, setpoint_pose.pose.orientation.y,
        setpoint_pose.pose.orientation.z, setpoint_pose.pose.orientation.w);

    if (current_RPY[2] < 0)
        current_RPY[2] = current_RPY[2] + M_PI * 2;
    else
        current_RPY[2] = current_RPY[2];

    ROS_INFO("Current yaw : %lf", radian_to_degree(current_RPY[2]));

    current_RPY[2] = (int)radian_to_degree(current_RPY[2]);
    current_RPY[2] = degree_to_radian(current_RPY[2]);
    radian_drone = current_RPY[2];

    double *current_Quaturnion = euler_to_quaternion(current_RPY[0], current_RPY[1], current_RPY[2]);
    
    setpoint_pose.pose.orientation.x = current_Quaturnion[0];
    setpoint_pose.pose.orientation.y = current_Quaturnion[1];
    setpoint_pose.pose.orientation.z = current_Quaturnion[2];    
    setpoint_pose.pose.orientation.w = current_Quaturnion[3];

    // _move_angular(degree_to_radian(temp) - current_RPY[2]);

    delete[] current_RPY;
    delete[] current_Quaturnion;

    if (current_state.armed == true && isTakeoff == false){
        isTakeoff = true;
    }
}

void Drone::_takeoff(float height)
{
    ROS_INFO("Takeoff...");

    if (!current_state.armed)
    {
        ROS_ERROR("Not Armed : %d", current_state.armed);
        return;
    }

    _set_currentPose();
    setpoint_pose.pose.position.z = 0;

    prev_request = ros::Time::now();
    while (setpoint_pose.pose.position.z < height)
    {
        if (ros::Time::now() - prev_request >= ros::Duration(height * 10.0))
        {
            ROS_ERROR("Drone can't takeoff");
            isTakeoff = false;
            return;
        }
        setpoint_pose.pose.position.z += 0.04;
        setpoint_local_pub.publish(setpoint_pose);
        ros::spinOnce();
        rate.sleep();
    }

    isTakeoff = true;
    setpoint_pose.pose.position.z = height;
    ROS_INFO("Takeoff Finish");
}

void Drone::_land()
{
    ROS_INFO("Land...");

    if (!current_state.armed)
    {
        ROS_ERROR("Not Flying : %d, Arm : %d", isTakeoff, current_state.armed);
        return;
    }

    if (current_state.mode != "OFFBOARD")
    {
        ROS_ERROR("Not OFFBOARD : %s", current_state.mode.c_str());
        return;
    }

    isTracking = false;
    nh.setParam("drone/isTracking", isTracking);
    ROS_INFO("Param changed : isTracking = %d", isTracking); 

    prev_request.isZero();
    std::string prev_mode = current_state.mode;
    int count = 0;

    while(ros::ok()){
        now.now();

        if (count > 3){
            ROS_ERROR("Error Land");
            return;
        }

        if (current_state.mode != "AUTO.LAND" && now - prev_request > ros::Duration(1.0)){
            _change_mode("AUTO.LAND");
            count++;
            prev_request = now;
        }

        if (prev_mode != current_state.mode){
            ROS_INFO("%s start", current_state.mode.c_str());
            prev_mode = current_state.mode;
        }

        if (current_state.armed == false){
            ROS_INFO("AUTO.LAND Finish");
            isTakeoff = false;
            _change_mode("MANUAL");

            setpoint_pose.pose.position.z = 0;   

            break;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void Drone::_hover()
{
    now = ros::Time::now();
    if (now - prev_request_hoveringMsg > ros::Duration(0.5)){
        ROS_INFO("hover... [%lf][%.2f, %.2f, %.2f][%.2f, %.2f, %.2f]", radian_to_degree(radian_drone),
                setpoint_pose.pose.position.x, setpoint_pose.pose.position.y, setpoint_pose.pose.position.z,
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);    
    
        prev_request_hoveringMsg = now;
    }
    setpoint_local_pub.publish(setpoint_pose);
}

void Drone::_move_linear(double pose_x, double pose_y, double pose_z)
{
    ROS_INFO("Goto... [%.2f, %.2f, %.2f]", pose_x, pose_y, pose_z);

    if (!isTakeoff)
    {
        ROS_ERROR("Not Flying : %d", isTakeoff);
        return;
    }

    double wp[3] = {0, 0, pose_z};
    
    if (pose_x > 0){
        wp[0] = pose_x * cos(radian_drone);
        wp[1] = pose_x * sin(radian_drone);
    } else if(pose_x < 0){
        wp[0] = abs(pose_x) * -cos(radian_drone);
        wp[1] = abs(pose_x) * -sin(radian_drone);
    }

    if (pose_y > 0){
        wp[0] = pose_y * sin(radian_drone);            //-cos(radian_drone + M_PI_2);
        wp[1] = pose_y * -cos(radian_drone);           //-sin(radian_drone + M_PI_2);
    } else if(pose_y < 0){
        wp[0] = abs(pose_y) * -sin(radian_drone);      //cos(radian_drone + M_PI_2);
        wp[1] = abs(pose_y) * cos(radian_drone);       //sin(radian_drone + M_PI_2);
    }

    double check_height = setpoint_pose.pose.position.z + wp[2];

    if (check_height < 1)
    {
        ROS_INFO("AutoLand start [z : %.2f]", setpoint_pose.pose.position.z);
        _land();
        return;
    }

    setpoint_pose.pose.position.x += wp[0];
    setpoint_pose.pose.position.y += wp[1];
    setpoint_pose.pose.position.z += wp[2];

    isMoving = true;
    while(ros::ok()){
        if (_check_moveComplete("linear")){
            break;
        }
        _hover();

        ros::spinOnce();
        rate.sleep();
    }
    isMoving = false;
    ROS_INFO("Move Linear Finish : %.2f, %.2f, %.2f", setpoint_pose.pose.position.x, setpoint_pose.pose.position.y, setpoint_pose.pose.position.z);
}

void Drone::_move_linear_fix_z(double pose_x, double pose_y, double pose_z)
{
    ROS_INFO("Goto... [%.2f, %.2f, %.2f]", pose_x, pose_y, pose_z);

    if (!isTakeoff)
    {
        ROS_ERROR("Not Flying : %d", isTakeoff);
        return;
    }

    double wp[3] = {0, 0, pose_z};
    
    if (pose_x > 0){
        wp[0] = pose_x * cos(radian_drone);
        wp[1] = pose_x * sin(radian_drone);
    } else if(pose_x < 0){
        wp[0] = abs(pose_x) * -cos(radian_drone);
        wp[1] = abs(pose_x) * -sin(radian_drone);
    }

    if (pose_y > 0){
        wp[0] = pose_y * sin(radian_drone);            //-cos(radian_drone + M_PI_2);
        wp[1] = pose_y * -cos(radian_drone);           //-sin(radian_drone + M_PI_2);
    } else if(pose_y < 0){
        wp[0] = abs(pose_y) * -sin(radian_drone);      //cos(radian_drone + M_PI_2);
        wp[1] = abs(pose_y) * cos(radian_drone);       //sin(radian_drone + M_PI_2);
    }

    double check_height = setpoint_pose.pose.position.z + wp[2];

    if (check_height < 1)
    {
        ROS_INFO("AutoLand start [z : %.2f]", setpoint_pose.pose.position.z);
        _land();
        return;
    }

    setpoint_pose.pose.position.x += wp[0];
    setpoint_pose.pose.position.y += wp[1];
    setpoint_pose.pose.position.z = current_pose.pose.position.z;

    isMoving = true;
    while(ros::ok()){
        if (_check_moveComplete("linear")){
            break;
        }
        _hover();

        ros::spinOnce();
        rate.sleep();
    }
    isMoving = false;
    ROS_INFO("Move Linear Finish : %.2f, %.2f, %.2f", setpoint_pose.pose.position.x, setpoint_pose.pose.position.y, setpoint_pose.pose.position.z);
}

void Drone::_move_angular(double radian)
{
    double goal_radian = radian_drone + radian;

    if (goal_radian >= M_PI*2)
        goal_radian -= M_PI*2;
    else if(goal_radian <=0)
        goal_radian += M_PI*2;

    radian_drone = goal_radian;

    double *q;
    q = euler_to_quaternion(0, 0, radian_drone);

    setpoint_pose.pose.orientation.x = q[0];
    setpoint_pose.pose.orientation.y = q[1];
    setpoint_pose.pose.orientation.z = q[2];
    setpoint_pose.pose.orientation.w = q[3];

    isMoving = true;
    while (ros::ok())
    {
        if (_check_moveComplete("angular")){
            break;
        }
        _hover();

        ros::spinOnce();
        rate.sleep();
    }
    isMoving = false;
    ROS_INFO("Move Angular Finish : [degree: %.3lf]", radian_to_degree(radian_drone));
    
    delete[] q;
}

bool Drone::_check_moveComplete(const char* select){
    //setpoint 와 current_pose 를 비교해서 일정 수치 이내로 들어오면 움직임 완료했다고 판단
    //각도 바뀌는 것도 넣어야하나? -> 큰 값이면 넣어야하는데 1도 움직이는 거는 필요 없을듯
    double margin_error_linear = 0.3;
    double margin_error_radian = 0.009;//0.009; // 약 0.5도 오차

    if (select == "linear")
    {
        double goal[3] = {setpoint_pose.pose.position.x, setpoint_pose.pose.position.y, setpoint_pose.pose.position.z};
        double current[3] = {current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z};
        if (_norm(goal, current, 3) < margin_error_linear)
            return true;
        else
            return false;
    }
    else if (select == "angular")
    {
        double *current_RPY = quaternion_to_euler(current_pose.pose.orientation.x, current_pose.pose.orientation.y,
            current_pose.pose.orientation.z, current_pose.pose.orientation.w);

        if (current_RPY[2] < 0){
            current_RPY[2] += M_PI * 2;
        } else if (current_RPY[2] > M_PI * 2){
            current_RPY[2] -= M_PI * 2;
        }
        
        // ROS_INFO("%lf , %lf , norm : %lf", radian_drone, current_RPY[2], _norm(&radian_drone, &current_RPY[2], 1));

        if (_norm(&radian_drone, &current_RPY[2], 1) < margin_error_radian)
            return true;
        else
            return false;

        
    }

}

void Drone::PublishMountControl(float gimbal_pitch, float gimbal_yaw){//태풍 짐벌 제어테스트
    mavros_msgs::MountControl msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.mode = 2;
    msg.pitch = gimbal_pitch * M_PI;
    msg.roll = gimbal_yaw * M_PI;
    msg.yaw = M_PI;

    mount_control_pub_.publish(msg);
    
} 

void Drone::PublishActuatorSetpoints(float gimbal_pitch_Actuator, float gimbal_yaw_Actuator){ //실제 짐벌 제어
    mavros_msgs::ActuatorControl msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.group_mix = 3;
    msg.controls[5] = gimbal_pitch_Actuator / M_PI;
    msg.controls[6] = gimbal_yaw_Actuator / M_PI;

    actuator_setpoint_pub_.publish(msg);
    ROS_INFO("%d",msg.controls[5]);
}

 void Drone::ControlGimbal_Jetson(float pitch, bool cal_type)
{
    if (cal_type)
        gimbal_pitch_Jetson = pitch + 0.1;
    else
        gimbal_pitch_Jetson = pitch - 0.1; 

    ROS_INFO("Gimbal Control [p: %.2f]", pitch);

    

    std_msgs::Float32 gimbal_msg;
    gimbal_msg.data = pitch;

    pub_pwmGimbal.publish(gimbal_msg);
}

void Drone::SetDefaultHeight()
{
    setpoint_pose.pose.position.z = 3.0;
}

// Set local position as current position
void Drone::SetCurrentPose()
{
    setpoint_pose.pose.position.x = current_pose.pose.position.x;
    setpoint_pose.pose.position.y = current_pose.pose.position.y;
    setpoint_pose.pose.position.z = current_pose.pose.position.z;
}

// Set Local yaw as current yaw
void Drone::SetCurrentYaw()
{
    setpoint_pose.pose.orientation.w = current_pose.pose.orientation.w;
    setpoint_pose.pose.orientation.x = current_pose.pose.orientation.x;
    setpoint_pose.pose.orientation.y = current_pose.pose.orientation.y;
    setpoint_pose.pose.orientation.z = current_pose.pose.orientation.z;

    double *current_RPY = quaternion_to_euler(setpoint_pose.pose.orientation.x, setpoint_pose.pose.orientation.y,
                                              setpoint_pose.pose.orientation.z, setpoint_pose.pose.orientation.w);

    if (current_RPY[2] < 0)
        current_RPY[2] = current_RPY[2] + M_PI * 2;
    else if (current_RPY[2] > M_PI * 2)
        current_RPY[2] = current_RPY[2] - M_PI * 2;

    radian_drone = current_RPY[2];

    delete[] current_RPY;
}