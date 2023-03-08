#include <drone.h>
// void Drone::cb_Aruco_pose(const geometry_msgs::Vector3Stamped::ConstPtr &aruco_m)
// {
//     geometry_msgs::Vector3 Aruco_pos = aruco_m->vector;


//     x_probability_max = 1.2;
//     x_probability_min = 0.8;
//     y_probability_max = 1.2;
//     y_probability_min = 0.8;
//     z_probability_angular = 1;
//     z_percentage = 0.1;


//     double poseX;

//     poseX = Aruco_pos.x;

//     long int xDifferenceAbsolute = labs(labs(Aruco_pos.x) - 0.0); //바운딩 박스 중심으로부터 카메라 X 오차의 절대값 //labs() : return absolute value
//     long int yDifferenceAbsolute = labs(labs(Aruco_pos.y) - 0.25); //바운딩 박스 중심으로부터 카메라 Y 오차의 절대값


//     return;
// }
void Drone::cb_Box(const darknet_ros_msgs::BoundingBoxes::ConstPtr &box_msg)
{
    if (!nh.getParam("/drone/isTracking", isTracking))
    {
        ROS_ERROR("No paramter named /drone/isTracking");
        return;
    }
    


    darknet_ros_msgs::BoundingBox Box = box_msg->bounding_boxes[0];

    int move_check = 0;    // 1: 이동 , 0: 정지 , -1 : 착륙
    int angle_check = 0;   // 1: + (반시계방향) , -1 : - (시계방향) , 0 : 정지
    
    //Move_msg.gimbal_roll = 0;
    

    // 고도에 따른 확률 설정
    if (current_pose.pose.position.z > 5){
        x_probability_max = 1.3;
        x_probability_min = 0.7;

        y_probability_max = 1.3;
        y_probability_min = 0.7;
        
        z_probability_angular = 5;
        z_percentage = 0.3;
        //Move_msg.gimbal_roll = 0;
    }//짐벌 각도 계산하는거 만들순서
    else{
        x_probability_max = 1.2;
        x_probability_min = 0.8;
        y_probability_max = 1.2;
        y_probability_min = 0.8;
        z_probability_angular = 1;
        z_percentage = 0.1;
    }

    //box_request = ros::Time::now();
    now = ros::Time::now();
    //box_request.isZero();
    //land_detect_check_request.isZero();


    if (now - box_request > ros::Duration(cnt_speed))
    {
        box_request = now;

        if (Box.probability >= box_probability)
        {
            //land_detect_check_request = now;
            if (isTracking == true)
            {


                long int xMiddle = (Box.xmin + Box.xmax) / 2; //바운딩박스 X 중심 //Calculate Middle of length
                long int yMiddle = (Box.ymin + Box.ymax) / 2; //바운딩박스 Y 중심
                long int xDifference = xMiddle - _frameMiddleX; //바운딩 박스 중심으로부터 카메라 X 오차 //Calculate how far away the box is from the centre.
                long int yDifference = yMiddle - _frameMiddleY; //바운딩 박스 중심으로부터 카메라 Y 오차
                long int xDifferenceAbsolute = labs(xMiddle - _frameMiddleX); //바운딩 박스 중심으로부터 카메라 X 오차의 절대값 //labs() : return absolute value
                long int yDifferenceAbsolute = labs(yMiddle - _frameMiddleY); //바운딩 박스 중심으로부터 카메라 Y 오차의 절대값

                long int bboxHeight = Box.ymax - Box.ymin; // 바운딩 박스의 높이길이
                long int bboxWidth = Box.xmax - Box.xmin; // 바운딩 박스의 가로길이
                ROS_INFO("z : %.2f",current_pose.pose.position.z);

                //box_request = now;

                double move_value[3] = {0,};

                if (current_pose.pose.position.z < 1.5){ // 고도가 2.0 미만일때 착륙
                        move_check = -1;
                    }
                

                // object 를 x축 중앙에 위치 시킨 뒤에(오차10%) object 쪽으로 이동하여 착륙
                if (xDifferenceAbsolute < (int)(_frameMiddleX * z_percentage))
                {
                    if (current_pose.pose.position.z < 1.5){ // 고도가 2.0 미만일때 착륙
                        move_check = -1;
                    }
                    else
                    {
                        ROS_INFO("gimbal_check %d", gimbal_check);
                        
                        //if (current_pose.pose.position.z > 3 && gimbal_check == 0)// 고도가 3.0 이상일때
                        if (gimbal_check == 0)
                        {
                            cnt_speed = 0.8;
                            //gimbal_check = 0;
                            if (current_pose.pose.position.z > 3)
                            {
                                if (yMiddle < (int)(_frameMiddleY * y_probability_min))
                                {
                                    move_check = 2;//전진
                                    ROS_INFO("Go straight %d [%ld < %d]", move_check, yMiddle, (int)(_frameMiddleY * y_probability_min));
                                }
                                else if (yMiddle > (int)(_frameMiddleY * y_probability_max))
                                {
                                    move_check = 3;//하강
                                    ROS_INFO("Go Down %d [%ld > %d]", move_check, yMiddle, (int)(_frameMiddleY * y_probability_max));
                                }
                                else
                                {
                                    move_check = 1;//대각선 방향으로 전진 밑 하강
                                    ROS_INFO("Move %d", move_check);
                                }
                            }
                            else
                            {
                                if (yMiddle < (int)(_frameMiddleY * y_probability_min))// 중앙정렬
                                {
                                    move_check = 11;//고도유지 전진
                                    ROS_INFO("Go straight2 %d [%ld < %d]", move_check, yMiddle, (int)(_frameMiddleY * y_probability_min));
                                }
                                else if (yMiddle > (int)(_frameMiddleY * y_probability_max))
                                {
                                    move_check = 12;//고도유지 후진
                                    ROS_INFO("Go Back2 %d [%ld > %d]", move_check, yMiddle, (int)(_frameMiddleY * y_probability_max));
                                }
                                else // 중앙정렬이 됬으면 랜딩패드 정 위로 이동 카메라 각도 90도 하단 변경
                                {
                                    move_check = 8;//랜딩패드 위로 이동
                                    gimbal_check=1;
				                    speed_2=0.08;
                                    box_probability =0.4;
                                    //Move_msg.gimbal_roll = 16;
                                    
                                    ROS_INFO("Go landingpad %d [%d < %ld < %d]", move_check, (int)(_frameMiddleY * y_probability_min), yMiddle, (int)(_frameMiddleY * y_probability_max));
                                }
                                
                            }
                        }
                        else// if (current_pose.pose.position.z < 3)// gimbal check == 1;
                        {
                            cnt_speed=1.0;//0.5
                            if (yMiddle < (int)(_frameMiddleY * y_probability_min))// 중앙정렬
                            {
                                move_check = 6;//전진
                                ROS_INFO("Go straight3 %d [%ld < %d]", move_check, yMiddle, (int)(_frameMiddleY * y_probability_min));
                            }
                            else if (yMiddle > (int)(_frameMiddleY * y_probability_max))
                            {
                                move_check = 7;//후진
                                ROS_INFO("Go Back3 %d [%ld > %d]", move_check, yMiddle, (int)(_frameMiddleY * y_probability_max));
                            }
                            else // 
                            {
                                move_check = 3;//하강
                                ROS_INFO("Go Down3 %d [%d < %ld < %d]", move_check, (int)(_frameMiddleY * y_probability_min), yMiddle, (int)(_frameMiddleY * y_probability_max));
                            }
                        }
                    }
                }
                else    //object 가 중앙에 오도록 yaw 값 수정
                {
                    if (current_pose.pose.position.z < 1.5){ // 고도가 1.5 미만일때 착륙
                        move_check = -1;
                    }    
                    
                    if (xMiddle > (int)(_frameMiddleX * x_probability_max))
                    { //turn left (시계방향)
                        if(gimbal_check == 0)//(current_pose.pose.position.z > 4){ // 고도가 6보다 크면 yaw 6보다 작으면 xy 제어 //짐벌 바뀌기 전이면 yaw 제어
                        {

                            angle_check = -z_probability_angular;
                            
                            ROS_INFO("Clock wise %d, [%ld > %d]", angle_check, xMiddle , (int)(_frameMiddleX * x_probability_max));
                        }
                        else// if(current_pose.pose.position.z <= 4){// gimbal check == 1;
                        {    move_check = 4; // 우측으로 이동
                            ROS_INFO("Turn Right %d, [%ld > %d]", move_check, xMiddle , (int)(_frameMiddleX * x_probability_max));
                        }
                    }
                    else if (xMiddle < (int)(_frameMiddleX * x_probability_min))
                    { // turn right (반시계방향)
                        if(gimbal_check == 0)//(current_pose.pose.position.z > 4){ //짐벌 바뀌기 전이면 yaw 제어
                        {

                            angle_check = z_probability_angular;
                            printf("1");
                            ROS_INFO("UnClock wise %d, [%ld < %d]", angle_check, xMiddle , (int)(_frameMiddleX * x_probability_min));
                        }
                        else// if(current_pose.pose.position.z < 3){// gimbal check == 1;
                        {    move_check = 5; // 좌측으로 이동
                            printf("2");
                            ROS_INFO("Turn Left %d, [%ld > %d]", move_check, xMiddle , (int)(_frameMiddleX * x_probability_max));
                        }
                    }
                }

                _move_angular(degree_to_radian(angle_check));


                if (move_check == -1){ //착륙
                    _land();
                } 
                else if(move_check == 1){//대각선 방향으로 전진 밑 하강
                    move_value[0] = current_pose.pose.position.z*tan(theta)/8;//setpoint_pose.pose.position.z*tan(theta)/7
                    move_value[2] = -(current_pose.pose.position.z/7);//setpoint_pose.pose.position.z*tan(theta)/7

                    _move_linear(move_value[0], move_value[1], move_value[2]);
                }
                else if(move_check == 2){ //전진
                    move_value[0] = current_pose.pose.position.z*tan(theta)/8;//setpoint_pose.pose.position.z*tan(theta)/7

                    _move_linear(move_value[0], move_value[1], move_value[2]);
                }
                else if(move_check == 3){// 하강
                    move_value[2] = -setpoint_pose.pose.position.z/7;

                    _move_linear(move_value[0], move_value[1], move_value[2]);
                }
                else if(move_check == 4){// 우측으로 이동
                    move_value[1] = speed_2;
                    _move_linear(move_value[0], move_value[1], move_value[2]);
                }
                else if(move_check == 5){// 좌측으로 이동
                    move_value[1] = -speed_2;
                    _move_linear(move_value[0], move_value[1], move_value[2]);
                }
                else if(move_check == 6){// 전진
                    move_value[0] = speed_2;
                    _move_linear(move_value[0], move_value[1], move_value[2]);
                }
                else if(move_check == 7){// 후진
                    move_value[0] = -speed_2;
                    _move_linear(move_value[0], move_value[1], move_value[2]);
                }
                else if(move_check == 8){// 랜딩패드 위로 이동
                    move_value[0] = (current_pose.pose.position.z*tan(theta)); //current_pose.pose.position.z*sqrt(3))-0.5
                    _move_linear(move_value[0], move_value[1], move_value[2]);

                    //PublishMountControl(-28, 0.0);
                    //PublishActuatorSetpoints(-1.6, 0.0);
                    ControlGimbal_Jetson(5.6, false);
                    ROS_INFO("============go aboat pad============");
                }
                else if(move_check == 9){//고도 유지하면서 우측으로 이동
                    move_value[1] = speed_1;
                    move_value[2] =2.9;
                    _move_linear_fix_z(move_value[0], move_value[1], move_value[2]);
                }
                else if(move_check == 10){//고도 유지하면서 좌측으로 이동
                    move_value[1] = -speed_1;
                    move_value[2] = 2.9;
                    _move_linear_fix_z(move_value[0], move_value[1], move_value[2]);
                }
                else if(move_check == 11){
                    move_value[0] = speed_1;//고도 유지하면서 전진
                    move_value[2] = 2.9;
                    _move_linear_fix_z(move_value[0], move_value[1], move_value[2]);
                }
                else if(move_check == 12){
                    move_value[0] = -speed_1;//고도 유지하면서 후진
                    move_value[2] = 2.9;
                    _move_linear_fix_z(move_value[0], move_value[1], move_value[2]);
                }
            }
        }
    }
}
