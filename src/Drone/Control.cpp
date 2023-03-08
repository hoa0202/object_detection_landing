/**
 * WASD : move
 * U/J  : up/down
 * T/G  : Takeoff/land(Ground)
 * O/P  : Offboard mode arming / Change mode to Offboard
 * N/M  : Mission input / Mission mode arming
 * 0    : On/Off tracking
 */


#include <drone.h>
#include <keyboard.h>

void test(double A[], double B[], int length){
    for (int i=0; i<length; i++){
        printf("[%d] A_value : %lf, B_value : %lf\n", i, A[i], B[i]);
    }
}

void Drone::_control()
{   
    ROS_INFO("Drone control...");

    init_keyboard();    // 키보드 char 1개 입력받도록 터미널 설정 변경

    if (!_connect()){   // px4 연결 확인
        close_keyboard();
        return;
    }

    float gimbal_pitch = 0.0;

    float gimbal_pitch_Actuator = 0.0;
    float gimbal_yaw_Actuator = 0.0;

    float gimbal_pitch_Gazebo = 0.0;

    while (ros::ok())
    {
        if (_kbhit())
        {
            int ch = _getch();
            switch (ch)
            {
                case 'q':
                    ROS_INFO("Exit node...");
                    close_keyboard();
                    ros::shutdown();
                    break;

                case 't': 
                    _takeoff(2.0);
                    break;

                case 'g':
                    _land();
                    break;
                    
                case 'u':
                    _move_linear(0,0,1);
                    break;
                case 'j':
                    _move_linear(0,0,-1);
                    break;

                case 'w':
                    _move_linear(1,0,0);
                    break;

                case 'a':
                    _move_linear(0,-1,0);
                    break;

                case 's':
                    _move_linear(-1,0,0);
                    break;

                case 'd':
                    _move_linear(0,1,0);
                    break;

                case '1':
                    _move_angular(degree_to_radian(1));
                    break;

                case '3':
                    _move_angular(degree_to_radian(-1));
                    break;

                case 'o':
                    _arm("OFFBOARD");
                    break;
                
                case 'c':
                    if(current_state.mode == "OFFBOARD"){
                        _change_mode("AUTO.MISSION");
                    } else if (current_state.mode == "AUTO.MISSION"){
                        _set_currentPose();
                        _change_mode("OFFBOARD");
                        _move_linear(2,0,0);
                    }else{
                        _set_currentPose();
                        _change_mode("OFFBOARD");
                    }
                    break;

                case 'm':
                    _arm("AUTO.MISSION");
                    break;

                case '0':
                    isTracking = !isTracking;
                    nh.setParam("drone/isTracking", isTracking);
                    ROS_INFO("Param changed : isTracking = %d", isTracking);         
                    break;

                case '8':
                    gimbal_pitch+=1.0;
                    if (gimbal_pitch >= -12.0)
                        gimbal_pitch = -12.0;
                    ROS_INFO("gimbal pitch : %.2f", gimbal_pitch);

                    

                    PublishMountControl(gimbal_pitch, 0.0);
                    break;
                    
                case '9':
                    gimbal_pitch-=1.0;
                    if (gimbal_pitch <= -28.0)
                        gimbal_pitch = -28.0;
                    ROS_INFO("gimbal pitch : %.2f", gimbal_pitch);

                    PublishMountControl(gimbal_pitch, 0.0);
                    break;
/*
                case '4':
                    gimbal_pitch_Actuator += 0.1;
                    gimbal_yaw_Actuator = 0.0;

                    if (gimbal_pitch_Actuator >= 0.0)
                        gimbal_pitch_Actuator = 0.0;
                    ROS_INFO("%.2f", gimbal_pitch_Actuator);
                    PublishActuatorSetpoints(gimbal_pitch_Actuator, gimbal_yaw_Actuator);
                    break;

                case '5':
                    gimbal_pitch_Actuator -= 0.1;
                    gimbal_yaw_Actuator = 0.0;

                    if (gimbal_pitch_Actuator <= -1.6)
                        gimbal_pitch_Actuator = -1.6;
                    ROS_INFO("%.2f", gimbal_pitch_Actuator);
                    PublishActuatorSetpoints(gimbal_pitch_Actuator, gimbal_yaw_Actuator);
                    break;
*/
                case '6': 
                    PublishMountControl(-28, 0.0); // In Gazebo, Gimbal Control
                    //ControlGimbal_Jetson(gimbal_pitch_Jetson, 0.0); // In Jetson, GImbal Control
                    break;
/*
                case '7':
                    PublishMountControl(--gimbal_pitch_Gazebo, false); // In Jetson, GImbal Control
                    ControlGimbal_Jetson(gimbal_pitch_Jetson, true); // In Gazebo, Gimbal Control
                    break;
*/
            }
        }
        else if (isTakeoff)
            {
                _hover();
            }
            

        ros::spinOnce();
        rate.sleep();
    }

    return;
}