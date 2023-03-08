#include <Convert.h>

double* quaternion_to_euler(double x, double y, double z, double w){
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);

    double *euler_rty = new double[3];
    m.getRPY(euler_rty[0], euler_rty[1], euler_rty[2]);

    return euler_rty;
}

double* euler_to_quaternion(double roll, double pitch, double yaw){
    
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    double *q = new double[4];  //quaternion (x, y, z, w)

    q[0]= sr * cp * cy - cr * sp * sy;  //x
    q[1]= cr * sp * cy + sr * cp * sy;  //y
    q[2]= cr * cp * sy - sr * sp * cy;  //z
    q[3] = cr * cp * cy + sr * sp * sy; //w

    return q;
}

double _norm(double goal[], double value[], int length){

    double result = 0;

    for (int i=0; i<length; i++){
        result += pow(goal[i] - value[i], 2);
    }
    result = sqrt(result);

    return result;
}

double degree_to_radian(double degree){
    return degree * M_PI / 180;
}

double radian_to_degree(double radian){
    return radian * 180 / M_PI;
}
