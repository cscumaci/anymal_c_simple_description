#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"


using namespace std;

Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4); 
Eigen::Matrix<double, 4, 1> pos_ee; 
Eigen::Matrix<double, 4, 1> FOOT_base_frame;   

double d, l, D, A, B, theta;

// da centro di massa a HAA

double delta_x = 0.2999;
double delta_y = 0.104;

//da HAA a HFE
 
double l6 = 0.0599;  // x
double l5 = 0.08381; // y


// da HFE a KFE
double l1 = 0.285;  // z
double l2 = 0.1003; // y

// da KFE a FOOT
 
double l4 = 0.08795;  // x
double l3 = 0.33797;  // z
double l7 = 0.01305;  // y


void Inverse_Kinematics_LF( double &x, double &y, double &z, double &q1, double &q2, double &q3)
{ 

    T(0,3) = -delta_x;
    T(1,3) = -delta_y;
    

    FOOT_base_frame << x,y,z,1;
    pos_ee = T * FOOT_base_frame;


    q1 = -atan2(pos_ee(1),pos_ee(2)) -atan2(-sqrt(pow(pos_ee(1),2) + pow(pos_ee(2),2) -pow((l2+l5+l7),2) ),l2+l5+l7) + M_PI_2; 
    q1 = q1 - 2*M_PI*floor((q1+M_PI)/(2*M_PI));

    d = sqrt(pow(pos_ee(2)*cos(q1) - pos_ee(1)*sin(q1),2)+ pow(pos_ee(0)-l6,2));

    l = sqrt(pow(l4,2)+pow(l3,2));

    D = (pow(d,2)-pow(l1,2)-pow(l,2))/(2*l1*l);

    theta = atan2(l4,l3);

    q3 = atan2(-sqrt(1-pow(D,2)),D) + atan2(l4,l3);
    q3 = q3 - 2*M_PI*floor((q3+M_PI)/(2*M_PI));

    B = -(l1 * (pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1)) + l*(pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1))*cos(q3-theta) + l*(pos_ee(0)-l6)*sin(q3-theta))/(pow(l,2)+pow(l1,2)+2*l*l1*cos(q3 - theta));

    A = -(l1 * (pos_ee(0) -l6)  +  l*(pos_ee(0)-l6)*cos(q3-theta) - l*(pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1))*sin(q3-theta))/(pow(l,2)+pow(l1,2)+2*l*l1*cos(q3 - theta));
    
    q2 =  atan2(A,B);
}



void Inverse_Kinematics_LH( double &x, double &y, double &z, double &q1, double &q2, double &q3)
{

    T(0,3) = delta_x;
    T(1,3) = -delta_y;

    FOOT_base_frame << x,y,z,1;
    pos_ee = T * FOOT_base_frame;


    q1 = -atan2(pos_ee(1),pos_ee(2)) -atan2(-sqrt(pow(pos_ee(1),2) + pow(pos_ee(2),2) -pow((l2+l5+l7),2) ),l2+l5+l7) + M_PI_2; 
    q1 = q1 - 2*M_PI*floor((q1+M_PI)/(2*M_PI));

    d = sqrt(pow(pos_ee(2)*cos(q1) - pos_ee(1)*sin(q1),2)+ pow(pos_ee(0)+l6,2));

    l = sqrt(pow(l4,2)+pow(l3,2));

    D = (pow(d,2)-pow(l1,2)-pow(l,2))/(2*l1*l);

    theta = atan2(l4,l3);

    q3 = atan2(sqrt(1-pow(D,2)),D) - atan2(l4,l3);
    q3 = q3 - 2*M_PI*floor((q3+M_PI)/(2*M_PI));

    B = -(l1 * (pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1)) + l*(pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1))*cos(q3+theta) + l*(pos_ee(0)+l6)*sin(q3+theta))/(pow(l,2)+pow(l1,2)+2*l*l1*cos(q3 + theta));

    A = -(l1 * (pos_ee(0) +l6)  +  l*(pos_ee(0)+l6)*cos(q3+theta) - l*(pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1))*sin(q3+theta))/(pow(l,2)+pow(l1,2)+2*l*l1*cos(q3 + theta));
    
    q2 =  atan2(A,B);
}

void Inverse_Kinematics_RF( double &x, double &y, double &z, double &q1, double &q2, double &q3)
{

    T(0,3) = -delta_x;
    T(1,3) = delta_y;
    

    FOOT_base_frame << x,y,z,1;
    pos_ee = T * FOOT_base_frame;


    q1 = -atan2(pos_ee(1),pos_ee(2)) - atan2(sqrt(pow(pos_ee(1),2) + pow(pos_ee(2),2) - pow((l2+l5+l7),2)),l2+l5+l7) - M_PI_2; 
    q1 = q1 - 2*M_PI*floor((q1+M_PI)/(2*M_PI));

    d = sqrt(pow(pos_ee(2)*cos(q1) - pos_ee(1)*sin(q1),2)+ pow(pos_ee(0)-l6,2));

    l = sqrt(pow(l4,2)+pow(l3,2));

    D = (pow(d,2)-pow(l1,2)-pow(l,2))/(2*l1*l);

    theta = atan2(l4,l3);

    q3 = atan2(-sqrt(1-pow(D,2)),D) + atan2(l4,l3);
    q3 = q3 - 2*M_PI*floor((q3+M_PI)/(2*M_PI));

    B = -(l1 * (pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1)) + l*(pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1))*cos(q3-theta) + l*(pos_ee(0)-l6)*sin(q3-theta))/(pow(l,2)+pow(l1,2)+2*l*l1*cos(q3 - theta));

    A = -(l1 * (pos_ee(0) -l6)  +  l*(pos_ee(0)-l6)*cos(q3-theta) - l*(pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1))*sin(q3-theta))/(pow(l,2)+pow(l1,2)+2*l*l1*cos(q3 - theta));
    
    q2 =  atan2(A,B);
}

void Inverse_Kinematics_RH( double &x, double &y, double &z, double &q1, double &q2, double &q3)
{
   
    T(0,3) = delta_x;
    T(1,3) = delta_y;
    
    FOOT_base_frame << x,y,z,1;
    pos_ee = T * FOOT_base_frame;


    q1 = -atan2(pos_ee(1),pos_ee(2)) -atan2(sqrt(pow(pos_ee(1),2) + pow(pos_ee(2),2) -pow((l2+l5+l7),2) ),l2+l5+l7) - M_PI_2; 
    q1 = q1 - 2*M_PI*floor((q1+M_PI)/(2*M_PI));

    d = sqrt(pow(pos_ee(2)*cos(q1) - pos_ee(1)*sin(q1),2)+ pow(pos_ee(0)+l6,2));

    l = sqrt(pow(l4,2)+pow(l3,2));

    D = (pow(d,2)-pow(l1,2)-pow(l,2))/(2*l1*l);

    theta = atan2(l4,l3);

    q3 = atan2(sqrt(1-pow(D,2)),D) - atan2(l4,l3);
    q3 = q3 - 2*M_PI*floor((q3+M_PI)/(2*M_PI));

    B = -(l1 * (pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1)) + l*(pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1))*cos(q3+theta) + l*(pos_ee(0)+l6)*sin(q3+theta))/(pow(l,2)+pow(l1,2)+2*l*l1*cos(q3 + theta));

    A = -(l1 * (pos_ee(0) +l6)  +  l*(pos_ee(0)+l6)*cos(q3+theta) - l*(pos_ee(2)*cos(q1) -  sin(q1)*pos_ee(1))*sin(q3+theta))/(pow(l,2)+pow(l1,2)+2*l*l1*cos(q3 + theta));
    
    q2 =  atan2(A,B);
    q2 = q2 - 2*M_PI*floor((q2+M_PI)/(2*M_PI));
}


void interpolation( double &q1_i,double &q2_i,double &q3_i, double &q1_f,double &q2_f,double &q3_f, double &q1_time, double &q2_time, double &q3_time, double &time, double &time_f)
{
    
    q1_time = pow(time/time_f, 3) * (2*(q1_i - q1_f)) + pow(time/time_f, 2) * (3*(q1_f - q1_i)) + q1_i;

    q1_time = q1_time - 2*M_PI*floor((q1_time+M_PI)/(2*M_PI));

    q2_time = pow(time/time_f, 3) * (2*(q2_i - q2_f)) + pow(time/time_f, 2) * (3*(q2_f - q2_i)) + q2_i;

    q2_time = q2_time - 2*M_PI*floor((q2_time+M_PI)/(2*M_PI));

    q3_time = pow(time/time_f, 3) * (2*(q3_i - q3_f)) + pow(time/time_f, 2) * (3*(q3_f - q3_i)) + q3_i;  

    q3_time = q3_i + (q3_f - q3_i) *((1 - cos(M_PI*time/time_f))/2);
}

void move_base(double &x_i, double &y_i, double &z_i, double &roll_i, double &pitch_i, double &yaw_i,double &x_f, double &y_f, double &z_f, double &roll_f, double &pitch_f, double &yaw_f,double &x_time, double &y_time, double &z_time, double &roll_time, double &pitch_time, double &yaw_time,double &time, double &time_f)
{
    x_time = pow(time,5)* (6*(x_f -x_i)/pow(time_f,5)) + pow(time,4)* (-15*(x_f -x_i)/pow(time_f,4)) + pow(time,3)* (10*(x_f -x_i)/pow(time_f,3)) + x_i;
    y_time = pow(time,5)* (6*(y_f -y_i)/pow(time_f,5)) + pow(time,4)* (-15*(y_f -y_i)/pow(time_f,4)) + pow(time,3)* (10*(y_f -y_i)/pow(time_f,3)) + y_i;
    z_time = pow(time,5)* (6*(z_f -z_i)/pow(time_f,5)) + pow(time,4)* (-15*(z_f -z_i)/pow(time_f,4)) + pow(time,3)* (10*(z_f -z_i)/pow(time_f,3)) + z_i;
    
    roll_time = pow(time,5)* (6*(roll_f -roll_i)/pow(time_f,5)) + pow(time,4)* (-15*(roll_f -roll_i)/pow(time_f,4)) + pow(time,3)* (10*(roll_f -roll_i)/pow(time_f,3)) + roll_i;
    
    
    pitch_time = pow(time,5)* (6*(pitch_f -pitch_i)/pow(time_f,5)) + pow(time,4)* (-15*(pitch_f -pitch_i)/pow(time_f,4)) + pow(time,3)* (10*(pitch_f -pitch_i)/pow(time_f,3)) + pitch_i;
    
    
    yaw_time = pow(time,5)* (6*(yaw_f -yaw_i)/pow(time_f,5)) + pow(time,4)* (-15*(yaw_f -yaw_i)/pow(time_f,4)) + pow(time,3)* (10*(yaw_f -yaw_i)/pow(time_f,3)) +  yaw_i;
    
}