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
    //q1 = q1 - 2*M_PI*floor((q1+M_PI)/(2*M_PI));

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
    //q1 = q1 - 2*M_PI*floor((q1+M_PI)/(2*M_PI));

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
    //q1_time_dot = pow(time,2)/pow(time_f,3) * (6*(q1_i - q1_f)) + time/pow(time_f,2) * (6*(q1_f - q1_i));
    //q1_time_dot_dot = time/pow(time_f,3) * (12*(q1_i - q1_f)) + 1/pow(time_f,2) * (6*(q1_f - q1_i));
    //q1_time = q1_time - 2*M_PI*floor((q1_time+M_PI)/(2*M_PI));

    q2_time = pow(time/time_f, 3) * (2*(q2_i - q2_f)) + pow(time/time_f, 2) * (3*(q2_f - q2_i)) + q2_i;
    //q2_time_dot = pow(time,2)/pow(time_f,3) * (6*(q2_i - q2_f)) + time/pow(time_f,2) * (6*(q2_f - q2_i));
    //q2_time_dot_dot = time/pow(time_f,3) * (12*(q2_i - q2_f)) + 1/pow(time_f,2) * (6*(q2_f - q2_i));


    //q2_time = q2_time - 2*M_PI*floor((q2_time+M_PI)/(2*M_PI));

    q3_time = pow(time/time_f, 3) * (2*(q3_i - q3_f)) + pow(time/time_f, 2) * (3*(q3_f - q3_i)) + q3_i;  
    //q3_time_dot = pow(time,2)/pow(time_f,3) * (6*(q3_i - q3_f)) + time/pow(time_f,2) * (6*(q3_f - q3_i));
    //q3_time_dot_dot = time/pow(time_f,3) * (12*(q3_i - q3_f)) + 1/pow(time_f,2) * (6*(q3_f - q3_i));


    //q3_time = q3_i + (q3_f - q3_i) *((1 - cos(M_PI*time/time_f))/2);





}

void move_base(double &x_i, double &y_i, double &z_i, double &roll_i, double &pitch_i, double &yaw_i,double &x_f, double &y_f, double &z_f, double &roll_f, 
               double &pitch_f, double &yaw_f,double &x_time, double &y_time, double &z_time, double &roll_time, double &pitch_time, double &yaw_time,
               double &x_time_dot, double &y_time_dot, double &z_time_dot, double &roll_time_dot, double &pitch_time_dot, double &yaw_time_dot,
               double &x_time_dot_dot, double &y_time_dot_dot, double &z_time_dot_dot, double &roll_time_dot_dot, double &pitch_time_dot_dot, double &yaw_time_dot_dot,
               double &time, double &time_f)
{
    //x_time = pow(time,3)* (2*(x_i - x_f)/pow(time_f,3)) + pow(time,2)* (3*(x_f - x_i)/pow(time_f,2)) + x_i;
    x_time = x_i + 6 * (x_f - x_i) * (pow(time, 5)/pow(time_f,5)) - 15 * (x_f - x_i) * (pow(time, 4)/pow(time_f,4)) + 10 * (x_f - x_i) * (pow(time, 3)/pow(time_f,3));
    x_time_dot = 30 * (x_f - x_i) * (pow(time,2)/pow(time_f,3)) + 30 * (x_f - x_i) * (pow(time,4)/pow(time_f,5)) - 60 * (x_f - x_i) * (pow(time,3)/pow(time_f, 4));
    x_time_dot_dot = 120 * (x_f - x_i) * (pow(time,3)/pow(time_f,5)) - 180 * (x_f - x_i) * (pow(time,2)/pow(time_f,4)) + 60 * (x_f - x_i) * ((time)/pow(time_f,3));


    //y_time = pow(time,3)* (2*(y_i -y_f)/pow(time_f,3)) + pow(time,2)* (3*(y_f -y_i)/pow(time_f,2)) + y_i;
    y_time = y_i + 6 * (y_f - y_i) * (pow(time, 5)/pow(time_f,5)) - 15 * (y_f - y_i) * (pow(time, 4)/pow(time_f,4)) + 10 * (y_f - y_i) * (pow(time, 3)/pow(time_f,3));
    y_time_dot = 30 * (y_f - y_i) * (pow(time,2)/pow(time_f,3)) + 30 * (y_f - y_i) * (pow(time,4)/pow(time_f,5)) - 60 * (y_f - y_i) * (pow(time,3)/pow(time_f, 4));
    y_time_dot_dot = 120 * (y_f - y_i) * (pow(time,3)/pow(time_f,5)) - 180 * (y_f - y_i) * (pow(time,2)/pow(time_f,4)) + 60 * (y_f - y_i) * ((time)/pow(time_f,3));

   //z_time = pow(time,3)* (2*(z_i -z_f)/pow(time_f,3)) + pow(time,2)* (3*(z_f -z_i)/pow(time_f,2)) + z_i;
    z_time = x_i + 6 * (z_f - z_i) * (pow(time, 5)/pow(time_f,5)) - 15 * (z_f - z_i) * (pow(time, 4)/pow(time_f,4)) + 10 * (z_f - z_i) * (pow(time, 3)/pow(time_f,3));
    z_time_dot = 30 * (z_f - z_i) * (pow(time,2)/pow(time_f,3)) + 30 * (z_f - z_i) * (pow(time,4)/pow(time_f,5)) - 60 * (z_f - z_i) * (pow(time,3)/pow(time_f, 4));
    z_time_dot_dot = 120 * (z_f - z_i) * (pow(time,3)/pow(time_f,5)) - 180 * (z_f - z_i) * (pow(time,2)/pow(time_f,4)) + 60 * (z_f - z_i) * ((time)/pow(time_f,3));
    
    
    //roll_time = pow(time,3)* (2*(roll_i -roll_f)/pow(time_f,3)) + pow(time,2)* (3*(roll_f -roll_i)/pow(time_f,2)) + roll_i;
    roll_time = roll_i + 6 * (roll_f - roll_i) * (pow(time, 5)/pow(time_f,5)) - 15 * (roll_f - roll_i) * (pow(time, 4)/pow(time_f,4)) + 10 * (roll_f - roll_i) * (pow(time, 3)/pow(time_f,3));
    roll_time = roll_time - 2*M_PI*floor((roll_time+M_PI)/(2*M_PI));
    roll_time_dot = 30 * (roll_f - roll_i) * (pow(time,2)/pow(time_f,3)) + 30 * (roll_f - roll_i) * (pow(time,4)/pow(time_f,5)) - 60 * (roll_f - roll_i) * (pow(time,3)/pow(time_f, 4));
    roll_time_dot_dot = 120 * (roll_f - roll_i) * (pow(time,3)/pow(time_f,5)) - 180 * (roll_f - roll_i) * (pow(time,2)/pow(time_f,4)) + 60 * (roll_f - roll_i) * ((time)/pow(time_f,3));

    
    //pitch_time = pow(time,3)* (2*(pitch_i -pitch_f)/pow(time_f,3)) + pow(time,2)* (3*(pitch_f -pitch_i)/pow(time_f,2)) + pitch_i;
    pitch_time = pitch_i + 6 * (pitch_f - pitch_i) * (pow(time, 5)/pow(time_f,5)) - 15 * (pitch_f - pitch_i) * (pow(time, 4)/pow(time_f,4)) + 10 * (pitch_f - pitch_i) * (pow(time, 3)/pow(time_f,3));
    pitch_time = pitch_time - 2*M_PI*floor((pitch_time+M_PI)/(2*M_PI));
    pitch_time_dot = 30 * (pitch_f - pitch_i) * (pow(time,2)/pow(time_f,3)) + 30 * (pitch_f - pitch_i) * (pow(time,4)/pow(time_f,5)) - 60 * (pitch_f - pitch_i) * (pow(time,3)/pow(time_f, 4));
    pitch_time_dot_dot = 120 * (pitch_f - pitch_i) * (pow(time,3)/pow(time_f,5)) - 180 * (pitch_f - pitch_i) * (pow(time,2)/pow(time_f,4)) + 60 * (pitch_f - pitch_i) * ((time)/pow(time_f,3));


    //yaw_time = pow(time,3)* (2*(yaw_i -yaw_f)/pow(time_f,3)) + pow(time,2)* (3*(yaw_f -yaw_i)/pow(time_f,2)) + yaw_i;
    yaw_time = yaw_i + 6 * (yaw_f - yaw_i) * (pow(time, 5)/pow(time_f,5)) - 15 * (yaw_f - yaw_i) * (pow(time, 4)/pow(time_f,4)) + 10 * (yaw_f - yaw_i) * (pow(time, 3)/pow(time_f,3));
    yaw_time = yaw_time - 2*M_PI*floor((yaw_time+M_PI)/(2*M_PI));
    yaw_time_dot = 30 * (yaw_f - yaw_i) * (pow(time,2)/pow(time_f,3)) + 30 * (yaw_f - yaw_i) * (pow(time,4)/pow(time_f,5)) - 60 * (yaw_f - yaw_i) * (pow(time,3)/pow(time_f, 4));
    yaw_time_dot_dot = 120 * (yaw_f - yaw_i) * (pow(time,3)/pow(time_f,5)) - 180 * (yaw_f - yaw_i) * (pow(time,2)/pow(time_f,4)) + 60 * (yaw_f - yaw_i) * ((time)/pow(time_f,3));
}   
