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

Eigen::MatrixXd Trasl= Eigen::MatrixXd::Identity(4, 4); 
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
   
    Trasl(0,3) = -delta_x;
    Trasl(1,3) = -delta_y;

    
    FOOT_base_frame << x,y,z,1;
    pos_ee = Trasl* FOOT_base_frame;


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

    
    Trasl(0,3) = delta_x;
    Trasl(1,3) = -delta_y;

    FOOT_base_frame << x,y,z,1;
    pos_ee = Trasl* FOOT_base_frame;


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

    Trasl(0,3) = -delta_x;
    Trasl(1,3) = delta_y;
    

    FOOT_base_frame << x,y,z,1;
    pos_ee = Trasl* FOOT_base_frame;


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
   
    Trasl(0,3) = delta_x;
    Trasl(1,3) = delta_y;
    
    FOOT_base_frame << x,y,z,1;
    pos_ee = Trasl* FOOT_base_frame;


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

    //q1_time = q1_time - 2*M_PI*floor((q1_time+M_PI)/(2*M_PI));

    q2_time = pow(time/time_f, 3) * (2*(q2_i - q2_f)) + pow(time/time_f, 2) * (3*(q2_f - q2_i)) + q2_i;

    //q2_time = q2_time - 2*M_PI*floor((q2_time+M_PI)/(2*M_PI));

    q3_time = pow(time/time_f, 3) * (2*(q3_i - q3_f)) + pow(time/time_f, 2) * (3*(q3_f - q3_i)) + q3_i;  

    //q3_time = q3_i + (q3_f - q3_i) *((1 - cos(M_PI*time/time_f))/2);
}

void move_base(double &x_i, double &y_i, double &z_i, double &roll_i, double &pitch_i, double &yaw_i,double &x_f, double &y_f, double &z_f, double &roll_f, double &pitch_f, double &yaw_f,double &x_time, double &y_time, double &z_time, double &roll_time, double &pitch_time, double &yaw_time,double &time, double &time_f)
{
    x_time = pow(time,3)* (2*(x_i -x_f)/pow(time_f,3)) + pow(time,2)* (3*(x_f -x_i)/pow(time_f,2)) + x_i;
    y_time = pow(time,3)* (2*(y_i -y_f)/pow(time_f,3)) + pow(time,2)* (3*(y_f -y_i)/pow(time_f,2)) + y_i;
    z_time = pow(time,3)* (2*(z_i -z_f)/pow(time_f,3)) + pow(time,2)* (3*(z_f -z_i)/pow(time_f,2)) + z_i;
    
    roll_time = pow(time,3)* (2*(roll_i -roll_f)/pow(time_f,3)) + pow(time,2)* (3*(roll_f -roll_i)/pow(time_f,2)) + roll_i;
    roll_time = roll_time - 2*M_PI*floor((roll_time+M_PI)/(2*M_PI));
    
    pitch_time = pow(time,3)* (2*(pitch_i -pitch_f)/pow(time_f,3)) + pow(time,2)* (3*(pitch_f -pitch_i)/pow(time_f,2)) + pitch_i;
    pitch_time = pitch_time - 2*M_PI*floor((pitch_time+M_PI)/(2*M_PI));
    
    yaw_time = pow(time,3)* (2*(yaw_i -yaw_f)/pow(time_f,3)) + pow(time,2)* (3*(yaw_f -yaw_i)/pow(time_f,2)) + yaw_i;
    yaw_time = yaw_time - 2*M_PI*floor((yaw_time+M_PI)/(2*M_PI));
}

/*void move_base(double &x_i, double &y_i, double &z_i, double &roll_i, double &pitch_i, double &yaw_i,double &x_f, double &y_f, double &z_f, double &roll_f, 
                double &pitch_f, double &yaw_f,double &x_time, double &y_time, double &z_time, double &roll_time, double &pitch_time, double &yaw_time,
                double &dot_x_time, double &dot_y_time, double &dot_z_time, double &dot_roll_time, double &dot_pitch_time, double &dot_yaw_time,
                double &dot_dot_x_time, double &dot_dot_y_time, double &dot_dot_z_time, double &dot_dot_roll_time, double &dot_dot_pitch_time, double &dot_dot_yaw_time,
                double &time, double &time_f)
{
    x_time = pow(time,5)* (6*(x_f -x_i)/pow(time_f,5)) + pow(time,4)* (-15*(x_f -x_i)/pow(time_f,4)) + pow(time,3)* (10*(x_f -x_i)/pow(time_f,3)) + x_i;
    dot_x_time = pow(time,4)* (30*(x_f -x_i)/pow(time_f,5)) + pow(time,2)* (30*(x_f -x_i)/pow(time_f,3)) + pow(time,3)* (-60*(x_f -x_i)/pow(time_f,4));
    dot_dot_x_time = pow(time,3)* (120*(x_f -x_i)/pow(time_f,5)) + pow(time,2)* (-180*(x_f -x_i)/pow(time_f,4)) + pow(time,1)* (60*(x_f -x_i)/pow(time_f,3));
    
    
    y_time = pow(time,5)* (6*(y_f -y_i)/pow(time_f,5)) + pow(time,4)* (-15*(y_f -y_i)/pow(time_f,4)) + pow(time,3)* (10*(y_f -y_i)/pow(time_f,3)) + y_i;
    dot_y_time = pow(time,4)* (30*(y_f -y_i)/pow(time_f,5)) + pow(time,2)* (30*(y_f -y_i)/pow(time_f,3)) + pow(time,3)* (-60*(y_f -y_i)/pow(time_f,4));
    dot_dot_y_time = pow(time,3)* (120*(y_f -y_i)/pow(time_f,5)) + pow(time,2)* (-180*(y_f -y_i)/pow(time_f,4)) + pow(time,1)* (60*(y_f -y_i)/pow(time_f,3));

    z_time = pow(time,5)* (6*(z_f -z_i)/pow(time_f,5)) + pow(time,4)* (-15*(z_f -z_i)/pow(time_f,4)) + pow(time,3)* (10*(z_f -z_i)/pow(time_f,3)) + z_i;
    dot_z_time = pow(time,4)* (30*(z_f -z_i)/pow(time_f,5)) + pow(time,2)* (30*(z_f -z_i)/pow(time_f,3)) + pow(time,3)* (-60*(z_f -z_i)/pow(time_f,4));
    dot_dot_z_time = pow(time,3)* (120*(z_f -z_i)/pow(time_f,5)) + pow(time,2)* (-180*(z_f -z_i)/pow(time_f,4)) + pow(time,1)* (60*(z_f -z_i)/pow(time_f,3));
    
    
    roll_time = pow(time,5)* (6*(roll_f -roll_i)/pow(time_f,5)) + pow(time,4)* (-15*(roll_f -roll_i)/pow(time_f,4)) + pow(time,3)* (10*(roll_f -roll_i)/pow(time_f,3)) + roll_i;
    dot_roll_time = pow(time,4)* (30*(roll_f -roll_i)/pow(time_f,5)) + pow(time,2)* (30*(roll_f -roll_i)/pow(time_f,3)) + pow(time,3)* (-60*(roll_f -roll_i)/pow(time_f,4));
    dot_dot_roll_time = pow(time,3)* (120*(roll_f -roll_i)/pow(time_f,5)) + pow(time,2)* (-180*(roll_f -roll_i)/pow(time_f,4)) + pow(time,1)* (60*(roll_f -roll_i)/pow(time_f,3));
    
    pitch_time = pow(time,5)* (6*(pitch_f -pitch_i)/pow(time_f,5)) + pow(time,4)* (-15*(pitch_f -pitch_i)/pow(time_f,4)) + pow(time,3)* (10*(pitch_f -pitch_i)/pow(time_f,3)) + pitch_i;
    dot_pitch_time = pow(time,4)* (30*(pitch_f -pitch_i)/pow(time_f,5)) + pow(time,2)* (30*(pitch_f -pitch_i)/pow(time_f,3)) + pow(time,3)* (-60*(pitch_f -pitch_i)/pow(time_f,4));
    dot_dot_pitch_time = pow(time,3)* (120*(pitch_f -pitch_i)/pow(time_f,5)) + pow(time,2)* (-180*(pitch_f -pitch_i)/pow(time_f,4)) + pow(time,1)* (60*(pitch_f -pitch_i)/pow(time_f,3));
    
    yaw_time = pow(time,5)* (6*(yaw_f -yaw_i)/pow(time_f,5)) + pow(time,4)* (-15*(yaw_f -yaw_i)/pow(time_f,4)) + pow(time,3)* (10*(yaw_f -yaw_i)/pow(time_f,3)) + yaw_i;
    dot_yaw_time = pow(time,4)* (30*(yaw_f -yaw_i)/pow(time_f,5)) + pow(time,2)* (30*(yaw_f -yaw_i)/pow(time_f,3)) + pow(time,3)* (-60*(yaw_f -yaw_i)/pow(time_f,4));
    dot_dot_yaw_time = pow(time,3)* (120*(yaw_f -yaw_i)/pow(time_f,5)) + pow(time,2)* (-180*(yaw_f -yaw_i)/pow(time_f,4)) + pow(time,1)* (60*(yaw_f -yaw_i)/pow(time_f,3));
 
}*/
void rotation(double &roll, double &pitch, double &yaw, Eigen::MatrixXd &Rot)
{
    Eigen::MatrixXd Rx= Eigen::MatrixXd::Zero(3, 3); 
    Eigen::MatrixXd Ry= Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd Rz= Eigen::MatrixXd::Zero(3, 3);

    Rx(0,0) = 1;
    Rx(1,1) = cos(roll);
    Rx(2,2) = cos(roll);
    Rx(1,2) = -sin(roll);
    Rx(2,1) = sin(roll);

    Rz(2,2) = 1;
    Rz(0,0) = cos(yaw);
    Rz(1,1) = cos(yaw);
    Rz(0,1) = -sin(yaw);
    Rz(1,0) = sin(yaw);


    Ry(0,0) = cos(pitch);
    Ry(1,1) = 1;
    Ry(2,2) = cos(pitch);
    Ry(0,2) = sin(pitch);
    Rx(2,0) = -sin(pitch);
    
    Rot = Rz * Ry * Rx;


}