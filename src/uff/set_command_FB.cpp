#include<iostream>
#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <cmath>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/LinkState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "inverse_kinematics.h"


using namespace std;
Eigen::Matrix<double, 3, 1> q_i_LF;
Eigen::Matrix<double, 3, 1> q_i_LH;
Eigen::Matrix<double, 3, 1> q_i_RF;
Eigen::Matrix<double, 3, 1> q_i_RH;

Eigen::Matrix<double, 3, 1> pos_foot_LF;
Eigen::Matrix<double, 3, 1> pos_foot_LH;
Eigen::Matrix<double, 3, 1> pos_foot_RF;
Eigen::Matrix<double, 3, 1> pos_foot_RH;

Eigen::Matrix<double, 6, 1> pos_base;
bool scivolo = 0;

void get_scivolo(const std_msgs::BoolConstPtr& msg)
{
 scivolo = msg->data;
}

void get_pos_base(const geometry_msgs::TwistConstPtr& msg)
{
        pos_base(0) = msg->linear.x;
        pos_base(1) = msg->linear.y;
        pos_base(2) = msg->linear.z;

        pos_base(3)= msg->angular.x;
        pos_base(4)= msg->angular.y;
        pos_base(5)= msg->angular.z;

}

void get_q_i_L(const geometry_msgs::TwistConstPtr& msg)
{
        q_i_LF(0) = msg->linear.x;
        q_i_LF(1) = msg->linear.y;
        q_i_LF(2) = msg->linear.z;

        q_i_LH(0) = msg->angular.x;
        q_i_LH(1) = msg->angular.y;
        q_i_LH(2) = msg->angular.z; 

}

void get_q_i_R(const geometry_msgs::TwistConstPtr& msg)
{
        q_i_RF(0) = msg->linear.x;
        q_i_RF(1) = msg->linear.y;
        q_i_RF(2) = msg->linear.z;

        q_i_RH(0) = msg->angular.x;
        q_i_RH(1) = msg->angular.y;
        q_i_RH(2) = msg->angular.z;

}

void get_L_foot_pos(const geometry_msgs::TwistConstPtr& msg)
{
      pos_foot_LF(0) = msg->linear.x;
      pos_foot_LF(1) = msg->linear.y;
      pos_foot_LF(2) = msg->linear.z;

      pos_foot_LH(0) = msg->angular.x;
      pos_foot_LH(1) = msg->angular.y;
      pos_foot_LH(2) = msg->angular.z;
}

void get_R_foot_pos(const geometry_msgs::TwistConstPtr& msg)
{
      pos_foot_RF(0) = msg->linear.x;
      pos_foot_RF(1) = msg->linear.y;
      pos_foot_RF(2) = msg->linear.z;

      pos_foot_RH(0) = msg->angular.x;
      pos_foot_RH(1) = msg->angular.y;
      pos_foot_RH(2) = msg->angular.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_command"); //The name of the node
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/anymal/floating_base_controller/command", 1);
  ros::Publisher base_pub = n.advertise<sensor_msgs::JointState>("/anymal/floating_base_controller/command_base", 1);
  
  ros::Subscriber q_i_L_sub = n.subscribe<geometry_msgs::Twist>("/anymal/floating_base_controller/q_i_L_pub", 1, get_q_i_L);    
  ros::Subscriber q_i_R_sub = n.subscribe<geometry_msgs::Twist>("/anymal/floating_base_controller/q_i_R_pub", 1, get_q_i_R);

  ros::Subscriber pos_foot_L_sub = n.subscribe<geometry_msgs::Twist>("/anymal/floating_base_controller/pos_foot_L_pub", 1, get_L_foot_pos);
  ros::Subscriber pos_foot_R_sub = n.subscribe<geometry_msgs::Twist>("/anymal/floating_base_controller/pos_foot_R_pub", 1, get_R_foot_pos);

  ros::Subscriber pos_base_sub = n.subscribe<geometry_msgs::Twist>("/anymal/floating_base_controller/pos_base_pub", 1, get_pos_base);
  ros::Subscriber scivolo_sub = n.subscribe<std_msgs::Bool>("/anymal/floating_base_controller/scivolo_pub", 1, get_scivolo);
  
  ros::Publisher c_switch_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/get_c_switch_set_command", 1);
  
  sensor_msgs::JointState joint_state;
  sensor_msgs::JointState base_state;
  
  

  double q1_time_LF; //= q_i_LF(0);
  double q2_time_LF; //= q_i_LF(1);
  double q3_time_LF; //= q_i_LF(2);

  double q1_f_LF;
  double q2_f_LF;
  double q3_f_LF;

  double q1_time_LH;
  double q2_time_LH;
  double q3_time_LH;

  double q1_f_LH;
  double q2_f_LH;
  double q3_f_LH;

  double q1_time_RF;
  double q2_time_RF;
  double q3_time_RF;

  double q1_f_RF;
  double q2_f_RF;
  double q3_f_RF;

  double q1_time_RH;
  double q2_time_RH;
  double q3_time_RH;

  double q1_f_RH;
  double q2_f_RH;
  double q3_f_RH;

  
  double delta_x_wp1_L = 0.04;
  double delta_y_wp1_L = 0.0;
  double delta_h_wp1_L = 0.025;

  double delta_x_wp2_L = 0.04;
  double delta_y_wp2_L = 0.0;
  double delta_h_wp2_L = -0.025;

  double delta_x_wp1_R = 0.06;
  double delta_y_wp1_R = 0.0;
  double delta_h_wp1_R = 0.025;

  double delta_x_wp2_R = 0.06;
  double delta_y_wp2_R = 0.0;
  double delta_h_wp2_R = -0.025;

  double x_wp_1;
  double y_wp_1;
  double z_wp_1;
  double x_wp_2;
  double y_wp_2;
  double z_wp_2;

  double delta_x = 0.06;

  double x_wp_dx = 0.09; //0.09; 
  double y_wp_dx = 0.08; //0.08; 
  double z_wp_dx = 0.55;  
  double roll_wp_dx = -0.02; //-0.02; 
  double pitch_wp_dx = -0.01; //-0.01; 
  double yaw_wp_dx = 0.0; 

  double x_wp_sx = 0.03; //0.03; 
  double y_wp_sx = -0.08; //- 0.08; 
  double z_wp_sx = 0.55;  
  double roll_wp_sx = 0.02; //0.02; 
  double pitch_wp_sx = -0.01; //-0.01; 
  double yaw_wp_sx = 0.0; 

  double x_time;
  double y_time;
  double z_time;
  double roll_time;
  double pitch_time;
  double yaw_time;

  double x_time_dot;
  double y_time_dot;
  double z_time_dot;
  double roll_time_dot;
  double pitch_time_dot;
  double yaw_time_dot;


  double x_time_dot_dot;
  double y_time_dot_dot;
  double z_time_dot_dot;
  double roll_time_dot_dot;
  double pitch_time_dot_dot;
  double yaw_time_dot_dot;

  double pitch = 0.2;

  double time_f = 2;
  double time_f_base = 0.5;

  double contatore = 0;

  double t;

  bool leg_up = 1;
  bool c_switch = 0;

  bool base_dx = 0;
  
  std_msgs::Bool c_switch_msg;  // c_switch

  Eigen::Matrix<double, 4, 1> move_leg;
  move_leg << 1 ,0 , 0, 0; 

  while (ros::ok())
    {
     
      t = ros::Time::now().toSec();

      if(t > 2 && t < 2.1)
      {
        c_switch = 1;
      }
      
      if (c_switch == 1 && scivolo == 0)
      {
        
        if ( move_leg(0) == 1 )
        {
          q1_time_LF = q_i_LF(0);
          q2_time_LF = q_i_LF(1);
          q3_time_LF = q_i_LF(2);
          
          q1_time_LH = q_i_LH(0);
          q2_time_LH = q_i_LH(1);
          q3_time_LH = q_i_LH(2);

          q1_time_RF = q_i_RF(0);
          q2_time_RF = q_i_RF(1);
          q3_time_RF = q_i_RF(2);
          
          q1_time_RH = q_i_RH(0);
          q2_time_RH = q_i_RH(1);
          q3_time_RH = q_i_RH(2);
          
         
            
          if(leg_up == 1)
          {
            x_wp_1 = pos_foot_LF(0) + delta_x_wp1_L;
            y_wp_1 = pos_foot_LF(1) + delta_y_wp1_L;
            z_wp_1 = pos_foot_LF(2) + delta_h_wp1_L;


            Inverse_Kinematics_LF(x_wp_1,y_wp_1,z_wp_1,q1_f_LF,q2_f_LF,q3_f_LF);  

            interpolation(q_i_LF(0), q_i_LF(1), q_i_LF(2), q1_f_LF, q2_f_LF, q3_f_LF, q1_time_LF, q2_time_LF, q3_time_LF, contatore, time_f);    

            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
            
              q_i_LF(0) = q1_f_LF;
              q_i_LF(1) = q2_f_LF;
              q_i_LF(2) = q3_f_LF;
              contatore = 0;
              leg_up = 0;
              
            }

          }
        
          else 
          {

            x_wp_2 = x_wp_1 + delta_x_wp2_L;
            y_wp_2 = y_wp_1 + delta_y_wp2_L;
            z_wp_2 = z_wp_1 + delta_h_wp2_L;
            
            Inverse_Kinematics_LF(x_wp_2,y_wp_2,z_wp_2,q1_f_LF,q2_f_LF,q3_f_LF);  

            interpolation(q_i_LF(0), q_i_LF(1), q_i_LF(2), q1_f_LF, q2_f_LF, q3_f_LF, q1_time_LF, q2_time_LF, q3_time_LF, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
                contatore = 0;
                leg_up = 1;
                move_leg << 0,1,0,0;
                
            }
          }
        }

        else if(move_leg(1) == 1)
        { 
          if(leg_up == 1)
          {

            x_wp_1 = pos_foot_LH(0) + delta_x_wp1_L;
            y_wp_1 = pos_foot_LH(1) + delta_y_wp1_L;
            z_wp_1 = pos_foot_LH(2) + delta_h_wp1_L;
            
            Inverse_Kinematics_LH(x_wp_1,y_wp_1,z_wp_1,q1_f_LH,q2_f_LH,q3_f_LH);  

            interpolation(q_i_LH(0), q_i_LH(1), q_i_LH(2), q1_f_LH, q2_f_LH, q3_f_LH, q1_time_LH, q2_time_LH, q3_time_LH, contatore, time_f);    

            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
            
              q_i_LH(0) = q1_f_LH;
              q_i_LH(1) = q2_f_LH;
              q_i_LH(2) = q3_f_LH;
              contatore = 0;
              leg_up = 0;

            }
          }
        
          else 
          {

            x_wp_2 = x_wp_1 + delta_x_wp2_L;
            y_wp_2 = y_wp_1 + delta_y_wp2_L;
            z_wp_2 = z_wp_1 + delta_h_wp2_L;

            Inverse_Kinematics_LH(x_wp_2,y_wp_2,z_wp_2,q1_f_LH,q2_f_LH,q3_f_LH);  

            interpolation(q_i_LH(0), q_i_LH(1), q_i_LH(2), q1_f_LH, q2_f_LH, q3_f_LH, q1_time_LH, q2_time_LH, q3_time_LH, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 0,0,1,0;
              c_switch = 0;
              base_dx = 1;

              x_time = x_wp_sx;
              y_time = y_wp_sx;
              z_time = z_wp_sx;
              roll_time = roll_wp_sx;
              pitch_time = pitch_wp_sx;
              yaw_time = yaw_wp_sx;

              /*x_time = x_wp_sx * cos(theta) + z_wp_sx * sin(theta);
              y_time = y_wp_sx;
              z_time = z_wp_sx* cos(theta) - x_wp_sx *sin(theta);
              roll_time = roll_wp_sx;
              pitch_time = pitch_wp_sx;
              yaw_time = yaw_wp_sx;*/


              delta_x_wp1_L = delta_x_wp1_R;
              delta_x_wp2_L = delta_x_wp2_R;
            }
          }
            
          
        }
        
       
        else if(move_leg(2) == 1)
        {
          if(leg_up == 1)
          {
            x_wp_1 = pos_foot_RF(0) + delta_x_wp1_R;
            y_wp_1 = pos_foot_RF(1) + delta_y_wp1_R;
            z_wp_1 = pos_foot_RF(2) + delta_h_wp1_R;
           
            Inverse_Kinematics_RF(x_wp_1,y_wp_1,z_wp_1,q1_f_RF,q2_f_RF,q3_f_RF);  

            interpolation(q_i_RF(0), q_i_RF(1), q_i_RF(2), q1_f_RF, q2_f_RF, q3_f_RF, q1_time_RF, q2_time_RF, q3_time_RF, contatore, time_f); 

            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
            
              q_i_RF(0) = q1_f_RF;
              q_i_RF(1) = q2_f_RF;
              q_i_RF(2) = q3_f_RF;
              contatore = 0;
              leg_up = 0;
              
            }
          }
        
          else 
          {

            x_wp_2 = x_wp_1 + delta_x_wp2_R;
            y_wp_2 = y_wp_1 + delta_y_wp2_R;
            z_wp_2 = z_wp_1 + delta_h_wp2_R;

            Inverse_Kinematics_RF(x_wp_2,y_wp_2,z_wp_2,q1_f_RF,q2_f_RF,q3_f_RF);  

            interpolation(q_i_RF(0), q_i_RF(1), q_i_RF(2), q1_f_RF, q2_f_RF, q3_f_RF, q1_time_RF, q2_time_RF, q3_time_RF, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 0,0,0,1;
            }
          }
        }

        else if(move_leg(3) == 1)
        {
          if(leg_up == 1)
          {
            x_wp_1 = pos_foot_RH(0) + delta_x_wp1_R;
            y_wp_1 = pos_foot_RH(1) + delta_y_wp1_R;
            z_wp_1 = pos_foot_RH(2) + delta_h_wp1_R;
           
            Inverse_Kinematics_RH(x_wp_1,y_wp_1,z_wp_1,q1_f_RH,q2_f_RH,q3_f_RH);  

            interpolation(q_i_RH(0), q_i_RH(1), q_i_RH(2), q1_f_RH, q2_f_RH, q3_f_RH, q1_time_RH, q2_time_RH, q3_time_RH, contatore, time_f); 

            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
            
              q_i_RH(0) = q1_f_RH;
              q_i_RH(1) = q2_f_RH;
              q_i_RH(2) = q3_f_RH;
              contatore = 0;
              leg_up = 0;
              
            }
          }
        
          else 
          {

            x_wp_2 = x_wp_1 + delta_x_wp2_R;
            y_wp_2 = y_wp_1 + delta_y_wp2_R;
            z_wp_2 = z_wp_1 + delta_h_wp2_R;

            Inverse_Kinematics_RH(x_wp_2,y_wp_2,z_wp_2,q1_f_RH,q2_f_RH,q3_f_RH);  

            interpolation(q_i_RH(0), q_i_RH(1), q_i_RH(2), q1_f_RH, q2_f_RH, q3_f_RH, q1_time_RH, q2_time_RH, q3_time_RH, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 1,0,0,0;
              c_switch = 0;
              base_dx = 0;

              x_time = x_wp_dx; // * cos(theta) + z_wp_dx * sin(theta); ;
              y_time = y_wp_dx;
              z_time = z_wp_dx; // * cos(theta) - x_wp_dx * sin(theta);;
              roll_time = roll_wp_dx;
              pitch_time = pitch_wp_dx;
              yaw_time = yaw_wp_dx;
            }
          }
        }

      }

      if (c_switch == 1 && scivolo == 1)
      {
        cout<<"prova"<<endl;
        
        if ( move_leg(0) == 1 )
        {
          q1_time_LF = q_i_LF(0);
          q2_time_LF = q_i_LF(1);
          q3_time_LF = q_i_LF(2);
          
          q1_time_LH = q_i_LH(0);
          q2_time_LH = q_i_LH(1);
          q3_time_LH = q_i_LH(2);

          q1_time_RF = q_i_RF(0);
          q2_time_RF = q_i_RF(1);
          q3_time_RF = q_i_RF(2);
          
          q1_time_RH = q_i_RH(0);
          q2_time_RH = q_i_RH(1);
          q3_time_RH = q_i_RH(2);
          
         
            
          if(leg_up == 1)
          {
            x_wp_1 = pos_foot_LF(0);
            y_wp_1 = pos_foot_LF(1);
            z_wp_1 = pos_foot_LF(2);


            Inverse_Kinematics_LF(x_wp_1,y_wp_1,z_wp_1,q1_f_LF,q2_f_LF,q3_f_LF);  

            interpolation(q_i_LF(0), q_i_LF(1), q_i_LF(2), q1_f_LF, q2_f_LF, q3_f_LF, q1_time_LF, q2_time_LF, q3_time_LF, contatore, time_f);    

            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
            
              q_i_LF(0) = q1_f_LF;
              q_i_LF(1) = q2_f_LF;
              q_i_LF(2) = q3_f_LF;
              contatore = 0;
              leg_up = 0;
              
            }

          }
        
          else 
          {

            x_wp_2 = x_wp_1;
            y_wp_2 = y_wp_1;
            z_wp_2 = z_wp_1;
            
            Inverse_Kinematics_LF(x_wp_2,y_wp_2,z_wp_2,q1_f_LF,q2_f_LF,q3_f_LF);  

            interpolation(q_i_LF(0), q_i_LF(1), q_i_LF(2), q1_f_LF, q2_f_LF, q3_f_LF, q1_time_LF, q2_time_LF, q3_time_LF, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
                contatore = 0;
                leg_up = 1;
                move_leg << 0,1,0,0;
                
            }
          }
        }

        else if(move_leg(1) == 1)
        { 
          if(leg_up == 1)
          {

            x_wp_1 = pos_foot_LH(0) + delta_x_wp1_L;
            y_wp_1 = pos_foot_LH(1) + delta_y_wp1_L;
            z_wp_1 = pos_foot_LH(2) + delta_h_wp1_L;
            
            Inverse_Kinematics_LH(x_wp_1,y_wp_1,z_wp_1,q1_f_LH,q2_f_LH,q3_f_LH);  

            interpolation(q_i_LH(0), q_i_LH(1), q_i_LH(2), q1_f_LH, q2_f_LH, q3_f_LH, q1_time_LH, q2_time_LH, q3_time_LH, contatore, time_f);    

            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
            
              q_i_LH(0) = q1_f_LH;
              q_i_LH(1) = q2_f_LH;
              q_i_LH(2) = q3_f_LH;
              contatore = 0;
              leg_up = 0;

            }
          }
        
          else 
          {

            x_wp_2 = x_wp_1 + delta_x_wp2_L;
            y_wp_2 = y_wp_1 + delta_y_wp2_L;
            z_wp_2 = z_wp_1 + delta_h_wp2_L;

            Inverse_Kinematics_LH(x_wp_2,y_wp_2,z_wp_2,q1_f_LH,q2_f_LH,q3_f_LH);  

            interpolation(q_i_LH(0), q_i_LH(1), q_i_LH(2), q1_f_LH, q2_f_LH, q3_f_LH, q1_time_LH, q2_time_LH, q3_time_LH, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 0,0,1,0;
              c_switch = 0;
              base_dx = 1;

              x_time = x_wp_sx;
              y_time = y_wp_sx;
              z_time = z_wp_sx;
              roll_time = roll_wp_sx;
              pitch_time = pitch_wp_sx;
              yaw_time = yaw_wp_sx;

              /*x_time = x_wp_sx * cos(theta) + z_wp_sx * sin(theta);
              y_time = y_wp_sx;
              z_time = z_wp_sx* cos(theta) - x_wp_sx *sin(theta);
              roll_time = roll_wp_sx;
              pitch_time = pitch_wp_sx;
              yaw_time = yaw_wp_sx;*/


              delta_x_wp1_L = delta_x_wp1_R;
              delta_x_wp2_L = delta_x_wp2_R;
            }
          }
            
          
        }
        
       
        else if(move_leg(2) == 1)
        {
          if(leg_up == 1)
          {
            x_wp_1 = pos_foot_RF(0);
            y_wp_1 = pos_foot_RF(1);
            z_wp_1 = pos_foot_RF(2);
           
            Inverse_Kinematics_RF(x_wp_1,y_wp_1,z_wp_1,q1_f_RF,q2_f_RF,q3_f_RF);  

            interpolation(q_i_RF(0), q_i_RF(1), q_i_RF(2), q1_f_RF, q2_f_RF, q3_f_RF, q1_time_RF, q2_time_RF, q3_time_RF, contatore, time_f); 

            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
            
              q_i_RF(0) = q1_f_RF;
              q_i_RF(1) = q2_f_RF;
              q_i_RF(2) = q3_f_RF;
              contatore = 0;
              leg_up = 0;
              
            }
          }
        
          else 
          {

            x_wp_2 = x_wp_1;
            y_wp_2 = y_wp_1;
            z_wp_2 = z_wp_1;

            Inverse_Kinematics_RF(x_wp_2,y_wp_2,z_wp_2,q1_f_RF,q2_f_RF,q3_f_RF);  

            interpolation(q_i_RF(0), q_i_RF(1), q_i_RF(2), q1_f_RF, q2_f_RF, q3_f_RF, q1_time_RF, q2_time_RF, q3_time_RF, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 0,0,0,1;
            }
          }
        }

        else if(move_leg(3) == 1)
        {
          if(leg_up == 1)
          {
            x_wp_1 = pos_foot_RH(0) + delta_x_wp1_R;
            y_wp_1 = pos_foot_RH(1) + delta_y_wp1_R;
            z_wp_1 = pos_foot_RH(2) + delta_h_wp1_R;
           
            Inverse_Kinematics_RH(x_wp_1,y_wp_1,z_wp_1,q1_f_RH,q2_f_RH,q3_f_RH);  

            interpolation(q_i_RH(0), q_i_RH(1), q_i_RH(2), q1_f_RH, q2_f_RH, q3_f_RH, q1_time_RH, q2_time_RH, q3_time_RH, contatore, time_f); 

            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
            
              q_i_RH(0) = q1_f_RH;
              q_i_RH(1) = q2_f_RH;
              q_i_RH(2) = q3_f_RH;
              contatore = 0;
              leg_up = 0;
              
            }
          }
        
          else 
          {

            x_wp_2 = x_wp_1 + delta_x_wp2_R;
            y_wp_2 = y_wp_1 + delta_y_wp2_R;
            z_wp_2 = z_wp_1 + delta_h_wp2_R;

            Inverse_Kinematics_RH(x_wp_2,y_wp_2,z_wp_2,q1_f_RH,q2_f_RH,q3_f_RH);  

            interpolation(q_i_RH(0), q_i_RH(1), q_i_RH(2), q1_f_RH, q2_f_RH, q3_f_RH, q1_time_RH, q2_time_RH, q3_time_RH, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 1,0,0,0;
              c_switch = 0;
              base_dx = 0;

              x_time = x_wp_dx; // * cos(theta) + z_wp_dx * sin(theta); ;
              y_time = y_wp_dx;
              z_time = z_wp_dx; // * cos(theta) - x_wp_dx * sin(theta);;
              roll_time = roll_wp_dx;
              pitch_time = pitch_wp_dx;
              yaw_time = yaw_wp_dx;
            }
          }
        }

      }


      else if(c_switch == 0 && t > 2.1 && base_dx == 1 )
      {

        move_base(x_wp_sx,y_wp_sx,z_wp_sx,roll_wp_sx,pitch_wp_sx,yaw_wp_sx,x_wp_dx,y_wp_dx,z_wp_dx,roll_wp_dx,pitch_wp_dx,yaw_wp_dx,x_time, y_time, z_time, 
                  roll_time, pitch_time, yaw_time, x_time_dot, y_time_dot, z_time_dot, roll_time_dot, pitch_time_dot, yaw_time_dot, 
                  x_time_dot_dot, y_time_dot_dot, z_time_dot_dot, roll_time_dot_dot, pitch_time_dot_dot, yaw_time_dot_dot, contatore,time_f_base);
        
        contatore = contatore + 0.01;

        if(contatore > time_f_base + 0.01)
        {
            contatore = 0;
            c_switch = 1;

            x_wp_sx = x_wp_dx + delta_x;

            q1_time_LF = q_i_LF(0);
            q2_time_LF = q_i_LF(1);
            q3_time_LF = q_i_LF(2);

            q1_time_LH = q_i_LH(0);
            q2_time_LH = q_i_LH(1);
            q3_time_LH = q_i_LH(2);

            q1_time_RF = q_i_RF(0);
            q2_time_RF = q_i_RF(1);
            q3_time_RF = q_i_RF(2);

            q1_time_RH = q_i_RH(0);
            q2_time_RH = q_i_RH(1);
            q3_time_RH = q_i_RH(2);

        }
      }
      else if(c_switch == 0 && t > 2.1 && base_dx == 0 )
      {
       
        move_base(x_wp_dx,y_wp_dx,z_wp_dx,roll_wp_dx,pitch_wp_dx,yaw_wp_dx,x_wp_sx,y_wp_sx,z_wp_sx,roll_wp_sx,pitch_wp_sx,yaw_wp_sx,x_time, y_time, z_time,
                  roll_time, pitch_time, yaw_time, x_time_dot, y_time_dot, z_time_dot, roll_time_dot, pitch_time_dot, yaw_time_dot, 
                  x_time_dot_dot, y_time_dot_dot, z_time_dot_dot, roll_time_dot_dot, pitch_time_dot_dot, yaw_time_dot_dot, contatore,time_f_base);
        
        contatore = contatore + 0.01;

        if(contatore > time_f_base + 0.01)
        {
            contatore = 0;
            c_switch = 1;

            x_wp_dx = x_wp_sx + delta_x;

            q1_time_LF = q_i_LF(0);
            q2_time_LF = q_i_LF(1);
            q3_time_LF = q_i_LF(2);

            q1_time_LH = q_i_LH(0);
            q2_time_LH = q_i_LH(1);
            q3_time_LH = q_i_LH(2);

            q1_time_RF = q_i_RF(0);
            q2_time_RF = q_i_RF(1);
            q3_time_RF = q_i_RF(2);

            q1_time_RH = q_i_RH(0);
            q2_time_RH = q_i_RH(1);
            q3_time_RH = q_i_RH(2);

        }
      }

      
      base_state.name.resize(6);
      base_state.position.resize(6);
      base_state.velocity.resize(6);
      base_state.effort.resize(6);

      joint_state.name.resize(12);
      joint_state.position.resize(12);
      joint_state.velocity.resize(12);
      joint_state.effort.resize(12);

      joint_state.name[0]="LF_HAA";
      joint_state.position[0] = q1_time_LF; 
      joint_state.velocity[0] = 0.0;
      joint_state.effort[0] = 0.0;

      joint_state.name[1]="LF_HFE";
      joint_state.position[1] = q2_time_LF; 
      joint_state.velocity[1] = 0.0;
      joint_state.effort[1] = 0.0;
      
      joint_state.name[2]="LF_KFE";
      joint_state.position[2] = q3_time_LF;
      joint_state.velocity[2] = 0.0;
      joint_state.effort[2] = 0.0;



      joint_state.name[3]="LH_HAA";
      joint_state.position[3] = q1_time_LH; 
      joint_state.effort[3] = 0.0;
      
      joint_state.name[4]="LH_HFE";
      joint_state.position[4] = q2_time_LH; 
      joint_state.velocity[4] = 0.0;
      joint_state.effort[4] = 0.0;
      
      joint_state.name[5]="LH_KFE";
      joint_state.position[5] = q3_time_LH; 
      joint_state.velocity[5] = 0.0;
      joint_state.effort[5] = 0.0;
      


      joint_state.name[6]="RF_HAA";
      joint_state.position[6] = q1_time_RF; 
      joint_state.velocity[6] = 0.0;
      joint_state.effort[6] = 0.0;
      
      joint_state.name[7]="RF_HFE";
      joint_state.position[7] = q2_time_RF; 
      joint_state.velocity[7] = 0.0;
      joint_state.effort[7] = 0.0;
      
      joint_state.name[8]="RF_KFE";
      joint_state.position[8] = q3_time_RF; 
      joint_state.effort[8] = 0.0;
      

      joint_state.name[9]="RH_HAA";
      joint_state.position[9] = q1_time_RH; 
      joint_state.velocity[9] = 0.0;
      joint_state.effort[9] = 0.0;
      
      joint_state.name[10]="RH_HFE";
      joint_state.position[10] = q2_time_RH; 
      joint_state.velocity[10] = 0.0;
      joint_state.effort[10] = 0.0;
      
      joint_state.name[11]="RH_KFE";
      joint_state.position[11] =  q3_time_RH; 
      joint_state.velocity[11] = 0.0;
      joint_state.effort[11] = 0.0;


      base_state.name[0] = " x ";
      base_state.position[0] = x_time;
      base_state.velocity[0] = x_time_dot;
      base_state.effort[0] = x_time_dot_dot;
      
      base_state.name[1] = " y ";
      base_state.position[1] = y_time;
      base_state.velocity[1] = y_time_dot;
      base_state.effort[1] = y_time_dot_dot;
      
      base_state.name[2] = " z ";
      base_state.position[2] = z_time;
      base_state.velocity[2] = z_time_dot;
      base_state.effort[2] = z_time_dot_dot;
      
      base_state.name[3] = " roll ";
      base_state.position[3] = roll_time;
      base_state.velocity[3] = roll_time_dot;
      base_state.effort[3] = roll_time_dot_dot;
      
      base_state.name[4] = " pitch ";
      base_state.position[4] = pitch_time;
      base_state.velocity[4] = pitch_time_dot;
      base_state.effort[4] = pitch_time_dot_dot;
      
      base_state.name[5] = " yaw ";
      base_state.position[5] = yaw_time;
      base_state.velocity[5] = yaw_time_dot;
      base_state.effort[5] = yaw_time_dot_dot;



      base_pub.publish(base_state);
      joint_pub.publish(joint_state);


      c_switch_msg.data = c_switch;
      c_switch_pub.publish(c_switch_msg);

      //cout<<"contatore"<<endl<< contatore<<endl;
      cout<<"time"<<endl<< t <<endl;
      cout<<"scivolo"<<endl<<scivolo<<endl;
          

      ros::spinOnce();
      loop_rate.sleep();

    
    }

  return 0;
}



      






      