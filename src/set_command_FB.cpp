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

bool friction_cone;
bool friction_cone_dietro;
bool friction_dietro;
bool friction_dietro_due;
bool scivolo;
bool scivolo_dietro;
bool scivolo_due;
bool scivolo_dietro_due;

bool fine;
bool endd;
bool freeze;
double delta_x_avanti;
double conta_friction;

void get_scivolo(const std_msgs::BoolConstPtr& msg)
{
 scivolo = msg->data;
}

void get_delta_x_avanti(const std_msgs::Float64ConstPtr& msg)
{
 delta_x_avanti = msg->data;
}

void get_conta_friction(const std_msgs::Float64ConstPtr& msg)
{
 conta_friction= msg->data;
}

void get_scivolo_dietro(const std_msgs::BoolConstPtr& msg)
{
 scivolo_dietro = msg->data;
}

void get_scivolo_due(const std_msgs::BoolConstPtr& msg)
{
 scivolo_due = msg->data;
}

void get_scivolo_dietro_due(const std_msgs::BoolConstPtr& msg)
{
 scivolo_dietro_due = msg->data;
}

void get_freeze(const std_msgs::BoolConstPtr& msg)
{
 freeze = msg->data;
}

void get_friction_cone(const std_msgs::BoolConstPtr& msg)
{
 friction_cone = msg->data;
}

void get_friction_cone_dietro(const std_msgs::BoolConstPtr& msg)
{
 friction_cone_dietro = msg->data;
}

void get_friction_dietro(const std_msgs::BoolConstPtr& msg)
{
 friction_dietro = msg->data;
}

void get_friction_dietro_due(const std_msgs::BoolConstPtr& msg)
{
 friction_dietro_due = msg->data;
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

  ros::Subscriber freeze_sub = n.subscribe<std_msgs::Bool>("/anymal/floating_base_controller/freeze_pub", 1, get_freeze);
  ros::Subscriber friction_cone_sub = n.subscribe<std_msgs::Bool>("/anymal/floating_base_controller/friction_cone_pub", 1, get_friction_cone);
  ros::Subscriber friction_cone_dietro_sub = n.subscribe<std_msgs::Bool>("/anymal/floating_base_controller/friction_cone_dietro_pub", 1, get_friction_cone_dietro);
  ros::Subscriber friction_dietro_sub = n.subscribe<std_msgs::Bool>("/anymal/floating_base_controller/friction_dietro_pub", 1, get_friction_dietro);
  ros::Subscriber friction_dietro_due_sub = n.subscribe<std_msgs::Bool>("/anymal/floating_base_controller/friction_dietro_due_pub", 1, get_friction_dietro_due);
  ros::Subscriber scivolo_sub = n.subscribe<std_msgs::Bool>("/anymal/floating_base_controller/scivolo_pub", 1, get_scivolo);
  ros::Subscriber scivolo_due_sub = n.subscribe<std_msgs::Bool>("/anymal/floating_base_controller/scivolo_due_pub", 1, get_scivolo_due);
  ros::Subscriber scivolo_dietro_sub = n.subscribe<std_msgs::Bool>("/anymal/floating_base_controller/scivolo_dietro_pub", 1, get_scivolo);
  ros::Subscriber scivolo_dietro_due_sub = n.subscribe<std_msgs::Bool>("/anymal/floating_base_controller/scivolo_dietro_due_pub", 1, get_scivolo_dietro_due);
  ros::Subscriber delta_x_avanti_sub = n.subscribe<std_msgs::Float64>("/anymal/floating_base_controller/delta_x_avanti_pub", 1000, get_delta_x_avanti);
  ros::Subscriber conta_friction_sub = n.subscribe<std_msgs::Float64>("/anymal/floating_base_controller/conta_friction_pub", 1000, get_conta_friction);
  
  
  
  ros::Publisher c_switch_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/get_c_switch_set_command", 1);
  ros::Publisher fine_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/get_fine_set_command", 1);
  ros::Publisher end_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/get_end_set_command", 1);
  ros::Publisher scivolo_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/get_scivolo", 1);
  ros::Publisher scivolo_due_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/get_scivolo_due", 1);
  ros::Publisher scivolo_dietro_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/get_scivolo_dietro", 1);
  ros::Publisher scivolo_dietro_due_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/get_scivolo_dietro_due", 1);
  ros::Publisher conta_friction_pub = n.advertise<std_msgs::Float64>("/anymal/floating_base_controller/conta_friction_pub", 1000);
  ros::Publisher delta_x_avanti_pub = n.advertise<std_msgs::Float64>("/anymal/floating_base_controller/delta_x_avanti_pub", 1000);


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

  

double delta_x_wp1_L = 0.085;
double delta_x_wp2_L = 0.085;
double delta_x_wp1_R = 0.085; 
double delta_x_wp2_R = 0.085; 

  double delta_x_wp1_Ll = 0.074;
  double delta_y_wp1_L = 0.0;
  double delta_h_wp1_L = 0.025;

  double delta_x_wp2_Ll = 0.074; 
  double delta_y_wp2_L = 0.0;
  double delta_h_wp2_L = -0.025;
  
  double delta_x_wp1_LF = 0.0742;
  double delta_y_wp1_LF = 0.0;
  double delta_h_wp1_LF = 0.025;

  double delta_x_wp2_LF = 0.0742;
  double delta_y_wp2_LF = 0.0;
  double delta_h_wp2_LF = -0.025;

  double delta_x_wp1_LH = 0.0755; //0.0755;
  double delta_y_wp1_LH = 0.0;
  double delta_h_wp1_LH = 0.025;

  double delta_x_wp2_LH = 0.0755; //0.0755;
  double delta_y_wp2_LH = 0.0;
  double delta_h_wp2_LH = -0.025;

  double delta_x_wp1_RF = 0.076;
  double delta_y_wp1_RF = 0.0;
  double delta_h_wp1_RF = 0.025;

  double delta_x_wp2_RF = 0.076;
  double delta_y_wp2_RF = 0.0;
  double delta_h_wp2_RF = -0.025;

  double delta_x_wp1_RH = 0.08;
  double delta_y_wp1_RH = 0.0;
  double delta_h_wp1_RH = 0.025; 

  double delta_x_wp2_RH = 0.08;
  double delta_y_wp2_RH = 0.0;
  double delta_h_wp2_RH = -0.025;

  double delta_y_wp1_RF_fd = -0.004;
  double delta_y_wp2_RF_fd = -0.004;


   double delta_x_wp1_RFF = 0.08;//0.04; //0.035;
   double delta_y_wp1_RFF = -0.0025;//2;//0.006;
   double delta_h_wp1_RFF = 0.025;

   double delta_x_wp2_RFF = 0.08; //0.04; // 0.035
   double delta_y_wp2_RFF = -0.0025; //2;//0.006;
   double delta_h_wp2_RFF = -0.025;  

   double delta_x_wp1_RHH = 0.076;
   double delta_y_wp1_RHH = -0.008; //-0.02;
   double delta_h_wp1_RHH = 0.025;

   double delta_x_wp2_RHH = 0.076;
   double delta_y_wp2_RHH = -0.008; //-0.02;
   double delta_h_wp2_RHH = -0.025; 


   double delta_x_wp1_LFF = 0.075;
   double delta_y_wp1_LFF = 0.0025; //0.008;//0.008;//0.006;//0.02;
   double delta_h_wp1_LFF = 0.025;

   double delta_x_wp2_LFF = 0.075;
   double delta_y_wp2_LFF = 0.0025; //0.008; //0.008; //0.006; //0.02;
   double delta_h_wp2_LFF = -0.025;

   double delta_x_wp1_LHH = 0.075; //0.072; //0.075;
   double delta_y_wp1_LHH = 0.0005;//0.002 -0.004; //-0.008; //-0.004;//-0.003; //0.0;//-0.03; //-0.03;
   double delta_h_wp1_LHH = 0.025;

   double delta_x_wp2_LHH = 0.075; //0.072; //0.075;
   double delta_y_wp2_LHH = 0.0005; //0.002 -0.004; //-0.008; //-0.004; //- 0.003; //0.0; //-0.03; //-0.03;
   double delta_h_wp2_LHH = -0.025;

   



  double x_wp_1;
  double y_wp_1;
  double z_wp_1;
  double x_wp_2;
  double y_wp_2;
  double z_wp_2;

  double conta = 0;
  

  double delta_x = 0.0765;// - delta_x_avanti; 
  double delta_xx = 0.0765; // - delta_x_avanti; 
   
  double x_wp_dxx = 0.075;
  double x_wp_dx = 0.09; //0.07; //0.09;
  double y_wp_dx = 0.05; 
  double z_wp_dx = 0.55;  
  double roll_wp_dx = -0.03; 
  double pitch_wp_dx = -0.01; 
  double yaw_wp_dx = 0.0; 

  double x_wp_sxx = 0.045;
  double x_wp_sx = 0.06; 
  double y_wp_sx = -0.05; //-0.07; //- 0.08; 
  double z_wp_sx = 0.55;  
  double roll_wp_sx = 0.02; 
  double pitch_wp_sx = -0.01; 
  double yaw_wp_sx = 0.0; 

  double x_time;
  double y_time;
  double z_time;
  double roll_time;
  double pitch_time;
  double yaw_time;

  double dot_x_time;
  double dot_y_time;
  double dot_z_time;
  double dot_roll_time;
  double dot_pitch_time;
  double dot_yaw_time;

  double dot_dot_x_time;
  double dot_dot_y_time;
  double dot_dot_z_time;
  double dot_dot_roll_time;
  double dot_dot_pitch_time;
  double dot_dot_yaw_time;

  double pitch = 0.2;

  double time_f = 0.5;
  double time_f_base = 2;

  double contatore = 0;

  double t;

  bool leg_up = 1;
  bool c_switch = 0;

  bool base_dx = 0;
  
 std_msgs::Bool c_switch_msg; 
 std_msgs::Bool fine_msg;  // c_switch
 std_msgs::Bool end_msg; 
 std_msgs::Bool scivolo_msg;

  Eigen::Matrix<double, 4, 1> move_leg;
  move_leg << 0 ,1 , 0, 0; 

  while (ros::ok())
    {
     
      t = ros::Time::now().toSec(); 

      if(t > 2 && t < 2.1)
      {
        c_switch = 1;
      }
       
      //if (c_switch == 1 && t < 95)
        
      if (c_switch == 1 && scivolo == 0 && scivolo_due == 0 && scivolo_dietro == 0 && scivolo_dietro_due == 0)
      {
           if ( move_leg(1) == 1)
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

          if(friction_cone == 0){
          if(conta == 0 || conta == 1){
            scivolo = 0;
            scivolo_due = 0;
            scivolo_dietro = 0; 
            scivolo_dietro_due = 0;
          
          if(leg_up == 1)
          {

            x_wp_1 = pos_foot_LH(0) + delta_x_wp1_Ll;
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

            x_wp_2 = x_wp_1 + delta_x_wp2_Ll;
            y_wp_2 = y_wp_1 + delta_y_wp2_L;
            z_wp_2 = z_wp_1 + delta_h_wp2_L;

            Inverse_Kinematics_LH(x_wp_2,y_wp_2,z_wp_2,q1_f_LH,q2_f_LH,q3_f_LH);  

            interpolation(q_i_LH(0), q_i_LH(1), q_i_LH(2), q1_f_LH, q2_f_LH, q3_f_LH, q1_time_LH, q2_time_LH, q3_time_LH, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 1,0,0,0;
              conta++; 
           
            }
          }
            
          
        }

        else{

           scivolo = 0;
            scivolo_due = 0;
            scivolo_dietro = 0; 
            scivolo_dietro_due = 0;
          
          if(leg_up == 1)
          {

            x_wp_1 = pos_foot_LH(0) + delta_x_wp1_LH;
            y_wp_1 = pos_foot_LH(1) + delta_y_wp1_LH;
            z_wp_1 = pos_foot_LH(2) + delta_h_wp1_LH;
            
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

            x_wp_2 = x_wp_1 + delta_x_wp2_LH;
            y_wp_2 = y_wp_1 + delta_y_wp2_LH;
            z_wp_2 = z_wp_1 + delta_h_wp2_LH;

            Inverse_Kinematics_LH(x_wp_2,y_wp_2,z_wp_2,q1_f_LH,q2_f_LH,q3_f_LH);  

            interpolation(q_i_LH(0), q_i_LH(1), q_i_LH(2), q1_f_LH, q2_f_LH, q3_f_LH, q1_time_LH, q2_time_LH, q3_time_LH, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 1,0,0,0;
           
            }
          }
            



        }
          }

        else{
          if( friction_cone == 1){
          if(leg_up == 1)
          {
            friction_cone = 1;
            x_wp_1 = pos_foot_LH(0) + delta_x_wp1_LHH;//delta_x_wp1_LH_fd;
            y_wp_1 = pos_foot_LH(1) + delta_y_wp1_LH;
            z_wp_1 = pos_foot_LH(2) + delta_h_wp1_LHH;
            
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
              friction_cone = 1;

            }
          }
        
          else 
          {
            friction_cone = 1;
            x_wp_2 = x_wp_1 + delta_x_wp2_LHH;// delta_x_wp2_LH_fd;
            y_wp_2 = y_wp_1 + delta_y_wp2_LH;
            z_wp_2 = z_wp_1 + delta_h_wp2_LHH;
            //cout<<"x_wp_sx_LL"<<endl<<x_wp_2<<endl;

            Inverse_Kinematics_LH(x_wp_2,y_wp_2,z_wp_2,q1_f_LH,q2_f_LH,q3_f_LH);  

            interpolation(q_i_LH(0), q_i_LH(1), q_i_LH(2), q1_f_LH, q2_f_LH, q3_f_LH, q1_time_LH, q2_time_LH, q3_time_LH, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 1,0,0,0;
              friction_cone = 1;
             
           }
          }
         }
        }
        }


        else if ( move_leg(0) == 1)
        {
          if(friction_cone == 0){  
          cout<<"normale"<<endl;
          scivolo = 0;
          scivolo_due = 0;
          scivolo_dietro = 0;
          scivolo_dietro_due = 0;
          if(leg_up == 1)
          {
          
            x_wp_1 = pos_foot_LF(0) + delta_x_wp1_LF;
            y_wp_1 = pos_foot_LF(1) + delta_y_wp1_LF;
            z_wp_1 = pos_foot_LF(2) + delta_h_wp1_LF;


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
          
          else{
        

            x_wp_2 = x_wp_1 + delta_x_wp2_LF;
            y_wp_2 = y_wp_1 + delta_y_wp2_LF;
            z_wp_2 = z_wp_1 + delta_h_wp2_LF;
            
            Inverse_Kinematics_LF(x_wp_2,y_wp_2,z_wp_2,q1_f_LF,q2_f_LF,q3_f_LF);  

            interpolation(q_i_LF(0), q_i_LF(1), q_i_LF(2), q1_f_LF, q2_f_LF, q3_f_LF, q1_time_LF, q2_time_LF, q3_time_LF, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
                contatore = 0;
                leg_up = 1;
                move_leg << 0,0,0,1;
              c_switch = 0;
              base_dx = 1;
              
              x_time = x_wp_sx;
              y_time = y_wp_sx;
              z_time = z_wp_sx;
              roll_time = roll_wp_sx;
              pitch_time = pitch_wp_sx;
              yaw_time = yaw_wp_sx;


              delta_x_wp1_L = delta_x_wp1_R;
              delta_x_wp2_L = delta_x_wp2_R;
                

              
            }

          }
        }
        else{
          if(leg_up == 1)
          {
            friction_cone = 1;

            x_wp_1 = pos_foot_LF(0) + delta_x_wp1_LF;
            y_wp_1 = pos_foot_LF(1) + delta_y_wp1_LFF;
            z_wp_1 = pos_foot_LF(2) + delta_h_wp1_LFF;


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
              friction_cone = 1;
              
            }

          }
          
          else{

            friction_cone = 1;
            x_wp_2 = x_wp_1 + delta_x_wp2_LF;
            y_wp_2 = y_wp_1 + delta_y_wp2_LFF;
            z_wp_2 = z_wp_1 + delta_h_wp2_LFF;
            
            Inverse_Kinematics_LF(x_wp_2,y_wp_2,z_wp_2,q1_f_LF,q2_f_LF,q3_f_LF);  

            interpolation(q_i_LF(0), q_i_LF(1), q_i_LF(2), q1_f_LF, q2_f_LF, q3_f_LF, q1_time_LF, q2_time_LF, q3_time_LF, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
                contatore = 0;
                leg_up = 1;
                move_leg << 0,0,0,1;
                c_switch = 0;
                base_dx = 1;
                friction_cone = 1;
                

                x_time = x_wp_sx;
                y_time = y_wp_sx;
                z_time = z_wp_sx;
                roll_time = roll_wp_sx;
                pitch_time = pitch_wp_sx;
                yaw_time = yaw_wp_sx;


                delta_x_wp1_L = delta_x_wp1_R;
                delta_x_wp2_L = delta_x_wp2_R;
                    

           
                
            }

          }
        }

    
        }

       
     else if(move_leg(3) == 1)
        {
          if(friction_cone == 0){
            scivolo = 0;
            scivolo_due = 0;
            scivolo_dietro = 0;
            scivolo_dietro_due = 0;

          if(leg_up == 1)
          {
            x_wp_1 = pos_foot_RH(0) + delta_x_wp1_RH;
            y_wp_1 = pos_foot_RH(1) + delta_y_wp1_RH;
            z_wp_1 = pos_foot_RH(2) + delta_h_wp1_RH;
           
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

            x_wp_2 = x_wp_1 + delta_x_wp2_RH;
            y_wp_2 = y_wp_1 + delta_y_wp2_RH;
            z_wp_2 = z_wp_1 + delta_h_wp2_RH;

            Inverse_Kinematics_RH(x_wp_2,y_wp_2,z_wp_2,q1_f_RH,q2_f_RH,q3_f_RH);  

            interpolation(q_i_RH(0), q_i_RH(1), q_i_RH(2), q1_f_RH, q2_f_RH, q3_f_RH, q1_time_RH, q2_time_RH, q3_time_RH, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 0,0,1,0;
              
          
            }
          }
        }

        else{
          if(friction_cone == 1){

          if(leg_up == 1)
          {
            friction_cone = 1;
            x_wp_1 = pos_foot_RH(0) + delta_x_wp1_RHH;
            y_wp_1 = pos_foot_RH(1) + delta_y_wp1_RH;
            z_wp_1 = pos_foot_RH(2) + delta_h_wp1_RHH;
           
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

             friction_cone = 1;
              
            }
          }
        
          else 
          {
            friction_cone = 1;
            x_wp_2 = x_wp_1 + delta_x_wp2_RHH; //delta_x_wp2_RH_fd;
            y_wp_2 = y_wp_1 + delta_y_wp2_RH;
            z_wp_2 = z_wp_1 + delta_h_wp2_RHH;

            Inverse_Kinematics_RH(x_wp_2,y_wp_2,z_wp_2,q1_f_RH,q2_f_RH,q3_f_RH);  

            interpolation(q_i_RH(0), q_i_RH(1), q_i_RH(2), q1_f_RH, q2_f_RH, q3_f_RH, q1_time_RH, q2_time_RH, q3_time_RH, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 0,0,1,0;
               friction_cone = 1;

            }
          }
        }


        }
        }
  
       else if(move_leg(2) == 1)
        {
          scivolo = 0;
          scivolo_due = 0;
          scivolo_dietro = 0;
          scivolo_dietro_due = 0;
          if(friction_cone == 0){
          if(leg_up == 1)
          {
          

            
            x_wp_1 = pos_foot_RF(0) + delta_x_wp1_RF;
            y_wp_1 = pos_foot_RF(1) + delta_y_wp1_RF;
            z_wp_1 = pos_foot_RF(2) + delta_h_wp1_RF;
    
           
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

            x_wp_2 = x_wp_1 + delta_x_wp2_RF;
            y_wp_2 = y_wp_1 + delta_y_wp2_RF;
            z_wp_2 = z_wp_1 + delta_h_wp2_RF;

            Inverse_Kinematics_RF(x_wp_2,y_wp_2,z_wp_2,q1_f_RF,q2_f_RF,q3_f_RF);  

            interpolation(q_i_RF(0), q_i_RF(1), q_i_RF(2), q1_f_RF, q2_f_RF, q3_f_RF, q1_time_RF, q2_time_RF, q3_time_RF, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 0,1,0,0;
              c_switch = 0;
              base_dx = 0;
              //endd = 0;
              conta_friction = 0;
             
              
               x_time = x_wp_dx; // * cos(theta) + z_wp_dx * sin(theta); ;
              y_time = y_wp_dx;
              z_time = z_wp_dx; // * cos(theta) - x_wp_dx * sin(theta);;
              roll_time = roll_wp_dx;
              pitch_time = pitch_wp_dx;
              yaw_time = yaw_wp_dx;
      
              delta_x_wp1_L = delta_x_wp1_R;
              delta_x_wp2_L = delta_x_wp2_R;
            }
          }
        }
        else{
          if(friction_cone == 1){
           if(leg_up == 1)
          {
            friction_cone = 1;
            scivolo = 0;
          scivolo_due = 0;
          scivolo_dietro = 0;
          scivolo_dietro_due = 0;
          
            x_wp_1 = pos_foot_RF(0) + delta_x_wp1_RF;
            y_wp_1 = pos_foot_RF(1) + delta_y_wp1_RFF;
            z_wp_1 = pos_foot_RF(2) + delta_h_wp1_RFF;
           
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

            x_wp_2 = x_wp_1 + delta_x_wp2_RF;
            y_wp_2 = y_wp_1 + delta_y_wp2_RFF;
            z_wp_2 = z_wp_1 + delta_h_wp2_RFF;

            Inverse_Kinematics_RF(x_wp_2,y_wp_2,z_wp_2,q1_f_RF,q2_f_RF,q3_f_RF);  

            interpolation(q_i_RF(0), q_i_RF(1), q_i_RF(2), q1_f_RF, q2_f_RF, q3_f_RF, q1_time_RF, q2_time_RF, q3_time_RF, contatore, time_f);    
          
            contatore = contatore + 0.01;

            if (contatore > time_f + 0.01)
            {
              contatore = 0;
              leg_up = 1;
              move_leg << 0,1,0,0;
              c_switch = 0;
              base_dx = 0;
              endd = 1;
              conta_friction++;
              //friction_cone = 0;
             
              
               x_time = x_wp_dx; // * cos(theta) + z_wp_dx * sin(theta); ;
              y_time = y_wp_dx;
              z_time = z_wp_dx; // * cos(theta) - x_wp_dx * sin(theta);;
              roll_time = roll_wp_dx;
              pitch_time = pitch_wp_dx;
              yaw_time = yaw_wp_dx;
      
              delta_x_wp1_L = delta_x_wp1_R;
              delta_x_wp2_L = delta_x_wp2_R;
            }
          }
        }
       
        }
      } 
      }
      

      else if(c_switch == 0 && t > 2.1 && base_dx == 1)
      {

        move_base(x_wp_sx,y_wp_sx,z_wp_sx,roll_wp_sx,pitch_wp_sx,yaw_wp_sx,x_wp_dx,y_wp_dx,z_wp_dx,roll_wp_dx,pitch_wp_dx,yaw_wp_dx,x_time, y_time, z_time, 
                  roll_time, pitch_time, yaw_time, dot_x_time, dot_y_time, dot_z_time, dot_roll_time, dot_pitch_time, dot_yaw_time,
                  dot_dot_x_time, dot_dot_y_time, dot_dot_z_time, dot_dot_roll_time, dot_dot_pitch_time, dot_dot_yaw_time, 
                  contatore,time_f_base);
        
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

      
      else if(c_switch == 0 && t > 2.1 && base_dx == 0)
      {
       
        move_base(x_wp_dx,y_wp_dx,z_wp_dx,roll_wp_dx,pitch_wp_dx,yaw_wp_dx,x_wp_sx,y_wp_sx,z_wp_sx,roll_wp_sx,pitch_wp_sx,yaw_wp_sx,x_time, y_time, z_time, 
                  roll_time, pitch_time, yaw_time,dot_x_time, dot_y_time, dot_z_time, dot_roll_time, dot_pitch_time, dot_yaw_time,
                  dot_dot_x_time, dot_dot_y_time, dot_dot_z_time, dot_dot_roll_time, dot_dot_pitch_time, dot_dot_yaw_time,contatore,time_f_base);
        
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
      base_state.velocity[0] = dot_x_time;
      base_state.effort[0] = dot_dot_x_time;
      
      base_state.name[1] = " y ";
      base_state.position[1] = y_time;
      base_state.velocity[1] = dot_y_time;
      base_state.effort[1] = dot_dot_y_time;
      
      base_state.name[2] = " z ";
      base_state.position[2] = z_time;
      base_state.velocity[2] = dot_z_time;
      base_state.effort[2] = dot_dot_z_time;
    
      
      base_state.name[3] = " roll ";
      base_state.position[3] = roll_time;
      base_state.velocity[3] = dot_z_time;
      base_state.effort[3] = dot_dot_z_time;
      
      base_state.name[4] = " pitch ";
      base_state.position[4] = pitch_time;
      base_state.velocity[4] = dot_pitch_time;
      base_state.effort[4] = dot_dot_pitch_time;
      
      base_state.name[5] = " yaw ";
      base_state.position[5] = yaw_time;
      base_state.velocity[5] = dot_yaw_time;
      base_state.effort[5] = dot_dot_yaw_time;



      base_pub.publish(base_state);
      joint_pub.publish(joint_state);


      c_switch_msg.data = c_switch;
      c_switch_pub.publish(c_switch_msg);

      fine_msg.data = fine;
      fine_pub.publish(fine_msg);

      end_msg.data = endd;
      end_pub.publish(end_msg);

      scivolo_msg.data = scivolo;
      scivolo_pub.publish(scivolo_msg);

       std_msgs::Float64 conta_friction_msg;
      conta_friction_msg.data = conta_friction;
      conta_friction_pub.publish(conta_friction_msg);

      std_msgs::Float64 delta_x_avanti_msg;
      delta_x_avanti_msg.data = delta_x_avanti;
      delta_x_avanti_pub.publish(delta_x_avanti_msg);

    


      //cout<<"contatore"<<endl<< contatore<<endl;
      cout<<"time"<<endl<< t <<endl;
      cout<<"fine"<< endl << fine <<endl;
      cout<<"endd"<<endl<<endd<<endl;
      cout<<"c_switch"<<endl<<c_switch<<endl;
      cout<<"friction_cone"<<endl<<friction_cone<<endl;  
      cout<<"friction_cone_dietro"<<endl<<friction_cone_dietro<<endl; 
      cout<<"friction_dietro"<<endl<<friction_dietro<<endl; 
      cout<<"friction_dietro_due"<<endl<<friction_dietro_due<<endl; 
      cout<<"scivolo"<<endl<<scivolo<<endl;
      cout<<"scivolo_due"<<endl<<scivolo_due<<endl;
      cout<<"scivolo_dietro"<<endl<<scivolo_dietro<<endl;
      cout<<"scivolo_dietro_due"<<endl<<scivolo_dietro_due<<endl;
      cout<<"freeze"<<endl<<freeze<<endl;

      ros::spinOnce();
      loop_rate.sleep();

    
    }

  return 0;
}



      






      
        