#include<iostream>
#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "std_msgs/Float64MultiArray.h"
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


Eigen::MatrixXd q_i_LF = Eigen::MatrixXd::Zero(3,1);
Eigen::MatrixXd q_i_LH = Eigen::MatrixXd::Zero(3,1);
Eigen::MatrixXd q_i_RF = Eigen::MatrixXd::Zero(3,1);
Eigen::MatrixXd q_i_RH = Eigen::MatrixXd::Zero(3,1);

Eigen::Matrix<double, 3, 1> pos_foot_LF;
Eigen::Matrix<double, 3, 1> pos_foot_LH;
Eigen::Matrix<double, 3, 1> pos_foot_RF;
Eigen::Matrix<double, 3, 1> pos_foot_RH;

Eigen::Matrix<double, 3, 1> pos_foot_LF_w;
Eigen::Matrix<double, 3, 1> pos_foot_LH_w;
Eigen::Matrix<double, 3, 1> pos_foot_RF_w;
Eigen::Matrix<double, 3, 1> pos_foot_RH_w;

Eigen::Matrix<double, 3, 1> pos_base;

Eigen::Matrix<double, 3, 1> pos_base_time;
Eigen::VectorXd pos_foot = Eigen::VectorXd::Zero(12);
vector<double> pos_foot_vec (12, 0);



int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_command"); //The name of the node
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher pos_foot_pub = n.advertise<std_msgs::Float64MultiArray>("pos_foot", 1);

  sensor_msgs::JointState joint_state;
  std_msgs::Float64MultiArray msg;
  msg.data.resize(12);

 

  double q1_time_LF = 0; 
  double q2_time_LF = 0; 
  double q3_time_LF = 0; 

  double q1_f_LF;
  double q2_f_LF;
  double q3_f_LF;

  double q1_time_LH = 0;
  double q2_time_LH = 0;
  double q3_time_LH = 0;

  double q1_f_LH;
  double q2_f_LH;
  double q3_f_LH;

  double q1_time_RF = 0;
  double q2_time_RF = 0;
  double q3_time_RF = 0;

  double q1_f_RF;
  double q2_f_RF;
  double q3_f_RF;

  double q1_time_RH = 0;
  double q2_time_RH = 0;
  double q3_time_RH = 0;

  double q1_f_RH;
  double q2_f_RH;
  double q3_f_RH;

  //Pianificazione camminata iniziale
   
   double delta_x_wp1_LFF = 0.028;
   double delta_y_wp1_LFF = 0.0; 
   double delta_h_wp1_LFF = 0.1;

   double delta_x_wp2_LFF = 0.028;
   double delta_y_wp2_LFF = 0.0; 
   double delta_h_wp2_LFF = -0.1;

   double delta_x_wp1_RFF = 0.06;
   double delta_y_wp1_RFF = 0.0;
   double delta_h_wp1_RFF = 0.1;

   double delta_x_wp2_RFF = 0.06; 
   double delta_y_wp2_RFF = 0.0; 
   double delta_h_wp2_RFF = -0.1;  


   double delta_x_wp1_LHH = 0.112; 
   double delta_y_wp1_LHH = 0.0;
   double delta_h_wp1_LHH = 0.1;

   double delta_x_wp2_LHH = 0.112; 
   double delta_y_wp2_LHH = 0.0; 
   double delta_h_wp2_LHH = -0.1;

   double delta_x_wp1_RHH = 0.15; 
   double delta_y_wp1_RHH = 0.0;
   double delta_h_wp1_RHH = 0.1;

   double delta_x_wp2_RHH = 0.15; 
   double delta_y_wp2_RHH = 0.0; 
   double delta_h_wp2_RHH = -0.1; 


  //Pianificazione su terreno
   double delta_x_wp1_LF = 0.08;
   double delta_y_wp1_LF = 0.0; 
   double delta_h_wp1_LF = 0.1;

   double delta_x_wp2_LF = 0.08; 
   double delta_y_wp2_LF = 0.0; 
   double delta_h_wp2_LF = -0.1;
  
   double delta_x_wp1_RF = 0.067; 
   double delta_y_wp1_RF = -0.0;
   double delta_h_wp1_RF = 0.1;

   double delta_x_wp2_RF = 0.067; 
   double delta_y_wp2_RF = 0.0; 
   double delta_h_wp2_RF = -0.1; 


   double delta_x_wp1_LH = 0.111;
   double delta_y_wp1_LH = 0.0;
   double delta_h_wp1_LH = 0.1;

   double delta_x_wp2_LH = 0.111; 
   double delta_y_wp2_LH = 0.0; 
   double delta_h_wp2_LH = -0.1;

   double delta_x_wp1_RH = 0.1168;
   double delta_y_wp1_RH = 0.0; 
   double delta_h_wp1_RH = 0.1;

   double delta_x_wp2_RH = 0.1168;
   double delta_y_wp2_RH = 0.0; 
   double delta_h_wp2_RH = -0.1; 


  double contt = 0;
  double cont_noslip = 1;
  double cont_slip = 1;

  double x_wp_1;
  double y_wp_1;
  double z_wp_1;
  double x_wp_2;
  double y_wp_2; 
  double z_wp_2;
 
  double delta_x = 0.032;
  double delta_xx = 0.10;
  double delta_yy = 0.055;
  double delta_xx_corr = 0.09; 
  double delta_yy_corr = 0.0715; 



  double x_wp_dx = 0.09; 
  double y_wp_dx = 0.05; 
  double z_wp_dx = 0.445; 
  double roll_wp_dx = -0.03; 
  double pitch_wp_dx = 0.0025;
  double yaw_wp_dx = 0.0; 

  double x_wp_sx = 0.06; 
  double y_wp_sx = -0.05; 
  double z_wp_sx = 0.445;  
  double roll_wp_sx = 0.03; 
  double pitch_wp_sx = 0.0025; 
  double yaw_wp_sx = 0.0; 

  double x_wp_i = 0.0; 
  double y_wp_i = 0.0; 
  double z_wp_i = 0.628000020980835;  
  double roll_wp_i = 0.0; 
  double pitch_wp_i = 0.0; 
  double yaw_wp_i = 0.0; 

  double x_wp_sx_f = 0.05; 
  double y_wp_sx_f = -0.05; 
  double z_wp_sx_f = 0.445;  
  double roll_wp_sx_f = 0.0; 
  double pitch_wp_sx_f = 0.0025; 
  double yaw_wp_sx_f = 0.0;

  double x_time = 0;
  double y_time = 0;
  double z_time = 0.628000020980835;
  double roll_time = 0;
  double pitch_time = 0;
  double yaw_time = 0;


  double time_f = 0.8;
  double time_f_base = 1.5;//2;

  double contatore = 0;
  double cont = 0;

  double t;

  bool leg_up = 1;
  bool c_switch = 0;
  
  bool init = 0;

  bool base_dx = 0;

  bool scivolo = 0;

  double CoM_x = 0;
  double CoM_y = 0;
 

  Eigen::MatrixXd Rot = Eigen::MatrixXd::Zero(3,3);

  Eigen::Matrix<double, 4, 1> move_leg;


  //posizione piedi iniziale presa da Rviz
  pos_foot_LF << 0.44775, 0.30113, -0.62298;
  pos_foot_LH << -0.44775, 0.30113, -0.62298;
  pos_foot_RF << 0.44775, -0.30113, -0.62293;
  pos_foot_RH << -0.44775, -0.30113, -0.62293;

  

  move_leg << 0 ,1 , 0, 0; 

  pos_base << x_wp_i, y_wp_i, z_wp_i;

  rotation(roll_wp_i, pitch_wp_i, yaw_wp_i, Rot);

  pos_foot_LF_w = pos_base +  Rot*pos_foot_LF;
      
  pos_foot_LH_w = pos_base +  Rot*pos_foot_LH;
      
  pos_foot_RF_w = pos_base +  Rot*pos_foot_RF;

  pos_foot_RH_w = pos_base +  Rot*pos_foot_RH;

  while (ros::ok())
  {
    t = ros::Time::now().toSec();

    //pianificazione inizio camminata senza terreno scivoloso
    //per mettere il robot in una configurazione più stabile

    if (c_switch == 1 && scivolo == 0)
    {

      if(move_leg(1) == 1)
      { 
        
        if(leg_up == 1)
        {

          x_wp_1 = pos_foot_LH(0) + delta_x_wp1_LHH;
          y_wp_1 = pos_foot_LH(1) + delta_y_wp1_LHH;
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
 
          }
        }
      
        else 
        {

          x_wp_2 = x_wp_1 + delta_x_wp2_LHH;
          y_wp_2 = y_wp_1 + delta_y_wp2_LHH;
          z_wp_2 = z_wp_1 + delta_h_wp2_LHH;


          Inverse_Kinematics_LH(x_wp_2,y_wp_2,z_wp_2,q1_f_LH,q2_f_LH,q3_f_LH);  

          interpolation(q_i_LH(0), q_i_LH(1), q_i_LH(2), q1_f_LH, q2_f_LH, q3_f_LH, q1_time_LH, q2_time_LH, q3_time_LH, contatore, time_f);    
        
          contatore = contatore + 0.01;

         

          if (contatore > time_f + 0.01)
          {
            contatore = 0;
            leg_up = 1;
            move_leg << 1,0,0,0;

            pos_foot_LH(0) = x_wp_2;
            pos_foot_LH(1) = y_wp_2;
            pos_foot_LH(2) = z_wp_2;
          }
        }
          
      }

      else if (move_leg(0) == 1 )
      {
        
          
        if(leg_up == 1)
        {
          x_wp_1 = pos_foot_LF(0) + delta_x_wp1_LFF;
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
            
          }
 
        }
      
        else
        {

  
  
          x_wp_2 = x_wp_1 + delta_y_wp2_LFF;
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
 
              pos_foot_LF(0) = x_wp_2;
              pos_foot_LF(1) = y_wp_2;
              pos_foot_LF(2) = z_wp_2;
              

     

              pos_base << x_time, y_time, z_time;

              rotation(roll_time, pitch_time, yaw_time, Rot);

              pos_foot_LF_w = pos_base +  Rot*pos_foot_LF;
                  
              pos_foot_LH_w = pos_base +  Rot*pos_foot_LH;
                  
              pos_foot_RF_w = pos_base +  Rot*pos_foot_RF;

              pos_foot_RH_w = pos_base +  Rot*pos_foot_RH;


              //cout<<"scarto"<<endl<<sqrt(pow(pos_foot_LF(0),2) + pow(x_time,2))<<endl;
          }
        }
      }
      
      

      else if(move_leg(3) == 1)
      {
        if(leg_up == 1)
        {
          x_wp_1 = pos_foot_RH(0) + delta_x_wp1_RHH;
          y_wp_1 = pos_foot_RH(1) + delta_y_wp1_RHH;
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
            
          }
        }
      
        else 
        {

          x_wp_2 = x_wp_1 + delta_x_wp2_RHH;
          y_wp_2 = y_wp_1 + delta_y_wp2_RHH;
          z_wp_2 = z_wp_1 + delta_h_wp2_RHH;

          Inverse_Kinematics_RH(x_wp_2,y_wp_2,z_wp_2,q1_f_RH,q2_f_RH,q3_f_RH);  

          interpolation(q_i_RH(0), q_i_RH(1), q_i_RH(2), q1_f_RH, q2_f_RH, q3_f_RH, q1_time_RH, q2_time_RH, q3_time_RH, contatore, time_f);    
        
          contatore = contatore + 0.01;


          if (contatore > time_f + 0.01)
          {
            contatore = 0;
            leg_up = 1;
            move_leg << 0,0,1,0;

            pos_foot_RH(0) = x_wp_2;
            pos_foot_RH(1) = y_wp_2;
            pos_foot_RH(2) = z_wp_2;
            
          }
        }
      }

      else if(move_leg(2) == 1)
      {
        if(leg_up == 1)
        {
          x_wp_1 = pos_foot_RF(0) + delta_x_wp1_RFF;
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

          x_wp_2 = x_wp_1 + delta_x_wp2_RFF;
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

            pos_foot_RF(0) = x_wp_2;
            pos_foot_RF(1) = y_wp_2;
            pos_foot_RF(2) = z_wp_2;


           // delta_x_wp1_LFF = delta_x_wp1_RFF + 0.03;
           // delta_x_wp2_LFF = delta_x_wp2_RFF + 0.03;


            pos_base << x_time, y_time, z_time;

            rotation(roll_time, pitch_time, yaw_time, Rot);

            pos_foot_LF_w = pos_base +  Rot*pos_foot_LF;
                
            pos_foot_LH_w = pos_base +  Rot*pos_foot_LH;
                
            pos_foot_RF_w = pos_base +  Rot*pos_foot_RF;

            pos_foot_RH_w = pos_base +  Rot*pos_foot_RH;



 
          }
        }
      }

    } 


    //spostamento della base per raggiungere la configurazione iniziale 
    //prima della camminata 

    else if(c_switch == 0 && base_dx == 0 && scivolo == 0 && init == 0)
    {
      cont = cont + 0.01;
      if(cont > 4)
      {
        
        move_base(x_wp_i,y_wp_i,z_wp_i,roll_wp_i,pitch_wp_i,yaw_wp_i,x_wp_sx_f,y_wp_sx_f,z_wp_sx_f,roll_wp_sx_f,pitch_wp_sx_f,yaw_wp_sx_f,x_time, y_time, z_time,
        roll_time, pitch_time, yaw_time,contatore,time_f_base);    

        pos_base_time << x_time, y_time, z_time;

        rotation(roll_time, pitch_time, yaw_time, Rot);

        pos_foot_LF = Rot.transpose() * (pos_foot_LF_w - pos_base_time);
      
        Inverse_Kinematics_LF(pos_foot_LF(0),pos_foot_LF(1),pos_foot_LF(2),q1_time_LF,q2_time_LF,q3_time_LF);

        pos_foot_LH = Rot.transpose() *(pos_foot_LH_w - pos_base_time);

        Inverse_Kinematics_LH(pos_foot_LH(0),pos_foot_LH(1),pos_foot_LH(2),q1_time_LH,q2_time_LH,q3_time_LH);

        pos_foot_RF = Rot.transpose() *(pos_foot_RF_w - pos_base_time);

        Inverse_Kinematics_RF(pos_foot_RF(0),pos_foot_RF(1),pos_foot_RF(2),q1_time_RF,q2_time_RF,q3_time_RF);

        pos_foot_RH = Rot.transpose() *(pos_foot_RH_w - pos_base_time);

        Inverse_Kinematics_RH(pos_foot_RH(0),pos_foot_RH(1),pos_foot_RH(2),q1_time_RH,q2_time_RH,q3_time_RH);


            pos_foot <<pos_foot_LF_w, pos_foot_LH_w, pos_foot_RF_w, pos_foot_RH_w;
            vector<double> pos_foot_vec(pos_foot.data(), pos_foot.data() + pos_foot.rows() * pos_foot.cols());
          
          msg.data.clear();
         
          for(int i = 0; i < 12; i++) 
          {
              msg.data.push_back(pos_foot_vec[i]); 
          }
          pos_foot_pub.publish(msg);
  
      
        
        contatore = contatore + 0.01;
  

        if(contatore > time_f_base + 0.01)
        { 
          contatore = 0;
          c_switch = 1;
          init = 1;

          x_wp_sx = x_time; 
          y_wp_sx = y_time; 
          z_wp_sx = z_time; 
          roll_wp_sx = roll_time;
          pitch_wp_sx = pitch_time;
          yaw_wp_sx = yaw_time;
          
          x_wp_dx = x_wp_sx + delta_xx;


          q_i_LF(0) = q1_time_LF;
          q_i_LF(1) = q2_time_LF;
          q_i_LF(2) = q3_time_LF;

          q_i_LH(0) = q1_time_LH;
          q_i_LH(1) = q2_time_LH;
          q_i_LH(2) = q3_time_LH;

          q_i_RF(0) = q1_time_RF;
          q_i_RF(1) = q2_time_RF;
          q_i_RF(2) = q3_time_RF;

          q_i_RH(0) = q1_time_RH;
          q_i_RH(1) = q2_time_RH;
          q_i_RH(2) = q3_time_RH;
    
        
        }
      }
    }

    else if(c_switch == 0 && base_dx == 1 && scivolo == 0 && init == 1)
    {  

      cout<<"primo"<<endl;
     
      move_base(x_wp_sx,y_wp_sx,z_wp_sx,roll_wp_sx,pitch_wp_sx,yaw_wp_sx,x_wp_dx,y_wp_dx,z_wp_dx,roll_wp_dx,pitch_wp_dx,yaw_wp_dx,x_time, y_time, z_time, roll_time, pitch_time, yaw_time,contatore,time_f_base);
      
      pos_base_time << x_time, y_time, z_time;

      rotation(roll_time, pitch_time, yaw_time, Rot);

      pos_foot_LF = Rot.transpose() * (pos_foot_LF_w - pos_base_time);
    
      Inverse_Kinematics_LF(pos_foot_LF(0),pos_foot_LF(1),pos_foot_LF(2),q1_time_LF,q2_time_LF,q3_time_LF);

      pos_foot_LH = Rot.transpose() *(pos_foot_LH_w - pos_base_time);

      Inverse_Kinematics_LH(pos_foot_LH(0),pos_foot_LH(1),pos_foot_LH(2),q1_time_LH,q2_time_LH,q3_time_LH);

      pos_foot_RF = Rot.transpose() *(pos_foot_RF_w - pos_base_time);

      Inverse_Kinematics_RF(pos_foot_RF(0),pos_foot_RF(1),pos_foot_RF(2),q1_time_RF,q2_time_RF,q3_time_RF);

      pos_foot_RH = Rot.transpose() *(pos_foot_RH_w - pos_base_time);

      Inverse_Kinematics_RH(pos_foot_RH(0),pos_foot_RH(1),pos_foot_RH(2),q1_time_RH,q2_time_RH,q3_time_RH);


      contatore = contatore + 0.01;


            pos_foot <<pos_foot_LF_w, pos_foot_LH_w, pos_foot_RF_w, pos_foot_RH_w;
            vector<double> pos_foot_vec(pos_foot.data(), pos_foot.data() + pos_foot.rows() * pos_foot.cols());
          
          msg.data.clear();
         
          for(int i = 0; i < 12; i++) 
          {
              msg.data.push_back(pos_foot_vec[i]); 
          }
          pos_foot_pub.publish(msg);
  


     

      if(contatore > time_f_base + 0.01)
      {
        
        contatore = 0;
        c_switch = 1;

       

        x_wp_dx = x_time; 
        y_wp_dx = y_time; 
        z_wp_dx = z_time; 
        roll_wp_dx = roll_time;
        pitch_wp_dx = pitch_time;
        yaw_wp_dx = yaw_time;
          
        x_wp_sx = x_wp_dx + delta_xx;
        

        q_i_LF(0) = q1_time_LF;
        q_i_LF(1) = q2_time_LF;
        q_i_LF(2) = q3_time_LF;

        q_i_LH(0) = q1_time_LH;
        q_i_LH(1) = q2_time_LH;
        q_i_LH(2) = q3_time_LH;

        q_i_RF(0) = q1_time_RF;
        q_i_RF(1) = q2_time_RF;
        q_i_RF(2) = q3_time_RF;

        q_i_RH(0) = q1_time_RH;
        q_i_RH(1) = q2_time_RH;
        q_i_RH(2) = q3_time_RH;


      }
    }

    else if(c_switch == 0 && base_dx == 0 && init == 1 && scivolo == 0)
    {
       
      cout<<"secondo"<<endl;


      if (cont_noslip > 0)
      {
        scivolo = 1;
        //cout<<endl<<"scivolo"<<endl<<scivolo<<endl;
      }
       

      move_base(x_wp_dx,y_wp_dx,z_wp_dx,roll_wp_dx,pitch_wp_dx,yaw_wp_dx,x_wp_sx,y_wp_sx,z_wp_sx,roll_wp_sx,pitch_wp_sx,yaw_wp_sx,x_time, y_time, z_time, roll_time, pitch_time, yaw_time,contatore,time_f_base);
      
      pos_base_time << x_time, y_time, z_time;

      rotation(roll_time, pitch_time, yaw_time, Rot);

      pos_foot_LF = Rot.transpose() * (pos_foot_LF_w - pos_base_time);
    
      Inverse_Kinematics_LF(pos_foot_LF(0),pos_foot_LF(1),pos_foot_LF(2),q1_time_LF,q2_time_LF,q3_time_LF);

      pos_foot_LH = Rot.transpose() *(pos_foot_LH_w - pos_base_time);

      Inverse_Kinematics_LH(pos_foot_LH(0),pos_foot_LH(1),pos_foot_LH(2),q1_time_LH,q2_time_LH,q3_time_LH);

      pos_foot_RF = Rot.transpose() *(pos_foot_RF_w - pos_base_time);

      Inverse_Kinematics_RF(pos_foot_RF(0),pos_foot_RF(1),pos_foot_RF(2),q1_time_RF,q2_time_RF,q3_time_RF);

      pos_foot_RH = Rot.transpose() *(pos_foot_RH_w - pos_base_time);

      Inverse_Kinematics_RH(pos_foot_RH(0),pos_foot_RH(1),pos_foot_RH(2),q1_time_RH,q2_time_RH,q3_time_RH);


      contatore = contatore + 0.01;


        pos_foot <<pos_foot_LF_w, pos_foot_LH_w, pos_foot_RF_w, pos_foot_RH_w;
        vector<double> pos_foot_vec(pos_foot.data(), pos_foot.data() + pos_foot.rows() * pos_foot.cols());
          
          msg.data.clear();
         
          for(int i = 0; i < 12; i++) 
          {
              msg.data.push_back(pos_foot_vec[i]); 
          }
          pos_foot_pub.publish(msg);
  



      if(contatore > time_f_base + 0.01)
      {
        cont_noslip = cont_noslip+1;
        contatore = 0;
        c_switch = 1;



        x_wp_sx = x_time; 
        y_wp_sx = y_time; 
        z_wp_sx = z_time; 
        roll_wp_sx = roll_time;
        pitch_wp_sx = pitch_time;
        yaw_wp_sx = yaw_time;
      
        x_wp_dx = x_wp_sx + delta_yy;
  

        q_i_LF(0) = q1_time_LF;
        q_i_LF(1) = q2_time_LF;
        q_i_LF(2) = q3_time_LF;

        q_i_LH(0) = q1_time_LH;
        q_i_LH(1) = q2_time_LH;
        q_i_LH(2) = q3_time_LH;

        q_i_RF(0) = q1_time_RF;
        q_i_RF(1) = q2_time_RF;
        q_i_RF(2) = q3_time_RF;

        q_i_RH(0) = q1_time_RH;
        q_i_RH(1) = q2_time_RH;
        q_i_RH(2) = q3_time_RH;

       

      }
    }

    //inizio pianificazione su terreno scivoloso
    if (c_switch == 1 && scivolo == 1)
    {
      
      if(move_leg(1) == 1)
      { 
        
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
         
          if(cont_slip < 3)
            { 
              x_wp_2 = x_wp_1 + delta_x_wp2_LH;  
            }

            else if(cont_slip > 2 && cont_slip < 12)
            {
              x_wp_2 = x_wp_1 + (delta_x_wp2_LH - 0.059);
            }

            else 
            {
              x_wp_2 = x_wp_1 + (delta_x_wp2_LH - 0.051);
            }
  
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

            pos_foot_LH(0) = x_wp_2;
            pos_foot_LH(1) = y_wp_2;
            pos_foot_LH(2) = z_wp_2;
          
           
          }
        }
          
      }

      else if (move_leg(0) == 1 )
      {
        
          
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
      
        else
          {
            
             if(cont_slip > 11)
            { 
              x_wp_2 = x_wp_1 + delta_x_wp2_LF + 0.01;  
            }

            else
            {
              cout<<endl<<"entrato"<<endl;
              x_wp_2 = x_wp_1 + (delta_x_wp2_LF + 0.0);
            }
         
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

              pos_foot_LF(0) = x_wp_2;
              pos_foot_LF(1) = y_wp_2;
              pos_foot_LF(2) = z_wp_2;

             

              pos_base << x_time, y_time, z_time;

              rotation(roll_time, pitch_time, yaw_time, Rot);

              pos_foot_LF_w = pos_base + Rot*pos_foot_LF;
                  
              pos_foot_LH_w = pos_base + Rot*pos_foot_LH;
                  
              pos_foot_RF_w = pos_base + Rot*pos_foot_RF;

              pos_foot_RH_w = pos_base + Rot*pos_foot_RH;

             
          
              
          }
        }
      }
      
      

      else if(move_leg(3) == 1)
      {
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
          
          if (cont_slip < 3)
          { 
            x_wp_2 = x_wp_1 + delta_x_wp2_RH;
            
          }

          else if (cont_slip > 2 && cont_slip < 12)
          {
            x_wp_2 = x_wp_1 + (delta_x_wp2_RH - 0.0709);
          }
         else
         {
           x_wp_2 = x_wp_1 + (delta_x_wp2_RH - 0.063);
         }
          
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

            pos_foot_RH(0) = x_wp_2;
            pos_foot_RH(1) = y_wp_2;
            pos_foot_RH(2) = z_wp_2;
           
            
          }
        }
      }

      else if(move_leg(2) == 1)
      {
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
            
            if(cont_slip < 3)
            { 
              x_wp_2 = x_wp_1 + delta_x_wp2_RF;  
            }
             else
            {
              cout<<endl<<"entrato"<<endl;
              x_wp_2 = x_wp_1 + (delta_x_wp2_RF + 0.023);
            }

             if(cont_slip > 11)
            { 
              cout<<endl<<"nooooooo"<<endl;
              x_wp_2 = x_wp_1 + delta_x_wp2_RF + 0.03;  
            }

            else
            {
              cout<<endl<<"entrato"<<endl;
              x_wp_2 = x_wp_1 + (delta_x_wp2_RF + 0.025);
            }

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
           

            pos_foot_RF(0) = x_wp_2;
            pos_foot_RF(1) = y_wp_2;
            pos_foot_RF(2) = z_wp_2;


            pos_base << x_time, y_time, z_time;

            rotation(roll_time, pitch_time, yaw_time, Rot);

            pos_foot_LF_w = pos_base + Rot*pos_foot_LF;
                
            pos_foot_LH_w = pos_base + Rot*pos_foot_LH;
                
            pos_foot_RF_w = pos_base + Rot*pos_foot_RF;

            pos_foot_RH_w = pos_base + Rot*pos_foot_RH;
          
          }
        }
      }

    }

    else if(c_switch == 0 && base_dx == 1 && scivolo == 1)
    {
       cout<<"terzo"<<endl;
      move_base(x_wp_sx,y_wp_sx,z_wp_sx,roll_wp_sx,pitch_wp_sx,yaw_wp_sx,x_wp_dx,y_wp_dx,z_wp_dx,roll_wp_dx,pitch_wp_dx,yaw_wp_dx,x_time, y_time, z_time, roll_time, pitch_time, yaw_time,contatore,time_f_base);
      
      pos_base_time << x_time, y_time, z_time;

      rotation(roll_time, pitch_time, yaw_time, Rot);

      pos_foot_LF = Rot.transpose() * (pos_foot_LF_w - pos_base_time);
    
      Inverse_Kinematics_LF(pos_foot_LF(0),pos_foot_LF(1),pos_foot_LF(2),q1_time_LF,q2_time_LF,q3_time_LF);

      pos_foot_LH = Rot.transpose() *(pos_foot_LH_w - pos_base_time);

      Inverse_Kinematics_LH(pos_foot_LH(0),pos_foot_LH(1),pos_foot_LH(2),q1_time_LH,q2_time_LH,q3_time_LH);

      pos_foot_RF = Rot.transpose() *(pos_foot_RF_w - pos_base_time);

      Inverse_Kinematics_RF(pos_foot_RF(0),pos_foot_RF(1),pos_foot_RF(2),q1_time_RF,q2_time_RF,q3_time_RF);

      pos_foot_RH = Rot.transpose() *(pos_foot_RH_w - pos_base_time);

      Inverse_Kinematics_RH(pos_foot_RH(0),pos_foot_RH(1),pos_foot_RH(2),q1_time_RH,q2_time_RH,q3_time_RH);
     
      contatore = contatore + 0.01;
        pos_foot <<pos_foot_LF_w, pos_foot_LH_w, pos_foot_RF_w, pos_foot_RH_w;
        vector<double> pos_foot_vec(pos_foot.data(), pos_foot.data() + pos_foot.rows() * pos_foot.cols());
          
          msg.data.clear();
         
          for(int i = 0; i < 12; i++) 
          {
              msg.data.push_back(pos_foot_vec[i]); 
          }
          pos_foot_pub.publish(msg);



      if(contatore > time_f_base + 0.01)
      {
        contatore = 0;
        c_switch = 1;

        x_wp_dx = x_time; 
        y_wp_dx = y_time; 
        z_wp_dx = z_time; 
        roll_wp_dx = roll_time;
        pitch_wp_dx = pitch_time;
        yaw_wp_dx = yaw_time;

      
        x_wp_sx = x_wp_dx + delta_xx_corr;
           
       
        q_i_LF(0) = q1_time_LF;
        q_i_LF(1) = q2_time_LF;
        q_i_LF(2) = q3_time_LF;

        q_i_LH(0) = q1_time_LH;
        q_i_LH(1) = q2_time_LH;
        q_i_LH(2) = q3_time_LH;

        q_i_RF(0) = q1_time_RF;
        q_i_RF(1) = q2_time_RF;
        q_i_RF(2) = q3_time_RF;

        q_i_RH(0) = q1_time_RH;
        q_i_RH(1) = q2_time_RH;
        q_i_RH(2) = q3_time_RH;

      }
    
       }

    else if(c_switch == 0 && base_dx == 0 && scivolo == 1)
    {
      cout<<"quarto"<<endl;     
      move_base(x_wp_dx,y_wp_dx,z_wp_dx,roll_wp_dx,pitch_wp_dx,yaw_wp_dx,x_wp_sx,y_wp_sx,z_wp_sx,roll_wp_sx,pitch_wp_sx,yaw_wp_sx,x_time, y_time, z_time, roll_time, pitch_time, yaw_time,contatore,time_f_base);
      
      pos_base_time << x_time, y_time, z_time;

      rotation(roll_time, pitch_time, yaw_time, Rot);

      pos_foot_LF = Rot.transpose() * (pos_foot_LF_w - pos_base_time);
    
      Inverse_Kinematics_LF(pos_foot_LF(0),pos_foot_LF(1),pos_foot_LF(2),q1_time_LF,q2_time_LF,q3_time_LF);

      pos_foot_LH = Rot.transpose() *(pos_foot_LH_w - pos_base_time);

      Inverse_Kinematics_LH(pos_foot_LH(0),pos_foot_LH(1),pos_foot_LH(2),q1_time_LH,q2_time_LH,q3_time_LH);

      pos_foot_RF = Rot.transpose() *(pos_foot_RF_w - pos_base_time);

      Inverse_Kinematics_RF(pos_foot_RF(0),pos_foot_RF(1),pos_foot_RF(2),q1_time_RF,q2_time_RF,q3_time_RF);

      pos_foot_RH = Rot.transpose() *(pos_foot_RH_w - pos_base_time);

      Inverse_Kinematics_RH(pos_foot_RH(0),pos_foot_RH(1),pos_foot_RH(2),q1_time_RH,q2_time_RH,q3_time_RH);
      contatore = contatore + 0.01;

        pos_foot <<pos_foot_LF_w, pos_foot_LH_w, pos_foot_RF_w, pos_foot_RH_w;
        vector<double> pos_foot_vec(pos_foot.data(), pos_foot.data() + pos_foot.rows() * pos_foot.cols());
          
          msg.data.clear();
         
          for(int i = 0; i < 12; i++) 
          {
              msg.data.push_back(pos_foot_vec[i]); 
          }
          pos_foot_pub.publish(msg);
              

      if(contatore > time_f_base + 0.01)
      {
         cont_slip = cont_slip+1;
        cout<<endl<<"cont_slip"<<endl<<cont_slip<<endl;
        

        contatore = 0;
        c_switch = 1;
        
        x_wp_sx = x_time; 
        y_wp_sx = y_time; 
        z_wp_sx = z_time; 
        roll_wp_sx = roll_time;
        pitch_wp_sx = pitch_time;
        yaw_wp_sx = yaw_time;

      
          
        x_wp_dx = x_wp_sx + delta_yy_corr;
            

      
        q_i_LF(0) = q1_time_LF;
        q_i_LF(1) = q2_time_LF;
        q_i_LF(2) = q3_time_LF;

        q_i_LH(0) = q1_time_LH;
        q_i_LH(1) = q2_time_LH;
        q_i_LH(2) = q3_time_LH;

        q_i_RF(0) = q1_time_RF;
        q_i_RF(1) = q2_time_RF;
        q_i_RF(2) = q3_time_RF;

        q_i_RH(0) = q1_time_RH;
        q_i_RH(1) = q2_time_RH;
        q_i_RH(2) = q3_time_RH;

      
      }  
    }

      
    //todo
    //Pianificazione camminata finale in cui il robot esce dal terreno
    //e viene reimpostata una camminata normale ("a x") con tempi
    //di movimento di gambe e base reimpostati alla vecolità
    //normale

    joint_state.header.stamp = ros::Time::now();
    joint_state.header.stamp.toSec();
    joint_state.header.stamp.toNSec();
    joint_state.name.resize(18);
    joint_state.position.resize(18);
    joint_state.velocity.resize(18);
    joint_state.effort.resize(18);



    joint_state.name[0] = "trans_x";
    joint_state.position[0] = x_time;
    joint_state.velocity[0] = 0.0;
    joint_state.effort[0] = 0.0;
    
    joint_state.name[1] = "trans_y";
    joint_state.position[1] = y_time;
    joint_state.velocity[1] = 0.0;
    joint_state.effort[1] = 0.0;
    
    joint_state.name[2] = "trans_z";
    joint_state.position[2] = z_time;
    joint_state.velocity[2] = 0.0;
    joint_state.effort[2] = 0.0;
    
    joint_state.name[3] = "rot_x";
    joint_state.position[3] = roll_time;
    joint_state.velocity[3] = 0.0;
    joint_state.effort[3] =  0.0;
    
    joint_state.name[4] = "rot_y";
    joint_state.position[4] = pitch_time;
    joint_state.velocity[4] = 0.0;
    joint_state.effort[4] = 0.0;
    
    joint_state.name[5] = "rot_z";
    joint_state.position[5] = yaw_time;
    joint_state.velocity[5] = 0.0;
    joint_state.effort[5] = 0.0;

    joint_state.name[6]="LF_HAA";
    joint_state.position[6] = q1_time_LF; 
    joint_state.velocity[6] = 0.0;
    joint_state.effort[6] = 0.0;

    joint_state.name[7]="LF_HFE";
    joint_state.position[7] = q2_time_LF; 
    joint_state.velocity[7] = 0.0;
    joint_state.effort[7] = 0.0;
    
    joint_state.name[8]="LF_KFE";
    joint_state.position[8] = q3_time_LF;
    joint_state.velocity[8] = 0.0;
    joint_state.effort[8] = 0.0;

    joint_state.name[9]="LH_HAA";
    joint_state.position[9] = q1_time_LH; 
    joint_state.velocity[9] = 0.0;
    joint_state.effort[9] = 0.0;
    
    joint_state.name[10]="LH_HFE";
    joint_state.position[10] = q2_time_LH; 
    joint_state.velocity[10] = 0.0;
    joint_state.effort[10] = 0.0;
    
    joint_state.name[11]="LH_KFE";
    joint_state.position[11] = q3_time_LH; 
    joint_state.velocity[11] = 0.0;
    joint_state.effort[11] = 0.0;
    
    joint_state.name[12]="RF_HAA";
    joint_state.position[12] = q1_time_RF; 
    joint_state.velocity[12] = 0.0;
    joint_state.effort[12] = 0.0;
    
    joint_state.name[13]="RF_HFE";
    joint_state.position[13] = q2_time_RF; 
    joint_state.velocity[13] = 0.0;
    joint_state.effort[13] = 0.0;
    
    joint_state.name[14]="RF_KFE";
    joint_state.position[14] = q3_time_RF;
    joint_state.velocity[14] = 0.0; 
    joint_state.effort[14] = 0.0;
    
    joint_state.name[15]="RH_HAA";
    joint_state.position[15] = q1_time_RH; 
    joint_state.velocity[15] = 0.0;
    joint_state.effort[15] = 0.0;
    
    joint_state.name[16]="RH_HFE";
    joint_state.position[16] = q2_time_RH; 
    joint_state.velocity[16] = 0.0;
    joint_state.effort[16] = 0.0;
    
    joint_state.name[17]="RH_KFE";
    joint_state.position[17] =  q3_time_RH; 
    joint_state.velocity[17] = 0.0;
    joint_state.effort[17] = 0.0;


    joint_pub.publish(joint_state);


  
    //cout<<"time"<<endl<< t <<endl;
    //cout<<"scivolo"<<endl<< scivolo <<endl;
    //cout<<"cont_noslip"<<endl<< cont_noslip <<endl;
    cout<<"cont_slip"<<endl<< cont_slip <<endl;
    

    //cout<<"correzione"<<endl<<correzione<<endl;
    

    ros::spinOnce();
    loop_rate.sleep();

  
  }

  return 0;
}


