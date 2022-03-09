#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
//#include "OsqpEigen/OsqpEigen.h"

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h> 
#include <hardware_interface/joint_state_interface.h> 
 
#include <ros/console.h>
#include <ros/ros.h> 
#include <ros/node_handle.h>
#include <ros/time.h>


#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/Constraints.h>

#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/LinkState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include <sensor_msgs/JointState.h>
#include "tf/transform_datatypes.h"
#include <stdio.h>     
#include <stdlib.h>   



using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;


namespace my_controller_ns
{
    

class floating_base_controller: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

    public:

    Model* model; 
    Model* model2;

    
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
    
    void starting(const ros::Time&);
    
    void stopping(const ros::Time&);

    
    
    void update(const ros::Time&, const ros::Duration& period);

    void state_estimator(const gazebo_msgs::LinkStatesConstPtr& msg);

    void setCommandCB(const sensor_msgs::JointStateConstPtr& msg);

    void setCommandCB_base(const sensor_msgs::JointStateConstPtr& msg);

    void c_switch_setCommand(const std_msgs::BoolConstPtr& msg);

    void fine_setCommand(const std_msgs::BoolConstPtr& msg);

    void endd_setCommand(const std_msgs::BoolConstPtr& msg);

    void scivolo_setCommand(const std_msgs::BoolConstPtr& msg);

    void scivolo_dietro_setCommand(const std_msgs::BoolConstPtr& msg);

    void scivolo_due_setCommand(const std_msgs::BoolConstPtr& msg);

    void scivolo_dietro_due_setCommand(const std_msgs::BoolConstPtr& msg);

    void conta_friction_setCommand(const std_msgs::Float64ConstPtr& msg);

    void delta_x_avanti_setCommand(const std_msgs::Float64ConstPtr& msg);

    private:

    Eigen::MatrixXd M_; 
    Eigen::MatrixXd M_inv;
    Eigen::VectorXd c_; 
    Eigen::VectorXd cor_; 
    Eigen::VectorXd g_; 
    Eigen::MatrixXd S_;
    Eigen::MatrixXd S_trans;
    Eigen::MatrixXd Jc_;
    Eigen::MatrixXd Fc;
    Eigen::MatrixXd p;
    Eigen::MatrixXd p_inv;

    
    Eigen::MatrixXd S_ker;
    Eigen::MatrixXd S_ker_trans;
    Eigen::MatrixXd S_ker_prec;
    Eigen::MatrixXd S_ker_dot;

    Eigen::MatrixXd Jc_prec;
    Eigen::MatrixXd Jc_dot;
    
    Eigen::MatrixXd J_c1;
    Eigen::MatrixXd J_c2;
    Eigen::MatrixXd J_c3;
    Eigen::MatrixXd J_c4;
    
    VectorNd QDot;
    VectorNd index_link;

    double mu = 0.75;
    double mu_e = mu/sqrt(2);
    double Ft_LFx, Ft_LFy, Ft_LHx, Ft_LHy, Ft_RFx, Ft_RFy, Ft_RHx, Ft_RHy;
    double Fc_LF_T, Fc_LH_T, Fc_RF_T, Fc_RH_T, Fc_LF_prova, Fc_LH_prova, Fc_RF_prova, Fc_RH_prova, Fc_LF3, Fc_RF3, Fc_LH3, Fc_RH3;

    double Fc_LF_Tx, Fc_LF_Ty, Fc_LH_Tx, Fc_LH_Ty, Fc_RF_Tx, Fc_RF_Ty, Fc_RH_Tx, Fc_RH_Ty, Fc_LFx_prova,Fc_LFy_prova, Fc_LHx_prova, Fc_LHy_prova, Fc_RFx_prova, Fc_RFy_prova,Fc_RHx_prova, Fc_RHy_prova;
     
    
    double max_LF = 28;
    double max_LH = 34;
    double max_RF = 29.5;
    double max_RH = 24.5;
     
    double contocono = 0;
    double contovel = 0;
    double value = 0;
 
    double pos_foot_LF_media;
        double pos_foot_LH_media;
            double pos_foot_RF_media;
                double pos_foot_RH_media;
                 double diff1;
                 double diff2;
                 double pos_LF_media;

    Eigen::Matrix<double, 3, 1> v1;
/*
    double foot_LF;
    double foot_LH;
    double foot_RF;
    double foot_RH;

    double foot_LF_prec;
    double foot_LH_prec;
    double foot_RF_prec;
    double foot_RH_prec;

    double pos_foot_LF_dot;
    double pos_foot_LH_dot;
    double pos_foot_RF_dot;
    double pos_foot_RH_dot;
*/
    double delta_vel_LF;
    double delta_vel_LH;
    double delta_vel_RF;
    double delta_vel_RH;

    double tau_LF;
    double tau_LH;
    double tau_RF;
    double tau_RH;

    bool friction_cone = 0; //=1, fuoru dal cono/recovery
    bool friction_cone_dietro = 0;
    bool friction_dietro = 0;
    bool friction_dietro_due = 0;
    bool scivolo = 0;
    bool scivolo_due = 0;
    bool scivolo_dietro = 0;
    bool scivolo_dietro_due = 0;

    double lungh_L = 0;
    double lungh_R = 0; 
    double conta_friction = 0;


    std_msgs::Bool friction_cone_msg;
    std_msgs::Bool friction_cone_dietro_msg;
    std_msgs::Bool friction_dietro_msg;
    std_msgs::Bool friction_dietro_due_msg;
    std_msgs::Bool freeze_msg;
    std_msgs::Bool scivolo_msg;
    std_msgs::Bool scivolo_due_msg;
    std_msgs::Bool scivolo_dietro_msg;
    std_msgs::Bool scivolo_dietro_due_msg;
    std_msgs::Float64 delta_x_avanti_msg;
    std_msgs::Float64 conta_friction_msg;



    bool a = 0;
    bool b = 0;
    bool c = 0;
    bool d = 0;
    bool e = 0;
    bool f = 0;
    bool endd = 0;
    bool freeze = 0;

    double conta = 0;


    Eigen::MatrixXd Jc_trans;
    Eigen::MatrixXd J_c_scomp_1;
    Eigen::MatrixXd J_c_scomp_2;


    Eigen::MatrixXd X_scomp;
    Eigen::MatrixXd id;
    Eigen::MatrixXd J_c_scomp_2_inv;

    Eigen::MatrixXd M_new;
    Eigen::MatrixXd C_new;
    Eigen::MatrixXd g_new;
    Eigen::MatrixXd vel_des;

    Eigen::MatrixXd pos_foot_LF_dot;
    Eigen::MatrixXd pos_foot_LH_dot;
    Eigen::MatrixXd pos_foot_RF_dot;
    Eigen::MatrixXd pos_foot_RH_dot;

    Eigen::MatrixXd foot_LF_prec;
    Eigen::MatrixXd foot_LH_prec;
    Eigen::MatrixXd foot_RF_prec;
    Eigen::MatrixXd foot_RH_prec;

    double x_m = 0.050;
    double y_m = 0.075;
    double z_m = 0.120;



    Eigen::MatrixXd S_S;
    Eigen::MatrixXd S_S_inv;

    Math::Vector3d pos_foot_LF;
    Math::Vector3d pos_foot_LH;
    Math::Vector3d pos_foot_RF;
    Math::Vector3d pos_foot_RH;

     /* Gain */
    double kp1, kp2, kp3, kv1, kv2, kv3;
    double roll, pitch, yaw;
    double kp1_6, kv1_6, kp2_6, kv2_6;


    int df = 500;
    int df1 = 1;

    double dt = 0.002;
    double delta_pos_x_media;
    double delta_pos_y_media;
    double delta_pos_z_media;
    double vel_LF_media;
    double vel_LH_media;
    double vel_RF_media;
    double vel_RH_media;

    double Fc_LF;
    double Fc_LH;
    double Fc_RF;
    double Fc_RH;

    int count = 1;

    bool flag = 1;

    double conto = 0;

    double contatore = 0;

    bool c_switch = 0; // 0 for quasi-vel, 1 PD

    double t;

    double media;

    
    
    /*q_current, dot_q_current, and tau_cmd definitions*/
    
    Eigen::Matrix<double, 18, 1> q_curr;
    Eigen::Matrix<double, 18, 1> dot_q_curr;
    Eigen::Matrix<double, 18, 1> dot_dot_q_curr;
    Eigen::Matrix<double, 12, 1> tau_cmd;

    Eigen::Matrix<double, 12, 1> command_q_des;
    Eigen::Matrix<double, 12, 1> command_dot_q_des;
    Eigen::Matrix<double, 12, 1> command_dot_dot_q_des;

    Eigen::Matrix<double, 6, 1> q_base;
    Eigen::Matrix<double, 12, 1> q_init_leg;
    Eigen::Matrix<double, 12, 1> q_des_init;
    Eigen::Matrix<double, 12, 1> q_dot_des_init;

    Eigen::Matrix<double, 12, 1> q_des;
    Eigen::Matrix<double, 12, 1> q_dot_des;
    Eigen::Matrix<double, 12, 1> q_dot_dot_des;

    
    Eigen::Matrix<double, 6, 1> ni_int_des;
    Eigen::Matrix<double, 6, 1> ni_des; 
    Eigen::Matrix<double, 6, 1> ni_dot_des;
    Eigen::Matrix<double, 6, 1> ni_curr;
    Eigen::Matrix<double, 6, 1> ni_int_curr;
    Eigen::Matrix<double, 6, 1> ni_dot_curr;

    Eigen::Matrix<double, 6, 1> base_des;
    Eigen::Matrix<double, 6, 1> base_dot_des;
    Eigen::Matrix<double, 6, 1> base_dot_dot_des;
     
   
    Eigen::Matrix<double, 7, 1> q_temp;
    Eigen::Matrix<double, 3, 1> rpy_temp;
    Eigen::Matrix<double, 6, 1> dot_q_temp;
    Eigen::Matrix<double, 6, 1> dot_dot_q_temp;

    Eigen::Matrix<double, 6, 1> err_6;
    Eigen::Matrix<double, 6, 1> dot_err_6;
    Eigen::Matrix<double, 6, 1> dot_dot_err_6;

    
    /*  error and dot error vectors */
    Eigen::Matrix<double, 12, 1> err_12;
    Eigen::Matrix<double, 12, 1> dot_err_12;
    Eigen::Matrix<double, 12, 1> dot_dot_err_12;

    Eigen::MatrixXd u;
    Eigen::MatrixXd Fc_trans;
    
    
    Eigen::Matrix<double, 6, 6> Kp_6;
    Eigen::Matrix<double, 6, 6> Kv_6;
    

    Eigen::Matrix<double, 12, 12> Kp_12;
    Eigen::Matrix<double, 12, 12> Kv_12;


    double foot_LF;
    double foot_LH;
    double foot_RF;
    double foot_RH;

    bool fine;

    double delta_x_avanti;

    double vel_media;


    /* ROS variables */
    ros::NodeHandle n;
    ros::Subscriber sub_command_;
    ros::Subscriber sub_command_base;
    ros::Subscriber sub_gaze;

    ros::Publisher error_pos;
   
    ros::Publisher force_L;
    ros::Publisher force_R;

    ros::Publisher tau_1_6;
    ros::Publisher tau_7_12;

    ros::Publisher sing_values;

    ros::Publisher pos_foot_L_pub;
    ros::Publisher pos_foot_R_pub;
    ros::Publisher q_i_L_pub;
    ros::Publisher q_i_R_pub;
    ros::Publisher vel_L_pub;
    ros::Publisher vel_R_pub;
    ros::Publisher pos_base_pub;
    ros::Publisher vel_foot_pub;
    ros::Publisher friction_cone_pub;
    ros::Publisher friction_cone_dietro_pub;
    ros::Publisher friction_dietro_pub;
    ros::Publisher friction_dietro_due_pub;
    ros::Publisher freeze_pub;
    ros::Publisher scivolo_pub;
    ros::Publisher scivolo_due_pub;
    ros::Publisher scivolo_dietro_pub;
    ros::Publisher scivolo_dietro_due_pub;
    ros::Publisher delta_x_avanti_pub;
    ros::Publisher conta_friction_pub;
   

    ros::Subscriber c_switch_sub;
    ros::Subscriber fine_sub;
    ros::Subscriber end_sub;
    ros::Subscriber scivolo_sub;
    ros::Subscriber scivolo_due_sub;
    ros::Subscriber scivolo_dietro_sub;
    ros::Subscriber scivolo_dietro_due_sub;
    ros::Subscriber conta_friction_sub;
    ros::Subscriber delta_x_avanti_sub;
    
    
    std::vector<hardware_interface::JointHandle> joint_handle;

};

}
