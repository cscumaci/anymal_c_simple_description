#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
//#include <OsqpEigen/OsqpEigen.h>

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

    private:

    Eigen::MatrixXd M_; 
    Eigen::MatrixXd M_inv;
    Eigen::VectorXd c_; 
    Eigen::VectorXd cor_; 
    Eigen::VectorXd g_; 
    Eigen::MatrixXd S_;
    Eigen::MatrixXd S_trans;
    Eigen::MatrixXd S_trans_inv;
    Eigen::MatrixXd Jc_;
    Eigen::MatrixXd Jc_inv;
    Eigen::MatrixXd Fc;
    Eigen::MatrixXd Fc_mu;
    Eigen::MatrixXd p;
    Eigen::MatrixXd p_inv;
    Eigen::MatrixXd tau_3LF;

    
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

    Eigen::Matrix<double, 3, 1> v1;


    Eigen::MatrixXd Jc_trans;
    Eigen::MatrixXd J_c_scomp_1;
    Eigen::MatrixXd J_c_scomp_2;


    Eigen::MatrixXd X_scomp;
    Eigen::MatrixXd id;
    Eigen::MatrixXd J_c_scomp_2_inv;

    Eigen::MatrixXd M_new;
    Eigen::MatrixXd C_new;
    Eigen::MatrixXd g_new;

    Eigen::MatrixXd S_S;
    Eigen::MatrixXd S_S_inv;

    Math::Vector3d pos_foot_LF;
    Math::Vector3d pos_foot_LH;
    Math::Vector3d pos_foot_RF;
    Math::Vector3d pos_foot_RH;

    /* Gain */
    double kp1_6, kp2_6, kp2, kp3, kv1_6, kv2_6, kv2, kv3;
    
    double roll, pitch, yaw;
    
    double theta = -0.26;

    double mu = 1;
    
    double g_new_x, g_new_z;
    
    bool slope = 0; // 1 piano inclinato
    
    double z_avg_L, z_avg_R, z_avg; 
    
    double mass= 6.227421;
    double g = 9.81;

    int df = 500;

    double dt = 0.002;

    int count = 1;

    bool flag = 1;

    double conto = 0;

    double contatore = 0;

    bool c_switch = 0; // 0 for quasi-vel, 1 PD

    double t;

    std_msgs::Bool slope_msg;
    
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
    
    Eigen::Matrix<double, 6, 6> Kp_6;
    Eigen::Matrix<double, 6, 6> Kv_6;

    Eigen::Matrix<double, 12, 12> Kp_12;
    Eigen::Matrix<double, 12, 12> Kv_12;


    /* ROS variables */
    ros::NodeHandle n;
    ros::Subscriber sub_command_;
    ros::Subscriber sub_command_base;
    ros::Subscriber sub_gaze;

    ros::Publisher error_pos;
    ros::Publisher error_vel;
    ros::Publisher error_acc;
   
    ros::Publisher force_L;
    ros::Publisher force_R;

    ros::Publisher tau_1_6;
    ros::Publisher tau_7_12;

    ros::Publisher sing_values;

    ros::Publisher pos_foot_L_pub;
    ros::Publisher pos_foot_R_pub;
    ros::Publisher q_i_L_pub;
    ros::Publisher q_i_R_pub;
    ros::Publisher pos_base_pub;
    ros::Publisher slope_pub;
   
    ros::Subscriber c_switch_sub;
    
    
    std::vector<hardware_interface::JointHandle> joint_handle;

};

}
