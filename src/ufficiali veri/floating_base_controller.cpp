
// Quasi velocities combined with pd controller

#include <pluginlib/class_list_macros.h>
#include "floating_base_controller.h" 
#include "pseudo_inversion.h"

#include "inverse_kinematics.h"

namespace my_controller_ns{

bool floating_base_controller::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{ 
    this->n = n;


    //Creation of the model
    model = new Model(); 
    
    //Creation of a second model that allows the right computation of the Jacobian matrix
    model2 = new Model();

    //Check for the right loading of the first model
    if (!Addons::URDFReadFromFile("/home/cris/catkin_ws/src/anymal_c_simple_description/urdf/model.urdf", model, true, false))
    {
        cout << "Error loading model" << endl;
        abort(); 
    }
   
    //Check for the right loading of the second model
    if (!Addons::URDFReadFromFile("/home/cris/catkin_ws/src/anymal_c_simple_description/urdf/model2.urdf", model2, true, false))
    {
        cout << "Error loading model2" << endl;
        abort();
    }
 
    cout <<Utils::GetModelDOFOverview(*model)<< endl;

    cout << model->dof_count << endl; 
   
    //Inizialization of the matrices

    S_ = Eigen::MatrixXd::Zero(12,18); 

    S_ker_prec = Eigen::MatrixXd::Zero(18, 6);
    S_ker_dot = Eigen::MatrixXd::Zero(18, 6);
    Jc_prec = Eigen::MatrixXd::Zero(12, 18);
    Jc_dot = Eigen::MatrixXd::Zero(12, 18); 

   


    err_6 = Eigen::VectorXd::Zero(6);
    dot_err_6 = Eigen::VectorXd::Zero(6);

    err_12 = Eigen::VectorXd::Zero(12);
    dot_err_12 = Eigen::VectorXd::Zero(12);


    //Computation of the Selection matrix of the actuated joints
    int k = 6;
    for (size_t i = 0; i<12; i++)
    {
        S_(i,k) = 1;
        k++;
    }
    
    //Check for the right numbers of the actuated joints
    std::vector<std::string> joint_names;
    if (!n.getParam("joint_names", joint_names) || joint_names.size() != 12) 
    {
        ROS_ERROR("Error in parsing joints name!");
        return false;
    }

    //Creation and filling of the angular positions and angular velocities vectors of the actuated joints
    for (size_t i = 0; i<12; i++)
    {   
        joint_handle.push_back(hw->getHandle(joint_names[i])); 
        q_curr(i+6) = joint_handle[i].getPosition();
        dot_q_curr(i+6) = joint_handle[i].getVelocity();
    }


    //Subscriber that subscribes a topic that gives the desired positions and velocities vectors of the joints
    this->sub_command_ = n.subscribe<sensor_msgs::JointState> ("command", 1, &floating_base_controller::setCommandCB, this); 

    this->sub_command_base = n.subscribe<sensor_msgs::JointState> ("command_base", 1, &floating_base_controller::setCommandCB_base, this); 
    
    //Subscriber that subscribes a topic from Gazebo that gives the pose of the unactuated joints
    this-> sub_gaze = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, &floating_base_controller::state_estimator, this);

    this-> error_pos = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/error_pos", 1);  
    this-> force_L = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/force_L", 1); 
    this-> force_R = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/force_R", 1); 
    //this-> sing_values = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/sing_values", 1);
    this-> tau_1_6 = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/tau_1_6", 1);
    this-> tau_7_12 = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/tau_7_12", 1);

    this-> pos_foot_L_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/pos_foot_L_pub", 1);
    this-> pos_foot_R_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/pos_foot_R_pub", 1);

    this-> vel_L_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/vel_L_pub", 1);
    this-> vel_R_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/vel_R_pub", 1);

    this-> q_i_L_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/q_i_L_pub", 1); 
    this-> q_i_R_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/q_i_R_pub", 1);

    this-> pos_base_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/pos_base_pub", 1);

    this-> vel_foot_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/vel_foot_pub", 1);

    this-> friction_cone_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/friction_cone_pub", 1);
    this-> friction_cone_dietro_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/friction_cone_dietro_pub", 1);
    this-> friction_dietro_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/friction_dietro_pub", 1);
    this-> friction_dietro_due_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/friction_dietro_due_pub", 1);
    this-> freeze_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/freeze_pub", 1);
    this-> scivolo_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/scivolo_pub", 1);
    this-> scivolo_due_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/scivolo_due_pub", 1);
    this-> scivolo_dietro_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/scivolo_dietro_pub", 1);
    this-> scivolo_dietro_due_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/scivolo_dietro_due_pub", 1);


    this-> c_switch_sub = n.subscribe<std_msgs::Bool>("get_c_switch_set_command", 1, &floating_base_controller::c_switch_setCommand, this);
    this-> fine_sub = n.subscribe<std_msgs::Bool>("get_fine_set_command", 1, &floating_base_controller::fine_setCommand, this);
    this-> end_sub = n.subscribe<std_msgs::Bool>("get_end_set_command", 1, &floating_base_controller::endd_setCommand, this);
    this-> scivolo_sub = n.subscribe<std_msgs::Bool>("scivolo_set_command", 1, &floating_base_controller::scivolo_setCommand, this);
    this-> scivolo_due_sub = n.subscribe<std_msgs::Bool>("scivolo_due_set_command", 1, &floating_base_controller::scivolo_due_setCommand, this);
    this-> scivolo_dietro_sub = n.subscribe<std_msgs::Bool>("scivolo_dietro_set_command", 1, &floating_base_controller::scivolo_dietro_setCommand, this);
    this-> scivolo_dietro_due_sub = n.subscribe<std_msgs::Bool>("scivolo_dietro_due_set_command", 1, &floating_base_controller::scivolo_dietro_due_setCommand, this);
   
   
   

    // Initial configuration

    //base desired accelerations
    ni_dot_des(0) = 0.0;
    ni_dot_des(1) = 0.0;
    ni_dot_des(2) = 0.0;
    ni_dot_des(3) = 0.0;
    ni_dot_des(4) = 0.0;
    ni_dot_des(5) = 0.0;

    //base desired velocities
    ni_des(0) = 0.0;
    ni_des(1) = 0.0;
    ni_des(2) = 0.0;
    ni_des(3) = 0.0;
    ni_des(4) = 0.0;
    ni_des(5) = 0.0;


    //base desired positions
    ni_int_des(0) = 0.03;
    ni_int_des(1) = -0.09; 
    ni_int_des(2) = 0.54;
    ni_int_des(3) = 0.0;
    ni_int_des(4) = -0.015;
    ni_int_des(5) = 0.0; 

   
    return 1;
}



void floating_base_controller::starting(const ros::Time& time){}


void floating_base_controller::stopping(const ros::Time& time){}


void floating_base_controller::update(const ros::Time& time, const ros::Duration& period)
    {   
        //Inizialization and definitions of matrices and vectors
        M_ =  Eigen::MatrixXd::Zero(18, 18);
        M_inv =  Eigen::MatrixXd::Zero(18, 18);
        c_ = Eigen::VectorXd::Zero(18, 1); 
        g_ = Eigen::VectorXd::Zero(18, 1);
        cor_ = Eigen::VectorXd::Zero(18);

        J_c1 = Eigen::MatrixXd::Zero(3, 18);
        J_c2 = Eigen::MatrixXd::Zero(3, 18);
        J_c3 = Eigen::MatrixXd::Zero(3, 18);
        J_c4 = Eigen::MatrixXd::Zero(3, 18);

        S_trans = Eigen::MatrixXd::Zero(18,12);

        tau_cmd =  Eigen::VectorXd::Zero(12);

        pos_foot_LF_dot =  Eigen::MatrixXd::Zero(3,1);
        pos_foot_LH_dot =  Eigen::MatrixXd::Zero(3,1);
        pos_foot_RF_dot =  Eigen::MatrixXd::Zero(3,1);
        pos_foot_RH_dot =  Eigen::MatrixXd::Zero(3,1);
    

        t = ros::Time::now().toSec(); 

        //Initialization of joints configurations vectors
        for (size_t i = 0; i < 18; i++)
        {
            q_curr(i) = 0;
            dot_q_curr(i) = 0;
            dot_dot_q_curr(i) = 0;
        }

      //Transformation from a quaternion parametrization to Euler angles(R,P,Y) of the unactuated joints
       tf::Quaternion q(q_temp[3], q_temp[4], q_temp[5], q_temp[6]);
       tf::Matrix3x3 m(q);
       m.getRPY(roll,pitch,yaw);
       rpy_temp << roll, pitch, yaw;

       //Update of current base positions 
        for (size_t i = 0; i<3; i++)
        {
            q_curr(i) = q_temp(i);
            ni_int_curr(i) = q_temp(i);
        }

        for (size_t i = 0; i<3; i++)
        {
            q_curr(i+3)= rpy_temp(i);
            ni_int_curr(i+3) = rpy_temp(i);
        }

        //Update of current base velocities 
        for (size_t i = 0; i<6; i++)
        {
            dot_q_curr(i) = dot_q_temp(i);
            ni_curr(i) = dot_q_temp(i);
        }

        //Update of current joints positions and velocities 
        for (size_t i = 0; i<12; i++)
        {
            q_curr(i+6) = joint_handle[i].getPosition();
            dot_q_curr(i+6) = joint_handle[i].getVelocity();
        }

       //cout << "q_curr " << endl << q_curr << endl;

        //Computation of the inertia matrix
        CompositeRigidBodyAlgorithm(*model, q_curr, M_); 

        //cout <<"inertia matrix " << endl << M_ << endl;
        
        //Computation of the generalized forces
        NonlinearEffects(*model, q_curr, dot_q_curr, c_);

        QDot = VectorNd::Zero (18);

        //Computation of the gravity forces
        NonlinearEffects(*model, q_curr, QDot, g_);

        //Computation of Coriolis forces
        cor_ = c_ - g_;

        S_trans = S_.transpose();

        //Inizialization of the vector that gives the body id of the interested links (used in the CalcPointJacobian function)

        index_link = VectorNd::Zero(4);

        index_link(0) = model->GetBodyId("LF_FOOT");
        index_link(1) = model->GetBodyId("LH_FOOT");
        index_link(2) = model->GetBodyId("RF_FOOT");
        index_link(3) = model->GetBodyId("RH_FOOT");

        v1 << 0.0 , 0.0, 0.0;

        //Computation of the  Jacobians' contacts matrices
        CalcPointJacobian(*model2, q_curr, index_link(0), v1, J_c1);
    
        CalcPointJacobian(*model2, q_curr, index_link(1), v1, J_c2);
      
        CalcPointJacobian(*model2, q_curr, index_link(2), v1, J_c3);
      
        CalcPointJacobian(*model2, q_curr, index_link(3), v1, J_c4);


        // Computation of base position and velocity errors
            
        if(t > 2.5 )
        {
            for(size_t i = 0; i < 6; i++)
            {
                ni_int_des(i) = base_des(i);
                ni_des(i) = base_dot_des(i);
            }
        }
          
        err_6 = ni_int_des - ni_int_curr;
        dot_err_6 = ni_des - ni_curr; 

       
        //cout << "err"<< endl << err_6 << endl;

       
   
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if(c_switch == 0)
        {
            Fc = Eigen::MatrixXd::Zero(12, 1);
            Jc_ = Eigen::MatrixXd::Zero(12,18);
            J_c_scomp_1 = Eigen::MatrixXd::Zero(12, 6);
            J_c_scomp_2 = Eigen::MatrixXd::Zero(12, 12);
            J_c_scomp_2_inv = Eigen::MatrixXd::Zero(12, 12);
            Jc_trans = Eigen::MatrixXd::Zero(18,12);
            X_scomp = Eigen::MatrixXd::Zero(12, 6);
            id = Eigen::MatrixXd::Identity(6, 6);

            S_ker = Eigen::MatrixXd::Zero(18, 6);
            S_ker_trans = Eigen::MatrixXd::Zero(6, 18);
            S_S = Eigen::MatrixXd::Zero(6, 12);
            S_S_inv = Eigen::MatrixXd::Zero(12, 6);

            M_new = Eigen::MatrixXd::Zero(6, 6);
            C_new = Eigen::MatrixXd::Zero(6, 1);
            g_new = Eigen::MatrixXd::Zero(6, 1);

            u = Eigen::MatrixXd::Zero(6, 1);
            vel_des = Eigen::MatrixXd::Zero(3, 1);

            p = Eigen::MatrixXd::Zero(12, 12);
            p_inv = Eigen::MatrixXd::Zero(12, 12);

            Jc_ << J_c1, J_c2, J_c3, J_c4;

            //cout << "Jacobian 4 contact points"<< endl << Jc_<< endl;

            // Metodo delle quasi velocità

            Jc_trans = Jc_.transpose();


            for (size_t i = 0; i<12; i++)
            {
                for (size_t j = 0; j<6; j++)
                {
                    J_c_scomp_1(i,j) = Jc_(i,j);
                }
            }
            
            for (size_t i = 0; i<12; i++)
            {
                for (size_t j = 0; j<12; j++)
                {
                    J_c_scomp_2(i,j) = Jc_(i,j+6);
                }
            }
            
            J_c_scomp_2_inv = J_c_scomp_2.inverse();
            
            X_scomp = -J_c_scomp_2_inv *J_c_scomp_1*id;
            
        
           // KER di Jc (18x6)
            
            S_ker << id, X_scomp;

            //cout << "Jc_ * S_ker is zero (4 contact points): " << endl << Jc_* S_ker << endl;

            S_ker_trans = S_ker.transpose();

            
            // New dynamic system
            
            M_new = S_ker_trans*M_*S_ker;
            g_new = S_ker_trans*g_;

        
            S_S = (S_ker_trans*S_trans);
            pseudo_inverse(S_S,S_S_inv);

            //Differentiation of S_ker
            if(count == 1  || count == 4)
            {
                S_ker_prec = S_ker;
                count = count + 1;
            
            }
            
            S_ker_dot = df*(S_ker - S_ker_prec); 
            S_ker_prec = S_ker;
   

            C_new = (S_ker_trans*M_*S_ker_dot)*ni_curr + S_ker_trans*cor_;


            //Computed torque

            //Computation of the gain matrices

            kp1 = 1050;  //1200
            kv1 = 250; 
            
            /*media = sqrt((pow(dot_q_curr(6),2) + pow(dot_q_curr(8),2) + pow(dot_q_curr(12),2) + pow(dot_q_curr(14),2) + pow(dot_q_curr(9),2) + pow(dot_q_curr(11),2) + pow(dot_q_curr(12),2) + pow(dot_q_curr(17),2))/8);
            media = sqrt((pow(dot_q_curr(8),2) + pow(dot_q_curr(11),2) + pow(dot_q_curr(14),2) + pow(dot_q_curr(17),2))/4);
           cout<<"media"<<endl<<media<<endl;*/
        
            pos_foot_LF = CalcBodyToBaseCoordinates(*model,q_curr,index_link(0),v1);
            pos_foot_LH = CalcBodyToBaseCoordinates(*model,q_curr,index_link(1),v1);
            pos_foot_RF = CalcBodyToBaseCoordinates(*model,q_curr,index_link(2),v1);
            pos_foot_RH = CalcBodyToBaseCoordinates(*model,q_curr,index_link(3),v1);

            cout<<"pos_foot_LF_y"<<endl<<pos_foot_LF(1)<<endl;
            cout<<"pos_foot_LH_y"<<endl<<pos_foot_LH(1)<<endl;
            cout<<"pos_foot_RF_y"<<endl<<pos_foot_RF(1)<<endl;
            cout<<"pos_foot_RH_y"<<endl<<pos_foot_RH(1)<<endl;

            pos_foot_LF_media = sqrt(pow(pos_foot_LF(0),2) + pow(pos_foot_LF(1),2) + pow(pos_foot_LF(2),2));
            pos_foot_LH_media  = sqrt(pow(pos_foot_LH(0),2) + pow(pos_foot_LH(1),2) + pow(pos_foot_LH(2),2));
            
            pos_foot_RF_media  = sqrt(pow(pos_foot_RF(0),2) + pow(pos_foot_RF(1),2) + pow(pos_foot_RF(2),2));
            pos_foot_RH_media  = sqrt(pow(pos_foot_RH(0),2) + pow(pos_foot_RH(1),2) + pow(pos_foot_RH(2),2)); 

           /* cout<<"pos_foot_LF_media"<<endl<<pos_foot_LF_media<<endl;
            cout<<"pos_foot_LH_media"<<endl<<pos_foot_LH_media<<endl;
            cout<<"pos_foot_RF_media"<<endl<<pos_foot_RF_media<<endl;
            cout<<"pos_foot_RH_media"<<endl<<pos_foot_RH_media<<endl;*/

            diff1 = sqrt(pow(pos_foot_LF(1),2)+ pow(pos_foot_LH(1),2));
            //cout<<"diff1"<<endl<<diff1<<endl;

            diff2 = sqrt(pow(pos_foot_RF(1),2) + pow(pos_foot_RH(1),2));
            //cout<<"diff2"<<endl<<diff2<<endl;






            geometry_msgs::Twist pos_foot_L_msg;

            pos_foot_L_msg.linear.x = pos_foot_LF(0);
            pos_foot_L_msg.linear.y = pos_foot_LF(1);
            pos_foot_L_msg.linear.z = pos_foot_LF(2);

            pos_foot_L_msg.angular.x = pos_foot_LH(0);
            pos_foot_L_msg.angular.y = pos_foot_LH(1);
            pos_foot_L_msg.angular.z = pos_foot_LH(2);

            this-> pos_foot_L_pub.publish(pos_foot_L_msg);

            geometry_msgs::Twist pos_foot_R_msg;

            pos_foot_R_msg.linear.x = pos_foot_RF(0);
            pos_foot_R_msg.linear.y = pos_foot_RF(1);
            pos_foot_R_msg.linear.z = pos_foot_RF(2);

            pos_foot_R_msg.angular.x = pos_foot_RH(0);
            pos_foot_R_msg.angular.y = pos_foot_RH(1);
            pos_foot_R_msg.angular.z = pos_foot_RH(2);

            this-> pos_foot_R_pub.publish(pos_foot_R_msg);
            

            u =  M_new * ni_dot_des + kp1 * err_6 + kv1 * dot_err_6 + C_new - g_new;
           
           /*       
            cout<<"u(0)"<<endl<<u(0)<<endl;
             if (u(0) > 30 && t > 5)
               
            {
                u(0) = 24;
            }
             if (u(0) < -20 && t > 5)
                
            {
                u(0) = -4;
            }
            cout<<"u(0)"<<endl<<u(0)<<endl;

   
           cout<<"u(1)"<<endl<<u(1)<<endl;
             if (u(1) > 50  && t > 5)
            {
                u(1) = 33;
            }
             if (u(1) < -50  && t > 5)
            {
                u(1) = -33;
            }
            cout<<"u(1)"<<endl<<u(1)<<endl;


            cout<<"u(2)"<<endl<<u(2)<<endl;
            if (u(2) > 470 && t > 5)
            {
                u(2) = 419;
            }
             if (u(2) < 405 && t > 5)
            {
                u(2) = 419;
            }
            cout<<"u(2)"<<endl<<u(2)<<endl;
            
            

           cout<<"u(3)"<<endl<<u(3)<<endl;
             if (u(3) > 20 && t > 5)
            {
                u(3) = 14;
            }

             if (u(3) < -30 && t > 5)
            {
                u(3) = -11.3;
            }
            cout<<"u(3)"<<endl<<u(3)<<endl;

           
            cout<<"u(4)"<<endl<<u(4)<<endl;
             if (u(4) > 13 && t > 5)
            {
                u(4) = 1.88;
            }

            if (u(4) < -3 && t > 5)
            {
                u(4) = 1.88;
            }
            cout<<"u(4)"<<endl<<u(4)<<endl;

           cout<<"u(5)"<<endl<<u(5)<<endl;
            if (u(5) > 30 && t > 5)
            {
                u(5) = 18;
            }

              if (u(5) < -30 && t > 5)
            {
                u(5) = -18;
            }
            cout<<"u(5)"<<endl<<u(5)<<endl;*/
            
            //Control law
            tau_cmd = S_S_inv * u;
            
            tau_LF = sqrt(pow(tau_cmd(0,0),2) + pow(tau_cmd(1,0),2) + pow(tau_cmd(2,0),2));
            tau_LH = sqrt(pow(tau_cmd(3,0),2) + pow(tau_cmd(4,0),2) + pow(tau_cmd(5,0),2));
            
            tau_RF = sqrt(pow(tau_cmd(6,0),2) + pow(tau_cmd(7,0),2) + pow(tau_cmd(8,0),2));
            tau_RH = sqrt(pow(tau_cmd(9,0),2) + pow(tau_cmd(10,0),2) + pow(tau_cmd(11,0),2)); 
/*
            cout<<"tau_LF"<<endl<<tau_LF<<endl;
            cout<<"tau_LH"<<endl<<tau_LH<<endl;
            cout<<"tau_RF"<<endl<<tau_RF<<endl;
            cout<<"tau_RH"<<endl<<tau_RH<<endl;*/

    
            for (size_t i=0; i<12; i++)
            {
                joint_handle[i].setCommand(tau_cmd[i]);
            }
        
            //cout << "tau_cmd (quasi-velocity)"<< endl << tau_cmd <<endl;
            
            //Fc computations
            M_inv = M_.inverse();
            p = (Jc_ *M_inv* Jc_trans);
            p_inv = p.inverse();


            // Differentation of Jc_
            if(count == 2 || count == 5)
            {
                Jc_prec = Jc_;
                count = count + 1;
            }
            
            Jc_dot = df * (Jc_ - Jc_prec);
            Jc_prec = Jc_;

           // Contact force

            Fc = p_inv * (Jc_ * M_inv * (S_trans * tau_cmd - cor_ + g_) + Jc_dot * dot_q_curr); 

           
            /*cout<<"Fc_LF"<<endl<<Fc_LF<<endl;
            cout<<"Fc_LH"<<endl<<Fc_LH<<endl;
            cout<<"Fc_RF"<<endl<<Fc_RF<<endl;
            cout<<"Fc_RH"<<endl<<Fc_RH<<endl;*/

            //cout << "Contact force (4 contact points)"<< endl << Fc <<endl;
            pos_foot_LF = CalcBodyToBaseCoordinates(*model,q_curr,index_link(0),v1);
            pos_foot_LH = CalcBodyToBaseCoordinates(*model,q_curr,index_link(1),v1);
            pos_foot_RF = CalcBodyToBaseCoordinates(*model,q_curr,index_link(2),v1);
            pos_foot_RH = CalcBodyToBaseCoordinates(*model,q_curr,index_link(3),v1);

            /*foot_LF = sqrt(pow(pos_foot_LF(0,0),2) + pow(pos_foot_LF(1,0),2) + pow(pos_foot_LF(2,0),2));
            foot_LH = sqrt(pow(pos_foot_LH(0,0),2) + pow(pos_foot_LH(1,0),2) + pow(pos_foot_LH(2,0),2));
            foot_RF = sqrt(pow(pos_foot_RF(0,0),2) + pow(pos_foot_RF(1,0),2) + pow(pos_foot_RF(2,0),2));
            foot_RH = sqrt(pow(pos_foot_RH(0,0),2) + pow(pos_foot_RH(1,0),2) + pow(pos_foot_RH(2,0),2));*/

            if(count == 3 || count == 6)
            {
                foot_LF_prec = pos_foot_LF;
                foot_LH_prec = pos_foot_LH;
                foot_RF_prec = pos_foot_RF;
                foot_RH_prec = pos_foot_RH;
                count = count + 1;
            }
            
            pos_foot_LF_dot = df * (pos_foot_LF - foot_LF_prec);
            pos_foot_LH_dot = df * (pos_foot_LH - foot_LH_prec);
            pos_foot_RF_dot = df * (pos_foot_RF - foot_RF_prec);
            pos_foot_RH_dot = df * (pos_foot_RH - foot_RH_prec);

          /*  
            cout<<"pos_foot_LF_dot"<<endl<<pos_foot_LF_dot<<endl;
            cout<<"pos_foot_LH_dot"<<endl<<pos_foot_LH_dot<<endl;
            cout<<"pos_foot_RF_dot"<<endl<<pos_foot_RF_dot<<endl;
            cout<<"pos_foot_RH_dot"<<endl<<pos_foot_RH_dot<<endl;
*/
           
            foot_LF_prec = pos_foot_LF;
            foot_LH_prec = pos_foot_LH;
            foot_RF_prec = pos_foot_RF;
            foot_RH_prec = pos_foot_RH;


            delta_vel_LF = sqrt(pow(vel_des(0,0) - pos_foot_LF_dot(0,0),2) + pow(vel_des(1,0) - pos_foot_LF_dot(1,0),2) + pow(vel_des(2,0) - pos_foot_LF_dot(2,0),2));
            delta_vel_LH = sqrt(pow(vel_des(0,0) - pos_foot_LH_dot(0,0),2) + pow(vel_des(1,0) - pos_foot_LH_dot(1,0),2) + pow(vel_des(2,0) - pos_foot_LH_dot(2,0),2));
            delta_vel_RF = sqrt(pow(vel_des(0,0) - pos_foot_RF_dot(0,0),2) + pow(vel_des(1,0) - pos_foot_RF_dot(1,0),2) + pow(vel_des(2,0) - pos_foot_RF_dot(2,0),2));
            delta_vel_RH = sqrt(pow(vel_des(0,0) - pos_foot_RH_dot(0,0),2) + pow(vel_des(1,0) - pos_foot_RH_dot(1,0),2) + pow(vel_des(2,0) - pos_foot_RH_dot(2,0),2));

            //vel_z_media = sqrt((pow(pos_foot_LF_dot(0),2) + pow(pos_foot_LH_dot(0),2) + pow(pos_foot_RF_dot(0),2) + pow(pos_foot_RH_dot(0),2))/4);
            vel_LF_media = sqrt(pow(pos_foot_LF_dot(0),2) + pow(pos_foot_LF_dot(1),2) + pow(pos_foot_LF_dot(2),2));
            vel_LH_media = sqrt(pow(pos_foot_LH_dot(0),2) + pow(pos_foot_LH_dot(1),2) + pow(pos_foot_LH_dot(2),2));
            
            vel_RF_media = sqrt(pow(pos_foot_RF_dot(0),2) + pow(pos_foot_RF_dot(1),2) + pow(pos_foot_RF_dot(2),2));
            vel_RH_media = sqrt(pow(pos_foot_RH_dot(0),2) + pow(pos_foot_RH_dot(1),2) + pow(pos_foot_RH_dot(2),2));


             Fc_LF = sqrt(pow(Fc(0,0),2) + pow(Fc(1,0),2) + pow(Fc(2,0),2));
            Fc_LH = sqrt(pow(Fc(3,0),2) + pow(Fc(4,0),2) + pow(Fc(5,0),2));
            
            Fc_RF = sqrt(pow(Fc(6,0),2) + pow(Fc(7,0),2) + pow(Fc(8,0),2));
            Fc_RH = sqrt(pow(Fc(9,0),2) + pow(Fc(10,0),2) + pow(Fc(11,0),2)); 

            Ft_LFx = Fc(2,0)*mu;
            Ft_LFy = Fc(2,0)*mu;

            Ft_LHx = Fc(5,0)*mu;
            Ft_LHy = Fc(5,0)*mu;

            Ft_RFx = Fc(8,0)*mu;
            Ft_RFy = Fc(8,0)*mu;

            Ft_RHx = Fc(11,0)*mu;
            Ft_RHy = Fc(11,0)*mu;

            //cout<<"Ft_LFx"<<endl<<Ft_LFx<<endl;

            

            cout<<"vel_LF_media"<<endl<<vel_LF_media<<endl;
            cout<<"vel_LH_media"<<endl<<vel_LH_media<<endl;
            cout<<"vel_RF_media"<<endl<<vel_RF_media<<endl;
            cout<<"vel_RH_media"<<endl<<vel_RH_media<<endl;


            if(vel_LF_media > 0.2 && vel_LF_media < 20 && t > 5){
                cout<<"sto scivolando con la LF"<<endl;
                contovel ++;
                cout<<"contovel"<<endl<<contovel<<endl;
            }

            if(vel_RF_media > 0.2 && vel_RF_media < 20 && t > 5){
                cout<<"sto scivolando con la RF"<<endl;
                 contovel ++;
                 cout<<"contovel"<<endl<<contovel<<endl; 
            }

             if(vel_LH_media > 0.2 && vel_LH_media < 20 && t > 5){
                cout<<"sto scivolando con la LH"<<endl;
                 contovel ++;
                 cout<<"contovel"<<endl<<contovel<<endl;
            }

             if(vel_RH_media > 0.2 && vel_RH_media < 20 && t > 5){
                cout<<"sto scivolando con la LH"<<endl;
                 contovel ++;
                 cout<<"contovel"<<endl<<contovel<<endl;
            }

             if(contovel == 4){
                cout<<"tvybimòoimoiòjvhchxserctyuihgtdryulhvcvhgtvbyuniòhvybghnlhcrdvbyghnmòorcxeccno"<<endl;
                //abort();
                freeze = 1;

            }

           
            else {
                contovel = 0;
            }

           /* if ( Ft_LFx > max_LF && t > 5){
                cout<<"fuori conooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo LF"<<endl;
                cout<<"Ft_LFx"<<endl<<Ft_LFx<<endl;
                contocono ++;
                cout<<"contocono"<<endl<<contocono<<endl;
                
            }
             if ( Ft_LHx > max_LH && t > 5){
                cout<<"fuori conooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo LH"<<endl;
                cout<<"Ft_LHx"<<endl<<Ft_LHx<<endl;
                contocono ++;
                cout<<"contocono"<<endl<<contocono<<endl;
                
            }

             if ( Ft_RFx > max_RF && t > 5){
                cout<<"fuori conooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo RF"<<endl;
                cout<<"Ft_RFx"<<endl<<Ft_RFx<<endl;
                contocono ++;
                cout<<"contocono"<<endl<<contocono<<endl;
                
            }

             if ( Ft_RHx > max_RH && t > 5){
                cout<<"fuori conooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo RH"<<endl;
                cout<<"Ft_RHx"<<endl<<Ft_RHx<<endl;
                contocono ++;
                cout<<"contocono"<<endl<<contocono<<endl;
            }

            if(contocono == 2 || contocono == 3 || contocono == 4){
                cout<<"bastaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
                contocono = 0;
            }

            else {
                contocono = 0;
            }
           
*/
        



           /* if (vel_z_media > 0.18 &&  vel_z_media < 0.2643 && t > 5){ 
                cout<<"sto scivolandoooooooooooooooooooooooooooooo"<<endl;
            }*/
             /*  if((pos_foot_LF(1) < 0.41 && pos_foot_LF(1) > 0.4 || pos_foot_LF(1) < 0.18 && pos_foot_LF(1) > 0.17) && t > 10) {
               cout<<"pos_foot_LF(1)"<<endl<<pos_foot_LF(1)<<endl;
               cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<endl;
                scivolo = 0;
                
               a =  1;

              
           }
           
           
            if((pos_foot_RF(1) > -0.41 && pos_foot_RF(1) < -0.4 || pos_foot_RF(1) < -0.17 && pos_foot_RF(1) > -0.18) && t > 10) {
               cout<<"pos_foot_RF(1)"<<endl<<pos_foot_RF(1)<<endl;
               cout<<"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"<<endl;
                scivolo = 0;
               
               b = 1;
              
              
           }

           if((pos_foot_LH(1) < 0.37 && pos_foot_LH(1) > 0.36 || pos_foot_LH(1) < 0.20 && pos_foot_LH(1) > 0.18) && t > 10) {
               cout<<"pos_foot_LH(1)"<<endl<<pos_foot_LH(1)<<endl;
               cout<<"cccccccccccccccccccccccccccccccccccccccccccccc"<<endl;
               scivolo_dietro_due = 0;
             
               c =  1;
               
           }

             if((pos_foot_RH(1) > -0.35 && pos_foot_RH(1) < -0.34 || pos_foot_RH(1) < -0.175 && pos_foot_RH(1) > -0.18) && t > 10) {
               cout<<"pos_foot_RH(1)"<<endl<<pos_foot_RH(1)<<endl;
               cout<<"ddddddddddddddddddddddddddddddddddddddddddddddddd"<<endl;
               scivolo_dietro_due = 0;
               d = 1;
               
               }*/



                if( pos_foot_LF(1) > 0.4  && t > 10) {
               cout<<"pos_foot_LF(1)"<<endl<<pos_foot_LF(1)<<endl;
               cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<endl;
                scivolo = 0;
                
               a =  1;

              
           }

            if( pos_foot_RF(1) < -0.4 && t > 10) {
               cout<<"pos_foot_RF(1)"<<endl<<pos_foot_RF(1)<<endl;
               cout<<"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"<<endl;
                scivolo = 0;
               
               b = 1;
              
              
           }

            if(pos_foot_LH(1) < 0.37 && t > 10) {
               cout<<"pos_foot_LH(1)"<<endl<<pos_foot_LH(1)<<endl;
               cout<<"cccccccccccccccccccccccccccccccccccccccccccccc"<<endl;
               scivolo_dietro_due = 0;
             
               c =  1;
               
           }

            if(pos_foot_RH(1) < -0.34 && t > 10) {
               cout<<"pos_foot_RH(1)"<<endl<<pos_foot_RH(1)<<endl;
               cout<<"ddddddddddddddddddddddddddddddddddddddddddddddddd"<<endl;
               scivolo_dietro_due = 0;
               d = 1;
               
               }
        
           
       
           
           /*if((pos_foot_LH(1) < 0.38 && pos_foot_LH(1) > 0.37 || pos_foot_LH(1) < 0.20 && pos_foot_LH(1) > 0.18) && t > 10) {
               cout<<"pos_foot_LH(1)"<<endl<<pos_foot_LH(1)<<endl;
               cout<<"cccccccccccccccccccccccccccccccccccccccccccccc"<<endl;
               c =  1;
           }*/

            
          
              
           
           /*
            if((pos_foot_LF(1) > 0.33 || pos_foot_LF(1) < 0.2) && t > 10) {
               cout<<"pos_foot_LF(1)"<<endl<<pos_foot_LF(1)<<endl;
               cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<endl;
               a =  1;
           }
               if((pos_foot_RF(1) < -0.34 || pos_foot_RF(1) > -0.2) && t > 10) {
               cout<<"pos_foot_RF(1)"<<endl<<pos_foot_RF(1)<<endl;
               cout<<"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"<<endl;
               b = 1;
              
           }
          if((pos_foot_LH(1) > 0.32 || pos_foot_LH(1) < 0.2) && t > 10) {
               cout<<"pos_foot_LH(1)"<<endl<<pos_foot_LH(1)<<endl;
               cout<<"cccccccccccccccccccccccccccccccccccccccccccccc"<<endl;
               c =  1;
           }
           if((pos_foot_RH(1) < -0.34 ||  pos_foot_RH(1) > -0.2) && t > 10) {
               cout<<"pos_foot_RH(1)"<<endl<<pos_foot_RH(1)<<endl;
               cout<<"ddddddddddddddddddddddddddddddddddddddddddddddddd"<<endl;
               d = 1;}
           */
          
           
             if(t > 10 && (a == 1 || b == 1) && endd == 0)
           {    friction_cone = 1;
                 scivolo = 0;
               scivolo_dietro = 0;
               scivolo_dietro_due = 0;
               scivolo_due = 0;
               cout<<"friction_cone"<<endl<<friction_cone<<endl;
               
               friction_dietro = 0;
               friction_dietro_due = 0;
               
             
               //abort();
            
           }
           else{
               friction_cone = 0;
           }

           if(t > 10 && (c == 1 || d == 1) && endd == 1)
           {
                friction_cone_dietro = 1;
                scivolo = 0;
               scivolo_dietro = 0;
               scivolo_dietro_due = 0;
               scivolo_due = 0;
               cout<<"friction_cone_dietro"<<endl<<friction_cone_dietro<<endl;
              
               friction_dietro = 0;
               friction_dietro_due = 0;
            
              
            
               //abort();
           }
           else{
               friction_cone_dietro = 0;        
           }

           
           cout<<"fine"<<endl<<fine<<endl;


           /* if((pos_foot_LH(1) < 0.41 && pos_foot_LH(1) > 0.38 || pos_foot_LH(1) < 0.27 && pos_foot_LH(1) > 0.24) && t > 10) {
               cout<<"pos_foot_LH(1)"<<endl<<pos_foot_LH(1)<<endl;
               cout<<"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"<<endl;
               e =  1;
           }

            if((pos_foot_RH(1) > -0.36 && pos_foot_RH(1) < -0.35 || pos_foot_RH(1) < -0.13 && pos_foot_RH(1) > -0.16) && t > 10) {
               cout<<"pos_foot_RH(1)"<<endl<<pos_foot_RH(1)<<endl;
               cout<<"fffffffffffffffffffffffffffffffffffffffffffff"<<endl;
               f = 1;}
            
            if((pos_foot_RH(1) > -0.39 && pos_foot_RH(1) < -0.38 || pos_foot_RH(1) < -0.13 && pos_foot_RH(1) > -0.16) && t > 10) {
               cout<<"pos_foot_RH(1)"<<endl<<pos_foot_RH(1)<<endl;
               cout<<"fffffffffffffffffffffffffffffffffffffffffffff"<<endl;
               f = 1;}

                 if((pos_foot_LH(1) < 0.41 && pos_foot_LH(1) > 0.4 || pos_foot_LH(1) < 0.27 && pos_foot_LH(1) > 0.24) && t > 10) {
               cout<<"pos_foot_LH(1)"<<endl<<pos_foot_LH(1)<<endl;
               cout<<"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"<<endl;
               e =  1;
           }


           if((pos_foot_RH(1) > -0.39 && pos_foot_RH(1) < -0.38 || pos_foot_RH(1) < -0.13 && pos_foot_RH(1) > -0.15) && t > 10) {
               cout<<"pos_foot_RH(1)"<<endl<<pos_foot_RH(1)<<endl;
               cout<<"fffffffffffffffffffffffffffffffffffffffffffff"<<endl;
               f = 1;}

                 if((pos_foot_LH(1) < 0.41 && pos_foot_LH(1) > 0.4 || pos_foot_LH(1) < 0.18 && pos_foot_LH(1) > 0.16) && t > 10) {
               cout<<"pos_foot_LH(1)"<<endl<<pos_foot_LH(1)<<endl;
               cout<<"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"<<endl;
               e =  1;
           }*/

            if((pos_foot_RH(1) > -0.39 && pos_foot_RH(1) < -0.38 || pos_foot_LH(1) < 0.37 && pos_foot_LH(1) > 0.36) && t > 10) {
               cout<<"pos_foot_RH(1)"<<endl<<pos_foot_RH(1)<<endl;
               cout<<"fffffffffffffffffffffffffffffffffffffffffffff"<<endl;
               scivolo = 0;
               scivolo_dietro = 0;
               scivolo_dietro_due = 0;
               scivolo_due = 0;
               friction_cone = 0;
               friction_cone_dietro = 0;
               f = 1;
               
              
               }

                 if((pos_foot_RH(1) < -0.13 && pos_foot_RH(1) > -0.15 || pos_foot_LH(1) < 0.18 && pos_foot_LH(1) > 0.16) && t > 10) {
               cout<<"pos_foot_LH(1)"<<endl<<pos_foot_LH(1)<<endl;
               cout<<"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"<<endl;
               scivolo = 0;
               scivolo_dietro = 0;
               scivolo_dietro_due = 0;
               scivolo_due = 0;
                 
               friction_cone = 0;
               friction_cone_dietro = 0;
               e =  1;
             
           }


             if(t > 95 && t < 200 && f == 1){
                   cout<<"pos_foot_LH(1)"<<endl<<pos_foot_LH(1)<<endl;
                   cout<<"pos_foot_RH(1)"<<endl<<pos_foot_RH(1)<<endl;
                //friction_cone = 0;
                   //friction_cone_dietro = 0;
                    scivolo = 0;
                   scivolo_dietro = 0;
                   scivolo_dietro_due = 0;
                   scivolo_due = 0;
                   a = 0;
                   b = 0; 
                   c = 0;
                   d = 0;
                    e = 0;
                    friction_cone_dietro = 0;
                   friction_dietro = 1;
                  /* 
                   e = 0;
                   endd = 1;*/
                  
                 

               }
               else{
                   friction_dietro = 0;
    
                  
                   }
                   
                    if(t > 95 && e == 1){
                   cout<<"pos_foot_LH(1)"<<endl<<pos_foot_LH(1)<<endl;
                   cout<<"pos_foot_RH(1)"<<endl<<pos_foot_RH(1)<<endl;
                   a = 0;
                   b = 0; 
                   c = 0;
                   d = 0;
                     f = 0;
                  //friction_cone = 0;
                  //friction_cone_dietro = 0;
                     scivolo = 0;
               scivolo_dietro = 0;
               scivolo_dietro_due = 0;
               scivolo_due = 0;
                   friction_dietro_due = 1;
                  /* a = 0;
                   b = 0;
                   c = 0;
                   d = 0;
                   f = 0;
                   endd = 1;*/
                 
                   
               }
               else{
                   friction_dietro_due = 0;
                
                  
                   }

           cout<<"pos_foot_LF(0)"<<endl<<pos_foot_LF(0)<<endl;
          /*  if(pos_foot_LF(0) > 0.45 && t > 10) {
               
               cout<<"pos_foot_LF(0)"<<endl<<pos_foot_LF(0)<<endl;
               cout<<"scivolo in avantiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii"<<endl;
               /*friction_cone = 0;
               friction_cone_dietro = 0;
                a = 0;
                b = 0;
                c = 0;
                d = 0;
               e = 0;
               f = 0;
               scivolo_due = 0;
               scivolo_dietro = 0;   
               scivolo_dietro_due = 0;
               
               scivolo = 1;
               
               //friction_dietro = 0;
               //friction_dietro_due = 0;

               //abort();
              
           }

           else{
               scivolo = 0;
               }


                if(pos_foot_RF(0) > 0.45 && t > 10) {
               cout<<"pos_foot_RF(0)"<<endl<<pos_foot_RF(0)<<endl;
               
               cout<<"scivolo in avantiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii"<<endl;
              
               scivolo_dietro = 0;
               scivolo_dietro_due = 0;

                /*friction_cone = 0;
               friction_cone_dietro = 0;
              a = 0;
              b = 0;
              c = 0;
              d = 0;
               e = 0;
               f = 0;
               
               scivolo = 0;
               scivolo_due = 1;
               
               //friction_dietro = 0;
               //friction_dietro_due = 0;

               //abort();
              
           }

           else{
               scivolo_due = 0;       
               }*/



/*
                if(pos_foot_LH(0) < -0.68 && t > 10) {
                      friction_cone = 0;
               friction_cone_dietro = 0;
                        
               cout<<"pos_foot_LH(0)"<<endl<<pos_foot_LH(0)<<endl;
               cout<<"scivolo indietroooooooooooooooooo"<<endl;
        
               a = 0;
              b = 0;
              c = 0;
              d = 0;
               e = 0;
               f = 0;
               scivolo_dietro_due = 0;
               scivolo = 0;
               scivolo_due = 0;
      
               scivolo_dietro = 1;
             
               
               
               //friction_dietro = 0;
               //friction_dietro_due = 0;

               //abort();
              
           } 

           else{
               scivolo_dietro = 0;
               }


                if(pos_foot_RH(0) < -0.68 && t > 10) {
                     cout<<"pos_foot_RH(0)"<<endl<<pos_foot_RH(0)<<endl;
                cout<<"scivolo indietrooooo"<<endl;
                
               friction_cone = 0;
               friction_cone_dietro = 0;
                a = 0;
              b = 0;
              c = 0;
              d = 0;
               e = 0;
               f = 0;
               scivolo_dietro = 0;
               scivolo = 0;
               scivolo_due = 0;
              
               scivolo_dietro_due = 1;
               
                
              
               
               //friction_dietro = 0;
               //friction_dietro_due = 0;

               //abort();
              
           }

           else{
               scivolo_dietro_due = 0;
               }


*/ 
          /* if(t>2.5){
                
            cout<<"delta_vel_LF"<<endl<<delta_vel_LF<<endl;
            cout<<"delta_vel_LH"<<endl<<delta_vel_LH<<endl;
            cout<<"delta_vel_RF"<<endl<<delta_vel_RF<<endl;
            cout<<"delta_vel_RH"<<endl<<delta_vel_RH<<endl;}*/

            //vel_media = sqrt((pow(pos_foot_LF_dot,2) +  pow(pos_foot_LH_dot,2) +  pow(pos_foot_RF_dot,2) +  pow(pos_foot_RH_dot,2))/4);
            //if (vel_media > 0.060 && vel_media < 0.080 && t > 2.5){
             //   cout<<"vel_media"<<endl<<vel_media<<endl;
               // cout<<"sto scivolando"<<endl;
            // 0.012; }

            geometry_msgs::Twist vel_foot_msg;
            vel_foot_msg.linear.x = vel_LF_media;
            vel_foot_msg.linear.y = vel_LH_media;
            vel_foot_msg.linear.z = vel_RF_media;

            vel_foot_msg.angular.x = vel_RH_media;

            this-> vel_foot_pub.publish(vel_foot_msg);

            

            /*if(contatore > 2.998)
            {
                for(size_t i = 0; i < 6; i++)
                {
                    q_base(i) = q_curr(i); 
                } 

                geometry_msgs::Twist pos_base_pub;  

                pos_base_pub.linear.x = q_base(0);
                pos_base_pub.linear.y = q_base(1);
                pos_base_pub.linear.z = q_base(2);
                pos_base_pub.angular.x = q_base(3);
                pos_base_pub.angular.y = q_base(4);
                pos_base_pub.angular.z = q_base(5);

                this-> pos_base_pub.publish(pos_base_pub);
            }*/
            
            
            for(size_t i = 0; i < 12; i++)
            {
                q_init_leg(i) = q_curr(i+6);
                command_q_des(i) = q_curr(i+6);
            } 
        
            geometry_msgs::Twist q_i_L;  // LF-LH

            q_i_L.linear.x = q_init_leg(0);
            q_i_L.linear.y = q_init_leg(1);
            q_i_L.linear.z = q_init_leg(2);
            q_i_L.angular.x = q_init_leg(3);
            q_i_L.angular.y = q_init_leg(4);
            q_i_L.angular.z = q_init_leg(5);

            geometry_msgs::Twist q_i_R;  // RF-RH

            q_i_R.linear.x = q_init_leg(6);
            q_i_R.linear.y = q_init_leg(7);
            q_i_R.linear.z = q_init_leg(8);
            q_i_R.angular.x = q_init_leg(9);
            q_i_R.angular.y = q_init_leg(10);
            q_i_R.angular.z = q_init_leg(11);

            this-> q_i_L_pub.publish(q_i_L);
            this-> q_i_R_pub.publish(q_i_R);
        
            

            cout << "time"<< endl<<t<<endl;
            //cout << "contatore" << endl<<contatore<<endl;

       }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        else 
        {       
            contatore = 0;
                   
            Kp_12 = Eigen::MatrixXd::Identity(12, 12);
            Kv_12 = Eigen::MatrixXd::Identity(12, 12);
        

            for(size_t i = 0; i < 12; i++)
            {
                q_des(i) = command_q_des(i);
                q_dot_des(i) = command_dot_q_des(i);
            }


            // Computation of position and velocity errors

            for (size_t i=0; i<12; i++)
            {
                err_12(i)     = 0;
                dot_err_12(i) = 0;
            }

            for (size_t i=0; i<12; i++)
            {
                err_12(i)     = q_des(i) - q_curr(i+6);
                dot_err_12(i) = q_dot_des(i) - dot_q_curr(i+6);
            }

            //cout << "err pos " << endl << err_12 <<endl;

            //Computation of the gain matrices

            kp2 = 25000;
            kv2 = 15;
            kp3 = 25000;
            kv3 = 15;    

            //Computation of the gain matrices

            for (size_t i=0; i<12; i++)
            {
                for (size_t j=0; j<12; j++)
                {
                    if (i==j && i<3)
                    {
                        Kp_12(i,j) = kp2 * Kp_12(i,j);
                        Kv_12(i,j) = kv2 * Kv_12(i,j);
                    }
                    else 
                    {
                        Kp_12(i,j) = kp3 * Kp_12(i,j);
                        Kv_12(i,j) = kv3 * Kv_12(i,j);
                    }
                }
            }

            for (size_t i=0; i<12; i++)
            {
                // PD with gravity compensation

                tau_cmd(i) = Kp_12(i,i) * err_12(i) + Kv_12(i,i) * dot_err_12(i) - g_(i+6);
            }
    
            for (size_t i=0; i<12; i++)
            {
                joint_handle[i].setCommand(tau_cmd[i]);
            }
            
            //cout << "tau_cmd (PD)"<< endl << tau_cmd <<endl;
           
        }

        freeze_msg.data = freeze;
        this->freeze_pub.publish(freeze_msg);

        scivolo_msg.data = scivolo;
        this->scivolo_pub.publish(scivolo_msg);

        scivolo_dietro_msg.data = scivolo_dietro;
        this->scivolo_dietro_pub.publish(scivolo_dietro_msg);

        scivolo_due_msg.data = scivolo_due;
        this->scivolo_due_pub.publish(scivolo_due_msg);

        scivolo_dietro_due_msg.data = scivolo_dietro_due;
        this->scivolo_dietro_due_pub.publish(scivolo_dietro_due_msg);

        friction_cone_msg.data = friction_cone;
        this->friction_cone_pub.publish(friction_cone_msg);

        friction_dietro_msg.data = friction_dietro;
        this->friction_dietro_pub.publish(friction_dietro_msg);

        friction_dietro_due_msg.data = friction_dietro_due;
        this->friction_dietro_due_pub.publish(friction_dietro_due_msg);

        friction_cone_dietro_msg.data = friction_cone_dietro;
        this->friction_cone_dietro_pub.publish(friction_cone_dietro_msg);

        //Plot of position errors
        geometry_msgs::Twist error_pos;
   
        error_pos.linear.x = err_6(0);
        error_pos.linear.y = err_6(1);
        error_pos.linear.z = err_6(2);
        error_pos.angular.x = err_6(3);
        error_pos.angular.y = err_6(4);
        error_pos.angular.z = err_6(5);
        this-> error_pos.publish(error_pos);


        //Plot of Fc
        geometry_msgs::Twist force_L;
        geometry_msgs::Twist force_R;

        //LF and LH 
        
        force_L.linear.x = Ft_LFx;
        force_L.linear.y = Ft_LFy;
        force_L.linear.z = Ft_LHx;
        force_L.angular.x = Ft_LHy;

       /* force_L.linear.x = Fc_LF;
        force_L.linear.y = Fc_LH;
        force_L.linear.z = Fc_RF;
        force_L.angular.x = Fc_RH;*/
        force_L.angular.y = Ft_RFx;
        force_L.angular.z = Ft_RFy;

        //RF and RH
        force_R.linear.x = pos_foot_LF(0);
        force_R.linear.y = pos_foot_LH(0);
        force_R.linear.z = pos_foot_RF(0);
        force_R.angular.x = pos_foot_RH(0);
        force_R.angular.y = Fc(8,0);
        force_R.angular.z = Fc(11,0);

        this-> force_L.publish(force_L);
        this-> force_R.publish(force_R);


        //Plot of tau
        geometry_msgs::Twist tau_1_6;
        geometry_msgs::Twist tau_7_12;

        tau_1_6.linear.x = u(0);
        tau_1_6.linear.y = u(1);
        tau_1_6.linear.z = u(2);
        tau_1_6.angular.x = u(3);
        /*
        tau_1_6.linear.x = tau_cmd(0,0);
        tau_1_6.linear.y = tau_cmd(1,0);
        tau_1_6.linear.z = tau_cmd(2,0);
        tau_1_6.angular.x = tau_cmd(3,0);*/
        tau_1_6.angular.y = u(4);
        tau_1_6.angular.z = u(5);

        
        tau_7_12.linear.x = diff1;
        tau_7_12.linear.y = diff2;
        tau_7_12.linear.z = pos_foot_RF_media;
        tau_7_12.angular.x = pos_foot_RH_media;
       // tau_7_12.angular.y = diff;
        tau_7_12.angular.z = tau_cmd(11,0);

        geometry_msgs::Twist vel_L_pub;
        geometry_msgs::Twist vel_R_pub;

        vel_L_pub.linear.x = vel_LF_media;
        vel_L_pub.linear.y = vel_LH_media;
        vel_L_pub.linear.z = vel_RF_media;
        vel_L_pub.angular.x = vel_RH_media;
        vel_L_pub.angular.y = dot_q_curr(10,0);
        vel_L_pub.angular.z = dot_q_curr(11,0); 

        
        vel_R_pub.linear.x = dot_q_curr(12,0);
        vel_R_pub.linear.y = dot_q_curr(13,0);
        vel_R_pub.linear.z = dot_q_curr(14,0); 
        vel_R_pub.angular.x = dot_q_curr(15,0);
        vel_R_pub.angular.y = dot_q_curr(16,0);
        vel_R_pub.angular.z = dot_q_curr(17,0);


        /*
        tau_1_6.linear.x = command_q_des(0);
        tau_1_6.linear.y = command_q_des(1);
        tau_1_6.linear.z = command_q_des(2);
        tau_1_6.angular.x = command_q_des(3);
        tau_1_6.angular.y = command_q_des(4);
        tau_1_6.angular.z = command_q_des(5);

        tau_7_12.linear.x = command_q_des(6);
        tau_7_12.linear.y = command_q_des(7);
        tau_7_12.linear.z = command_q_des(8);
        tau_7_12.angular.x = contatore;*/

        this-> vel_L_pub.publish(vel_L_pub);
        this-> vel_R_pub.publish(vel_R_pub);

        this-> tau_1_6.publish(tau_1_6);
        this-> tau_7_12.publish(tau_7_12);

       
    }
    
    void floating_base_controller::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)
    {
        command_q_des = Eigen::Map<const Eigen::Matrix<double, 12, 1>>((msg->position).data());
        command_dot_q_des = Eigen::Map<const Eigen::Matrix<double, 12, 1>>((msg->velocity).data());
        command_dot_dot_q_des = Eigen::Map<const Eigen::Matrix<double, 12, 1>>((msg->effort).data()); 

    }
    void floating_base_controller::setCommandCB_base(const sensor_msgs::JointStateConstPtr& msg)
    {
        base_des = Eigen::Map<const Eigen::Matrix<double, 6, 1>>((msg->position).data());
        base_dot_des = Eigen::Map<const Eigen::Matrix<double, 6, 1>>((msg->velocity).data());
        base_dot_dot_des = Eigen::Map<const Eigen::Matrix<double, 6, 1>>((msg->effort).data());
    }

    void floating_base_controller::c_switch_setCommand(const std_msgs::BoolConstPtr& msg)
    {
        c_switch = msg->data;
    }

    void floating_base_controller::fine_setCommand(const std_msgs::BoolConstPtr& msg)
    {
        fine = msg->data;
    }


    void floating_base_controller::endd_setCommand(const std_msgs::BoolConstPtr& msg)
    {
        endd = msg->data;
    }

    void floating_base_controller::scivolo_setCommand(const std_msgs::BoolConstPtr& msg)
    {
        scivolo = msg->data;
    }


    void floating_base_controller::scivolo_dietro_setCommand(const std_msgs::BoolConstPtr& msg)
    {
        scivolo_dietro = msg->data;
    }

     void floating_base_controller::scivolo_due_setCommand(const std_msgs::BoolConstPtr& msg)
    {
        scivolo_due = msg->data;
    }

    void floating_base_controller::scivolo_dietro_due_setCommand(const std_msgs::BoolConstPtr& msg)
    {
        scivolo_dietro_due = msg->data;
    }

    void floating_base_controller::state_estimator(const gazebo_msgs::LinkStatesConstPtr& msg)

    {
        q_temp[0] = msg->pose[1].position.x;
        q_temp[1] = msg->pose[1].position.y;
        q_temp[2] = msg->pose[1].position.z;
        q_temp[3] = msg->pose[1].orientation.x;
        q_temp[4] = msg->pose[1].orientation.y;
        q_temp[5] = msg->pose[1].orientation.z;
        q_temp[6] =  msg->pose[1].orientation.w;

        dot_q_temp[0] = msg->twist[1].linear.x;
        dot_q_temp[1] = msg->twist[1].linear.y;
        dot_q_temp[2] = msg->twist[1].linear.z;
        dot_q_temp[3] = msg->twist[1].angular.x;
        dot_q_temp[4] = msg->twist[1].angular.y;
        dot_q_temp[5] = msg->twist[1].angular.z;

    }

    PLUGINLIB_EXPORT_CLASS(my_controller_ns::floating_base_controller, controller_interface::ControllerBase);


}

//1,030050
//-0,041995

//0,993602
//0,055009

//0,953435
//-0,030020

//0,933504
//-0,072252

//0,946882
//-0,058936

//0,950090
//-0,033333

//1,003510
//-0,090465

//0,993626
//-0,034393

//0,993618
//0,077632

//0,960198
//-0,045559

//0,960198
//-0,045559

//0,983586
//-0,022170

//0,966903
//-0,038916

//1,141460
//-0,017677

//1,069150
//-0,299966

//0.72
//1,043300
//-0,028708

//0.75 1.3
//1,172250
//-0,093386

//0.75 1.3
//1,188490
//-0,077115

//0.75 1.3 finito
//1,212060
//-0,157856

//uguale a sopra
//1,177180
//-0,003433
//1,228130
//-0,141793
//1,213040
//-0,115483
//1,232950
//-0,074729

//1,284690
//-0,022926

//good 1.5
//1,343970
//-0,107726
//1,395490
//-0,116645