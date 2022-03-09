
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
    dot_dot_err_6 = Eigen::VectorXd::Zero(6);

    dot_dot_q_temp = Eigen::VectorXd::Zero(6);

    err_12 = Eigen::VectorXd::Zero(12);
    dot_err_12 = Eigen::VectorXd::Zero(12);
    dot_dot_err_12 = Eigen::VectorXd::Zero(12);


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
        //dot_dot_q_curr(i+6) = joint_handle[i].getEffort();

    }
    
    

    //Subscriber that subscribes a topic that gives the desired positions and velocities vectors of the joints
    this->sub_command_ = n.subscribe<sensor_msgs::JointState> ("command", 1, &floating_base_controller::setCommandCB, this); 

    this->sub_command_base = n.subscribe<sensor_msgs::JointState> ("command_base", 1, &floating_base_controller::setCommandCB_base, this); 
    
    //Subscriber that subscribes a topic from Gazebo that gives the pose of the unactuated joints
    this-> sub_gaze = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, &floating_base_controller::state_estimator, this);

    this-> error_pos = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/error_pos", 1); 
    this-> error_vel = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/error_vel", 1); 
    this-> error_acc = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/error_acc", 1);  
    this-> force_L = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/force_L", 1); 
    this-> force_R = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/force_R", 1); 
    //this-> sing_values = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/sing_values", 1);
    this-> tau_1_6 = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/tau_1_6", 1);
    this-> tau_7_12 = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/tau_7_12", 1);

    this-> pos_foot_L_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/pos_foot_L_pub", 1);
    this-> pos_foot_R_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/pos_foot_R_pub", 1);

    this-> q_i_L_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/q_i_L_pub", 1); 
    this-> q_i_R_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/q_i_R_pub", 1);

    this-> pos_base_pub = n.advertise<geometry_msgs::Twist>("/anymal/floating_base_controller/pos_base_pub", 1);

    this-> c_switch_sub = n.subscribe<std_msgs::Bool>("get_c_switch_set_command", 1, &floating_base_controller::c_switch_setCommand, this);

    this-> slope_pub = n.advertise<std_msgs::Bool>("/anymal/floating_base_controller/slope_pub", 1);
    
     

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

    
    ni_int_des(0) = 0.03; //0.03;
    ni_int_des(1) = -0.08; //-0.08;
    ni_int_des(2) = 0.55; //0.54
    ni_int_des(3) = 0.0; //0.02;
    ni_int_des(4) = 0.0;
    ni_int_des(5) = 0.0; 
    
    //camminata più bassa
   /* ni_int_des(0) = 0.03;
    ni_int_des(1) = -0.06;
    ni_int_des(2) = 0.48;
    ni_int_des(3) = 0.01;
    ni_int_des(4) = 0.0;
    ni_int_des(5) = 0.0;*/

    /*ni_int_des(0) = 0.0;
    ni_int_des(1) = -0.0;
    ni_int_des(2) = 0.65; //0.54
    ni_int_des(3) = 0.0;
    ni_int_des(4) = -0.26;
    ni_int_des(5) = 0.0; */

  


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

        t = ros::Time::now().toSec(); 

        //Initialization of joints configurations vectors
        for (size_t i = 0; i < 18; i++)
        {
            q_curr(i) = 0;
            dot_q_curr(i) = 0;
            //dot_dot_q_curr(i) = 0;
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

        //Update of current base accelerations
      /*  for (size_t i = 0; i<6; i++)
       {
            dot_dot_q_curr(i) = dot_dot_q_temp(i);
            ni_dot_curr(i) = dot_dot_q_temp(i);
        }*/

        //Update of current joints positions and velocities 
        for (size_t i = 0; i<12; i++)
        {
            q_curr(i+6) = joint_handle[i].getPosition();
            dot_q_curr(i+6) = joint_handle[i].getVelocity();
            //dot_dot_q_curr(i+6) = joint_handle[i].getEffort();
        }

        cout << "q_curr " << endl << q_curr << endl;

        //Computation of the inertia matrix
        CompositeRigidBodyAlgorithm(*model, q_curr, M_); 

        //cout <<"inertia matrix " << endl << M_ << endl;
        
        //Computation of the generalized forces
        NonlinearEffects(*model, q_curr, dot_q_curr, c_);

        QDot = VectorNd::Zero (18);

        //Computation of the gravity forces
        NonlinearEffects(*model, q_curr, QDot, g_);
        
        //cout << "gravity" << endl << g_<<endl;

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
            
        /*if(t > 2.5 )
        {
            for(size_t i = 0; i < 6; i++)
            {
                ni_int_des(i) = base_des(i);
                ni_des(i) = base_dot_des(i);
            }
        }*/

         if(t > 2.5 )
        {
            for(size_t i = 0; i < 6; i++)
            {
                ni_int_des(i) = base_des(i);
                ni_des(i) = base_dot_des(i);
                //ni_dot_des(i) = base_dot_dot_des(i);
            }
        }
          
        err_6 = ni_int_des - ni_int_curr;
        dot_err_6 = ni_des - ni_curr; 
        //dot_dot_err_6 = ni_dot_des - ni_dot_curr; 
 
        cout << "err"<< endl << err_6 << endl;

       
   
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if(c_switch == 0)
        {
            Fc = Eigen::MatrixXd::Zero(12, 1);
            //Fc_sym = Eigen::MatrixXd::Zero(12, 1);
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

            p = Eigen::MatrixXd::Zero(12, 12);
            p_inv = Eigen::MatrixXd::Zero(12, 12);

            Kp_6 =  Eigen::MatrixXd::Identity(6, 6);
            Kv_6 =  Eigen::MatrixXd::Identity(6, 6);

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

            cout << "gravity" << endl << g_<<endl;

            pos_foot_LF = CalcBodyToBaseCoordinates(*model,q_curr,index_link(0),v1);
            pos_foot_LH = CalcBodyToBaseCoordinates(*model,q_curr,index_link(1),v1);
            pos_foot_RF = CalcBodyToBaseCoordinates(*model,q_curr,index_link(2),v1);
            pos_foot_RH = CalcBodyToBaseCoordinates(*model,q_curr,index_link(3),v1);

            z_avg_L = sqrt((pow(pos_foot_LF(2),2) + pow(pos_foot_LH(2),2))/2);
            z_avg_R = sqrt((pow(pos_foot_RF(2),2) + pow(pos_foot_RH(2),2))/2);

            z_avg = sqrt((pow(pos_foot_LF(2),2) + pow(pos_foot_LH(2),2) + pow(pos_foot_RF(2),2) + pow(pos_foot_RH(2),2))/4);
            
            
            /*if(q_curr(0) > 0.11 && q_curr(0) < 1.5 && t > 3)
            {
                slope = 1;  
            }
            else
            {
                slope = 0;
            }*/

            /*if (slope == 1) 
            {
                g_new_x = g_(0) * cos(theta) + g_(2) * sin(theta);  // con matrice di rotazione intorno a y di theta (inclinazione piano)
                g_new_z = g_(2) * cos(theta) - g_(0) * sin(theta);
                g_(0) = g_new_x; 
                g_(2) = g_new_z;
                cout << "gravity_slopeeeeee" << endl << g_(0)<< endl<< g_(2)<<endl;
            } */
            
            //cout << "gravity_yyyyyyyyyyyyyyyyyyyyyyyyyyyy" << endl << g_<<endl;

            // New dynamic system
            
            M_new = S_ker_trans*M_*S_ker;
            g_new = S_ker_trans*g_;

        
            S_S = (S_ker_trans*S_trans);
            pseudo_inverse(S_S,S_S_inv);

            //Differentiation of S_ker
            if(count == 1 || count == 3)
            {
                S_ker_prec = S_ker;
                count = count + 1;
            
            }
            
            S_ker_dot = df * (S_ker - S_ker_prec);
            S_ker_prec = S_ker;
   

            C_new = (S_ker_trans*M_*S_ker_dot)*ni_curr + S_ker_trans*cor_;


            //Computed torque

            //Computation of the gain matrices

            /*kp1_6 = 1200; 
            kp2_6 = 1200;
            kv1_6 = 250;
            kv2_6 = 250; */

            Kp_6(0,0) = 1200; 
            Kp_6(1,1) = 1200;  
            Kp_6(2,2) = 1200; 
            Kp_6(3,3) = 1200; 
            Kp_6(4,4) = 1200; 
            Kp_6(5,5) = 1200; 

            Kv_6(0,0) = 250;
            Kv_6(1,1) = 250; 
            Kv_6(2,2) = 250; 
            Kv_6(3,3) = 250; 
            Kv_6(4,4) = 250; 
            Kv_6(5,5) = 250;


//kp1 = 20000;  kv1 = 400; 



           /* kp1_6 = 1200; 
            kp2_6 = 1200;
            kv1_6 = 250; 
            kv2_6 = 250;
*/
            /*if(slope == 1)
            {
                kp1_6 = 1000;
                //kp2_6 = 800;
                //kv1_6 = 10;
                //kv2_6 = 10;

            }*/

            
            
            //pos_foot_LF(2) > -0.459
            

           /* for (size_t i=0; i<6; i++)
            {
                for (size_t j=0; j<6; j++)
                {
                    
                        Kp_6(i,j) = kp1_6 * Kp_6(i,j);
                        Kv_6(i,j) = kv1_6 * Kv_6(i,j);
                    
                    
                }
            }*/
            
            /*if(slope == 1)
            {
                Kp_6(0,0) = 2000;
            }*/
            //Kp_6(0,0) = 25000;
            //Kp_6(1,1) = 2000;
            //Kp_6(2,2) = 2000;

            //cout <<"KP"<< endl << Kp_6<< endl;
            
            u =  M_new * ni_dot_des + Kp_6 * err_6 + Kv_6 * dot_err_6 + C_new - g_new;
            
          

            //Control law
            tau_cmd = S_S_inv * u;


           

            
            for (size_t i=0; i<12; i++)
            {
                joint_handle[i].setCommand(tau_cmd[i]);
            }
        
            cout << "tau_cmd (quasi-velocity)"<< endl << tau_cmd <<endl;
            
            //Fc computations
            M_inv = M_.inverse();
            p = (Jc_ *M_inv* Jc_trans);
            p_inv = p.inverse();


            // Differentation of Jc_
            if(count == 2 || count == 4)
            {
                Jc_prec = Jc_;
                count = count + 1;
            }
            
            Jc_dot = df * (Jc_ - Jc_prec);
            Jc_prec = Jc_;

           // Contact force

            Fc = p_inv * (Jc_ * M_inv * (S_trans * tau_cmd - cor_ + g_) + Jc_dot * dot_q_curr);  

            //cout << "Contact force (4 contact points)"<< endl << Fc <<endl;

            //QP problem
           
   /*         Eigen::SparseMatrix<double> H_s(12,12);
            H_s.insert(0,0) = 1;
            H_s.insert(1,1) = 1;
            H_s.insert(2,2) = 1;
            H_s.insert(3,3) = 1;
            H_s.insert(4,4) = 1;
            H_s.insert(5,5) = 1;
            H_s.insert(6,6) = 1;
            H_s.insert(7,7) = 1;
            H_s.insert(8,8) = 1;
            H_s.insert(9,9) = 1;
            H_s.insert(10,10) = 1;
            H_s.insert(11,11) = 1;

            Eigen::Matrix<double,12 ,1> gradient;
            gradient = tau_cmd;

            Eigen::Matrix<double,12 ,12> A;
            A = p_inv * (Jc_ * M_inv * S_trans);

            double mu = 1;
            double mu_e = mu/sqrt(2);

            Eigen::Matrix<double,16 ,12> A_Foot;
            Eigen::Matrix<double,4 ,12> A_LF;
            Eigen::Matrix<double,4 ,12> A_LH;
            Eigen::Matrix<double,4 ,12> A_RF;
            Eigen::Matrix<double,4 ,12> A_RH;
            Eigen::MatrixXd A_id;
            A_id = Eigen::MatrixXd::Identity(12, 12);

            for (size_t i=0; i<12; i++)
            {
                A_LF(0,i) = mu_e * A(2,i) - A (0,i);
                A_LF(1,i) = -mu_e * A(2,i) - A (0,i);
                A_LF(2,i) = mu_e * A(2,i) - A (1,i);
                A_LF(3,i) = -mu_e * A(2,i) - A (1,i);       
            }

            for (size_t i=0; i<12; i++)
            {
                A_LH(0,i) = mu_e * A(5,i) - A (3,i);
                A_LH(1,i) = -mu_e * A(5,i) - A (3,i);
                A_LH(2,i) = mu_e * A(5,i) - A (4,i);
                A_LH(3,i) = -mu_e * A(5,i) - A (4,i);       
            }

            for (size_t i=0; i<12; i++)
            {
                A_RF(0,i) = mu_e * A(8,i) - A (6,i);
                A_RF(1,i) = -mu_e * A(8,i) - A (6,i);
                A_RF(2,i) = mu_e * A(8,i) - A (7,i);
                A_RF(3,i) = -mu_e * A(8,i) - A (7,i);       
            }

            for (size_t i=0; i<12; i++)
            {
                A_RH(0,i) = mu_e * A(11,i) - A (9,i);
                A_RH(1,i) = -mu_e * A(11,i) - A (9,i);
                A_RH(2,i) = mu_e * A(11,i) - A (10,i);
                A_RH(3,i) = -mu_e * A(11,i) - A (10,i);       
            }

            A_Foot << A_LF, A_LH, A_RF, A_RH, A_id;


            cout <<"A_FOOT"<< endl<< A_Foot<< endl;
            
            Eigen::SparseMatrix<double> A_s(28,12);
            for (size_t i=0; i<28; i++)
            {
                for(size_t j=0; j<12; j++)
                {
                    A_s.insert(i,j) = A_Foot(i,j);
                }
            }
            cout <<"A_FOOT_SPARSE"<< endl<< A_s<< endl;
            
            Eigen::Matrix<double,12 ,1> b_;

            b_ = p_inv * (Jc_ * M_inv * (- cor_ + g_) + Jc_dot * dot_q_curr); 

            Eigen::Matrix<double,28 ,1> b;
    
            b(0) =  mu_e * b_(2) - b_(0);
            b(1) =  -mu_e * b_(2) - b_(0);
            b(2) =  mu_e * b_(2) - b_(1);
            b(3) =  -mu_e * b_(2) - b_(1);

            b(4) =  mu_e * b_(5) - b_(3);
            b(5) =  -mu_e * b_(5) - b_(3);
            b(6) =  mu_e * b_(5) - b_(4);
            b(7) =  -mu_e * b_(5) - b_(4);

            b(8) =  mu_e * b_(8) - b_(6);
            b(9) =  -mu_e * b_(8) - b_(6);
            b(10) =  mu_e * b_(8) - b_(7);
            b(11) =  -mu_e * b_(8) - b_(7);

            b(12) =  mu_e * b_(11) - b_(9);
            b(13) =  -mu_e * b_(11) - b_(9);
            b(14) =  mu_e * b_(11) - b_(10);
            b(15) =  -mu_e * b_(11) - b_(10);

            for(size_t j=16; j<28; j++)
            {
                  b(j) = 80;
            }

            cout <<"b"<< endl<< b<< endl;
            
            Eigen::Matrix<double,28 ,1> lowerBound;
            lowerBound = -b;

            Eigen::Matrix<double,28 ,1>  upperBound;
            for(size_t j=0; j<28; j++)
            {
                if(j<16)
                {
                upperBound(j) = 0;
                }
                else
                {
                upperBound(j) = 80;
                }
            }


            OsqpEigen::Solver solver;
            solver.settings()->setVerbosity(true);
            solver.settings()->setAlpha(1.0);
            solver.data()->setNumberOfVariables(12);
            solver.data()->setNumberOfConstraints(18);

            solver.data()->setHessianMatrix(H_s);
            solver.data()->setGradient(gradient);
            solver.data()->setLinearConstraintsMatrix(A_s);
            solver.data()->setLowerBound(lowerBound);
            solver.data()->setUpperBound(upperBound);


*/


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

    
            for(size_t i = 0; i < 6; i++)
            {
                q_base(i) = q_curr(i); 
                //command_dot_q_des = dot_q_curr(i); 
                //command_dot_dot_q_des = dot_dot_q_curr(i); 

            } 

            geometry_msgs::Twist pos_base_pub;  

            pos_base_pub.linear.x = q_base(0);
            pos_base_pub.linear.y = q_base(1);
            pos_base_pub.linear.z = q_base(2);
            pos_base_pub.angular.x = q_base(3);
            pos_base_pub.angular.y = q_base(4);
            pos_base_pub.angular.z = q_base(5);

            this-> pos_base_pub.publish(pos_base_pub);
            
            for(size_t i = 0; i < 12; i++)
            {
                q_init_leg(i) = q_curr(i+6);
                command_q_des(i) = q_curr(i+6);
            } 

            for(size_t i = 0; i < 12; i++)
            {
                q_des_init(i) = dot_q_curr(i+6);
                command_dot_q_des(i) = dot_q_curr(i+6);
            } 

          /*  for(size_t i = 0; i < 12; i++)
            {
                q_dot_des_init(i) = dot_dot_q_curr(i+6);
                command_dot_dot_q_des(i) = dot_dot_q_curr(i+6);
            } */
        
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
        
            
            slope_msg.data = slope;
            this-> slope_pub.publish(slope_msg);

            //cout << "time"<< endl<<t<<endl;
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
              //  q_dot_dot_des(i) = command_dot_dot_q_des(i);
            }


            // Computation of position and velocity errors

            for (size_t i=0; i<12; i++)
            {
                err_12(i)     = 0;
                dot_err_12(i) = 0;
              //  dot_dot_err_12(i) = 0;
            }

            for (size_t i=0; i<12; i++)
            {
                err_12(i)     = q_des(i) - q_curr(i+6);
                dot_err_12(i) = q_dot_des(i) - dot_q_curr(i+6);
               // dot_dot_err_12(i) = q_dot_dot_des(i) - dot_dot_q_curr(i+6);
            }

            //cout << "err pos " << endl << err_12 <<endl;

            //Computation of the gain matrices

            /*kp2 = 15000;
            kv2 = 20;
            kp3 = 15000;
            kv3 = 20;  */  

            kp2 = 15000;
            kv2 = 10;
            kp3 = 15000;
            kv3 = 10; 

           /* if(slope == 1)
            {
                cout << "unoooooo"<< endl;
                kp2 = 6000;
                //kv2 = 15;
                kp3 = 6000;
                //kv3 = 10;
            }*/

            
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
                    else if (i==j && i>2 && i<6)
                    {
                        Kp_12(i,j) = kp2 * Kp_12(i,j);
                        Kv_12(i,j) = kv2 * Kv_12(i,j);
                    }
                    else if(i==j && i>5 && i<9)
                    {
                        Kp_12(i,j) = kp3 * Kp_12(i,j);
                        Kv_12(i,j) = kv3 * Kv_12(i,j);
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

       

        //Plot of position errors
        geometry_msgs::Twist error_pos;
   
        error_pos.linear.x = err_6(0);
        error_pos.linear.y = err_6(1);
        error_pos.linear.z = err_6(2);
        error_pos.angular.x = err_6(3);
        error_pos.angular.y = err_6(4);
        error_pos.angular.z = err_6(5);
        this-> error_pos.publish(error_pos);

        //Plot of velocity errors
        geometry_msgs::Twist error_vel;
   
        error_vel.linear.x = dot_err_6(0);
        error_vel.linear.y = dot_err_6(1);
        error_vel.linear.z = dot_err_6(2);
        error_vel.angular.x = dot_err_6(3);
        error_vel.angular.y = dot_err_6(4);
        error_vel.angular.z = dot_err_6(5);
        this-> error_vel.publish(error_vel);

        //Plot of acceleration errors
        geometry_msgs::Twist error_acc;
   
        error_acc.linear.x = dot_dot_err_6(0);
        error_acc.linear.y = dot_dot_err_6(1);
        error_acc.linear.z = dot_dot_err_6(2);
        error_acc.angular.x = dot_dot_err_6(3);
        error_acc.angular.y = dot_dot_err_6(4);
        error_acc.angular.z = dot_dot_err_6(5);
        this-> error_acc.publish(error_acc);


        //Plot of Fc
        geometry_msgs::Twist force_L;
        geometry_msgs::Twist force_R;

        //LF and LH 
        force_L.linear.x = Fc(0,0);
        force_L.linear.y = Fc(1,0);
        force_L.linear.z = Fc(2,0);
        force_L.angular.x = Fc(3,0);
        force_L.angular.y = Fc(4,0);
        force_L.angular.z = Fc(5,0);

        //RF and RH
        force_R.linear.x = Fc(6,0);
        force_R.linear.y = Fc(7,0);
        force_R.linear.z = Fc(8,0);
        force_R.angular.x = Fc(9,0);
        force_R.angular.y = Fc(10,0);
        force_R.angular.z = Fc(11,0);

        this-> force_L.publish(force_L);
        this-> force_R.publish(force_R);


        //Plot of tau
        geometry_msgs::Twist tau_1_6;
        geometry_msgs::Twist tau_7_12;

        /*tau_1_6.linear.x = tau_cmd(0,0);
        tau_1_6.linear.y = tau_cmd(1,0);
        tau_1_6.linear.z = tau_cmd(2,0);
        tau_1_6.angular.x = tau_cmd(3,0);
        tau_1_6.angular.y = tau_cmd(4,0);
        tau_1_6.angular.z = tau_cmd(5,0); */

       /* tau_7_12.linear.x = tau_cmd(6,0);
        tau_7_12.linear.y = tau_cmd(7,0);
        tau_7_12.linear.z = tau_cmd(8,0); 
        tau_7_12.angular.x = tau_cmd(9,0);
        tau_7_12.angular.y = tau_cmd(10,0);
        tau_7_12.angular.z = tau_cmd(11,0);*/

        
        tau_1_6.linear.x = command_q_des(0);
        tau_1_6.linear.y = command_q_des(1);
        tau_1_6.linear.z = command_q_des(2);
        tau_1_6.angular.x = command_q_des(3);
        tau_1_6.angular.y = command_q_des(4);
        tau_1_6.angular.z = command_q_des(5);

        tau_7_12.linear.x = command_q_des(6);
        tau_7_12.linear.y = command_q_des(7);
        tau_7_12.linear.z = command_q_des(8);
        tau_7_12.angular.x = base_des(2);
        tau_7_12.angular.y = g_(2);
        tau_7_12.angular.z = g_(4);
        
        //tau_7_12.angular.x = contatore;


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


    void floating_base_controller::state_estimator(const gazebo_msgs::LinkStatesConstPtr& msg)

    {
        //mettere 3 se inserisco il piano al posto di 1
        q_temp[0] = msg->pose[1].position.x;
        q_temp[1] = msg->pose[1].position.y;
        q_temp[2] = msg->pose[1].position.z;
        q_temp[3] = msg->pose[1].orientation.x;
        q_temp[4] = msg->pose[1].orientation.y;
        q_temp[5] = msg->pose[1].orientation.z;
        q_temp[6] = msg->pose[1].orientation.w;

        dot_q_temp[0] = msg->twist[1].linear.x;
        dot_q_temp[1] = msg->twist[1].linear.y;
        dot_q_temp[2] = msg->twist[1].linear.z;
        dot_q_temp[3] = msg->twist[1].angular.x;
        dot_q_temp[4] = msg->twist[1].angular.y;
        dot_q_temp[5] = msg->twist[1].angular.z;

    }

    PLUGINLIB_EXPORT_CLASS(my_controller_ns::floating_base_controller, controller_interface::ControllerBase);

 


}

 /*int main(int argc, char** argv)
{
    ros::Rate loop_rate(10);
    

    return 0;
}*/