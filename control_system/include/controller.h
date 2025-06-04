//include all dependencies and libraries
//create class with declared callback functions and ros subscribers/publishers
//-- subscribers: to trajectory (reference state), actual state
//-- publshers: to mavros topic
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include "freyja_msgs/CtrlCommand.h"
#include "freyja_msgs/CurrentState.h"
#include <mavros_msgs/RCIn.h>
#include <eigen3/Eigen/Dense>
#include "control_system_msgs/ReferenceState.h"
#include "control_system_msgs/ControllerDebug.h"




class Controller
{
    //global variables to use in all object functions
    double thrust_max;
    double thrust_min;
    double roll_max_deg;
    double pitch_max_deg;
    //variables only defined once
    Eigen::Matrix<double, 3, 3> lambda_x;
    Eigen::Matrix<double, 3, 3> lambda_v;
    Eigen::Matrix<double, 3, 3> lambda_q;
    Eigen::Matrix<double, 3, 1> gamma_hat;
    Eigen::Matrix<double, 3, 1> gamma_hat_old;
    Eigen::Matrix<double, 3, 1> k_;
    Eigen::Matrix<double, 3, 3> I3;
    Eigen::Matrix<double, 6, 6> I6;
    Eigen::Matrix<double, 3, 3> Zero3;
    Eigen::Matrix<double, 3, 1> abs_s_phi_old_;

    public:
        Controller();
        ros::NodeHandle nh_;
        double controller_rate;
        double mp;
        double mq;
        double total_mass;
        double r;
        double rho;
        double alpha;
        double gamma_hat_max;
        double phi;
        double phi_z;
        double phi_old;
        double phi_z_old;
        double T;
        double mean_ex;
        double mean_ey;
        double mean_ez;
        double stddev_ex;
        double stddev_ey;
        double stddev_ez;
        double E_x;
        double E_y;
        double E_z;
        int controller_iterations;
        bool in_shrinking_law;
        //variables defined each iteration
        Eigen::Matrix<double, 3, 1> x_;
        Eigen::Matrix<double, 3, 1> v_;
        Eigen::Matrix<double, 3, 1> q_;
        Eigen::Matrix<double, 3, 1> w_;
        Eigen::Matrix<double, 3, 1> q_filtered_;
        Eigen::Matrix<double, 3, 1> w_filtered_;
        Eigen::Matrix<double, 3, 1> x_ref_;
        Eigen::Matrix<double, 3, 1> v_ref_;
        Eigen::Matrix<double, 3, 1> x_ref_future_;
        Eigen::Matrix<double, 3, 1> v_ref_future_;
        Eigen::Matrix<double, 3, 3> q_hat;
        Eigen::Matrix<double, 3, 3> w_hat;
        Eigen::Matrix<double, 6, 6> M;
        Eigen::Matrix<double, 6, 6> M_inv;
        Eigen::Matrix<double, 6, 6> M_solved_inv;
        Eigen::Matrix<double, 6, 1> F;
        Eigen::Matrix<double, 6, 3> U;
        Eigen::Matrix<double, 9, 1> A;
        Eigen::Matrix<double, 6, 1> A_top6;
        Eigen::Matrix<double, 6, 1> A_top6_solved;
        Eigen::Matrix<double, 6, 3> B;
        Eigen::Matrix<double, 3, 1> Av;
        Eigen::Matrix<double, 3, 1> Aw;
        Eigen::Matrix<double, 3, 1> Aq;
        Eigen::Matrix<double, 3, 3> Bx;
        Eigen::Matrix<double, 3, 3> Bw;
        Eigen::Matrix<double, 3, 1> S_;
        Eigen::Matrix<double, 3, 1> s_phi_;
        Eigen::Matrix<double, 3, 1> abs_s_phi_;
        Eigen::Matrix<double, 3, 3> sat_s;
        Eigen::Matrix<double, 3, 1> uas_;
        Eigen::Matrix<double, 3, 1> uas_old_;
        Eigen::Matrix<double, 3, 1> u_;
        Eigen::Matrix<double, 3, 25> S_array;
        Eigen::Matrix<double, 3, 25> Uas_array;
        //defining values each time for matrices (easier than typing everything out symbolically)
        void define_values();
        //subscribers and their callbacks
        ros::Subscriber rc_sub;
        void rc_callback(const mavros_msgs::RCIn::ConstPtr &);
        ros::Subscriber reference_sub;
        void reference_callback(const control_system_msgs::ReferenceState::ConstPtr &);
        ros::Subscriber state_sub;
        void state_callback(const freyja_msgs::CurrentState::ConstPtr  &);
        ros::Subscriber filtered_state_sub;
        void filtered_state_callback(const freyja_msgs::CurrentState::ConstPtr &);
        //publisher
        ros::Publisher input_pub;
        ros::Publisher controller_debug_pub;
        //other functions
        void apply_thrust_saturation(double& tx_des, double& ty_des, double& tz_des, double& tx_sat, double& ty_sat, double& tz_sat);
        void apply_angle_saturation(double& roll_sat, double& pitch_sat, double& tx_sat, double& ty_sat, double& tz_sat);
        void compute_input(double& tn, double& te, double& td);
        //void track_sliding_surfaces();
        void publish_to_mavros(double roll, double pitch, double yaw, double thrust_mag);
        void control_loop_function();

};