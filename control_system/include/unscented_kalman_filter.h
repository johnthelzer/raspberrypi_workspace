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
#include <mavros_msgs/State.h>
#include "control_system/ControllerDebug.h"
#include <eigen3/Eigen/Dense>
#include <string>
#include <iostream>


class Filter
{
    double alpha;
    double beta;
    double kappa;
    double M;
    double mp;
    double mq;
    double r;
    double T;
    double filter_rate;
    bool filter_started;
    bool in_guided_nogps_mode;

    Eigen::Matrix<double, 3, 3> I3;
    Eigen::Matrix<double, 3, 3> Zero3;
    Eigen::Matrix<double, 3, 1> k_;
    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 6, 6> R;
    Eigen::Matrix<double, 6, 1> predicted_mean;
    Eigen::Matrix<double, 6, 6> predicted_cov;
    Eigen::Matrix<double, 6, 1> estimated_mean;
    Eigen::Matrix<double, 6, 6> estimated_cov;
    Eigen::Matrix<double, 3, 1> uk_;
    Eigen::Matrix<double, 6, 1> y_k_;

    ros::Time last_update_time;
    
    public:
        Filter();
        ros::NodeHandle nh_;
    
        //subscribers and their callbacks
        ros::Subscriber rc_sub;
        void rc_callback(const mavros_msgs::State::ConstPtr &);
        ros::Subscriber state_sub;
        void state_callback(const freyja_msgs::CurrentState::ConstPtr  &);
        ros::Subscriber input_sub;
        void input_callback(const control_system::ControllerDebug::ConstPtr &);

        //publishers
        ros::Publisher filtered_state_pub;

        //other functions
        void initialize();
        void predict();
        void update();

};