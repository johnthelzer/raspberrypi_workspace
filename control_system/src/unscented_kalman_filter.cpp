#include "unscented_kalman_filter.h"
#define g 9.81
#define pi 3.14159

Filter::Filter() :nh_()
{
    last_update_time = ros::Time::now();
    T = 0.02; //seconds, since last update. Initialized here to predicted value of 0.02.
    filter_started = false;
    hover = false;
    in_guided_nogps_mode = false;
    mp = 0.294;
    mq = 2.520;
    r = 0.114;
    alpha = 0.001;
    beta = 2;
    kappa = 0;
    M = 6;
    uk_ << 0,0,0;
    I3 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    Zero3 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0;
    k_ << 0, 0, 1;
    Q << 0.001*I3, Zero3, //was 0.0001
        Zero3, 0.001*I3;
    R << 0.025*I3, Zero3,
        Zero3, 0.0001*I3;

    rc_sub = nh_.subscribe("/mavros/state", 1, &Filter::rc_callback, this);
    state_sub = nh_.subscribe("/current_state", 1, &Filter::state_callback, this);
    input_sub = nh_.subscribe("/controller_debug", 1, &Filter::input_callback, this);
    filtered_state_pub = nh_.advertise<freyja_msgs::CurrentState>("/filtered_current_state", 1, true);
    predicted_state_pub = nh_.advertise<freyja_msgs::CurrentState>("/predicted_current_state", 1, true);
    
}
void Filter::rc_callback(const mavros_msgs::State::ConstPtr &msg)
{
    std::string flight_mode = msg -> mode;
    if (flight_mode == "GUIDED_NOGPS") //in GUIDED_NOGPS mode
    {
        in_guided_nogps_mode = true;
    }
    else
    {
        in_guided_nogps_mode = false;
        filter_started = false;
    }
}
void Filter::state_callback(const freyja_msgs::CurrentState::ConstPtr &msg)
{
    //make state vector from updates
    T = (ros::Time::now() - last_update_time).toSec(); //time elapsed since last measurement
    last_update_time = ros::Time::now();
    double p_n, p_e, p_d, v_n, v_e, v_d, roll, pitch, yaw;
    p_n = msg->pn;
    p_e = msg->pe;
    p_d = msg->pd;
    v_n = msg->vn;
    v_e = msg->ve;
    v_d = msg->vd;
    roll = msg->roll;
    pitch = msg->pitch;
    yaw = msg->yaw;

    double w_n, w_e, w_d, q_n, q_e, q_d;
    w_n = msg->wn;
    w_e = msg->we;
    w_d = msg->wd;
    q_n = msg->qn;
    q_e = msg->qe;
    q_d = msg->qd;

    
    y_k_ << w_n, w_e, w_d, q_n, q_e, q_d; //newest measurement
    if (in_guided_nogps_mode && filter_started) //filter has started
    {
        //run predict
        predict();
        filter_started = true;
        //run update- get estimated state, estimated covariance
        update();
        //publish new estimated state 
        double w_n_estimate = estimated_mean(0,0);
        double w_e_estimate = estimated_mean(1,0);
        double w_d_estimate = estimated_mean(2,0);
        double q_n_estimate = estimated_mean(3,0);
        double q_e_estimate = estimated_mean(4,0);
        double q_d_estimate = estimated_mean(5,0);
        freyja_msgs::CurrentState filtered_state_msg;
        filtered_state_msg.pn = p_n;
        filtered_state_msg.pe = p_e;
        filtered_state_msg.pd = p_d;
        filtered_state_msg.vn = v_n;
        filtered_state_msg.ve = v_e;
        filtered_state_msg.vd = v_d;
        filtered_state_msg.qn = q_n_estimate;
        filtered_state_msg.qe = q_e_estimate;
        filtered_state_msg.qd = q_d_estimate;
        filtered_state_msg.wn = w_n_estimate;
        filtered_state_msg.we = w_e_estimate;
        filtered_state_msg.wd = w_d_estimate;
        filtered_state_msg.roll = roll;
        filtered_state_msg.pitch = pitch;
        filtered_state_msg.yaw = yaw;
        filtered_state_msg.dt = T;
        filtered_state_msg.filtered = 1;
        filtered_state_msg.hover = hover;
        filtered_state_msg.header.stamp = ros::Time::now();
        filtered_state_pub.publish(filtered_state_msg);
        //publish predicted state
        double w_n_predicted = predicted_mean(0,0);
        double w_e_predicted = predicted_mean(1,0);
        double w_d_predicted = predicted_mean(2,0);
        double q_n_predicted = predicted_mean(3,0);
        double q_e_predicted = predicted_mean(4,0);
        double q_d_predicted = predicted_mean(5,0);
        freyja_msgs::CurrentState predicted_state_msg;
        predicted_state_msg.pn = p_n;
        predicted_state_msg.pe = p_e;
        predicted_state_msg.pd = p_d;
        predicted_state_msg.vn = v_n;
        predicted_state_msg.ve = v_e;
        predicted_state_msg.vd = v_d;
        predicted_state_msg.qn = q_n_predicted;
        predicted_state_msg.qe = q_e_predicted;
        predicted_state_msg.qd = q_d_predicted;
        predicted_state_msg.wn = w_n_predicted;
        predicted_state_msg.we = w_e_predicted;
        predicted_state_msg.wd = w_d_predicted;
        predicted_state_msg.filtered = 1;
        predicted_state_msg.hover = hover;
        predicted_state_msg.header.stamp = ros::Time::now();
        predicted_state_pub.publish(predicted_state_msg);

    }
    else if(in_guided_nogps_mode && !filter_started) //filter not started yet
    {
        //run initialize
        initialize();
        filter_started = true;
        //run update- get estimated state, estimated covariance
        update();
        //publish new estimated state (and cov?)
        double w_n_estimate = estimated_mean(0,0);
        double w_e_estimate = estimated_mean(1,0);
        double w_d_estimate = estimated_mean(2,0);
        double q_n_estimate = estimated_mean(3,0);
        double q_e_estimate = estimated_mean(4,0);
        double q_d_estimate = estimated_mean(5,0);
        freyja_msgs::CurrentState filtered_state_msg;
        filtered_state_msg.pn = p_n;
        filtered_state_msg.pe = p_e;
        filtered_state_msg.pd = p_d;
        filtered_state_msg.vn = v_n;
        filtered_state_msg.ve = v_e;
        filtered_state_msg.vd = v_d;
        filtered_state_msg.qn = q_n_estimate;
        filtered_state_msg.qe = q_e_estimate;
        filtered_state_msg.qd = q_d_estimate;
        filtered_state_msg.wn = w_n_estimate;
        filtered_state_msg.we = w_e_estimate;
        filtered_state_msg.wd = w_d_estimate;
        filtered_state_msg.roll = roll;
        filtered_state_msg.pitch = pitch;
        filtered_state_msg.yaw = yaw;
        filtered_state_msg.dt = T;
        filtered_state_msg.filtered = 1;
        filtered_state_msg.hover = hover;
        filtered_state_msg.header.stamp = ros::Time::now();
        filtered_state_pub.publish(filtered_state_msg);
        //publish predicted state
        double w_n_predicted = predicted_mean(0,0);
        double w_e_predicted = predicted_mean(1,0);
        double w_d_predicted = predicted_mean(2,0);
        double q_n_predicted = predicted_mean(3,0);
        double q_e_predicted = predicted_mean(4,0);
        double q_d_predicted = predicted_mean(5,0);
        freyja_msgs::CurrentState predicted_state_msg;
        predicted_state_msg.pn = p_n;
        predicted_state_msg.pe = p_e;
        predicted_state_msg.pd = p_d;
        predicted_state_msg.vn = v_n;
        predicted_state_msg.ve = v_e;
        predicted_state_msg.vd = v_d;
        predicted_state_msg.qn = q_n_predicted;
        predicted_state_msg.qe = q_e_predicted;
        predicted_state_msg.qd = q_d_predicted;
        predicted_state_msg.wn = w_n_predicted;
        predicted_state_msg.we = w_e_predicted;
        predicted_state_msg.wd = w_d_predicted;
        predicted_state_msg.filtered = 1;
        predicted_state_msg.hover = hover;
        predicted_state_msg.header.stamp = ros::Time::now();
        predicted_state_pub.publish(predicted_state_msg);
    }
    else //not in guided_nogps
    {
        filter_started = false;
        //publish unfiltered state
        freyja_msgs::CurrentState filtered_state_msg;
        filtered_state_msg.pn = p_n;
        filtered_state_msg.pe = p_e;
        filtered_state_msg.pd = p_d;
        filtered_state_msg.vn = v_n;
        filtered_state_msg.ve = v_e;
        filtered_state_msg.vd = v_d;
        filtered_state_msg.qn = q_n;
        filtered_state_msg.qe = q_e;
        filtered_state_msg.qd = q_d;
        filtered_state_msg.wn = w_n;
        filtered_state_msg.we = w_e;
        filtered_state_msg.wd = w_d;
        filtered_state_msg.roll = roll;
        filtered_state_msg.pitch = pitch;
        filtered_state_msg.yaw = yaw;
        filtered_state_msg.dt = T;
        filtered_state_msg.filtered = 0;
        filtered_state_msg.hover = hover;
        filtered_state_msg.header.stamp = ros::Time::now();
        filtered_state_pub.publish(filtered_state_msg);
        //publish predicted state
        double w_n_predicted = predicted_mean(0,0);
        double w_e_predicted = predicted_mean(1,0);
        double w_d_predicted = predicted_mean(2,0);
        double q_n_predicted = predicted_mean(3,0);
        double q_e_predicted = predicted_mean(4,0);
        double q_d_predicted = predicted_mean(5,0);
        freyja_msgs::CurrentState predicted_state_msg;
        predicted_state_msg.pn = p_n;
        predicted_state_msg.pe = p_e;
        predicted_state_msg.pd = p_d;
        predicted_state_msg.vn = v_n;
        predicted_state_msg.ve = v_e;
        predicted_state_msg.vd = v_d;
        predicted_state_msg.qn = q_n;
        predicted_state_msg.qe = q_e;
        predicted_state_msg.qd = q_d;
        predicted_state_msg.wn = w_n;
        predicted_state_msg.we = w_e;
        predicted_state_msg.wd = w_d;
        predicted_state_msg.filtered = 1;
        predicted_state_msg.hover = hover;
        predicted_state_msg.header.stamp = ros::Time::now();
        predicted_state_pub.publish(predicted_state_msg);
    }
}

void Filter::input_callback(const control_system_msgs::ControllerDebug::ConstPtr &msg)
{
    double un = msg->u_n_sat;
    double ue = msg->u_e_sat;
    double ud = msg->u_d_sat;
    uk_ << un, ue, ud;
    hover = msg->hover;
}  
void Filter::initialize()
{
    //initial guess of state should be w = 0, q = k
    predicted_mean << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    predicted_cov << 0.1, 0, 0, 0, 0, 0,
                    0, 0.1, 0, 0, 0, 0,
                    0, 0, 0.1, 0, 0, 0,
                    0, 0, 0, 0.1, 0, 0,
                    0, 0, 0, 0, 0.1, 0,
                    0, 0, 0, 0, 0, 0.1;
}
void Filter::predict()
{
    //choose sigma points
    Eigen::Matrix<double,6,13> delta_x;
    Eigen::Matrix<double,6,6> P_k_k;
    Eigen::Matrix<double,6,1> x_hat_k_k_;
    x_hat_k_k_ = estimated_mean;
    P_k_k = estimated_cov;
    double c = std::pow(alpha,2)*(M+kappa);
    Eigen::Matrix<double, 6, 6> cP;
    cP = c*P_k_k;
    Eigen::Matrix<double, 6, 6> L; //lower triangular of cP
    L = cP.llt().matrixL();
    Eigen::Matrix<double, 6, 1> zero_vector;
    zero_vector<<0,0,0,0,0,0;
    delta_x.col(0) = zero_vector;
    for (int i = 0; i<M; i++)
    {
        delta_x.col(i+1) = L.col(i);
        delta_x.col(i+M+1) = -1*L.col(i);
    }
    Eigen::Matrix<double,6,13> x_hat_sigma;
    for (int i = 0; i<(2*M+1); i++)
    {
        x_hat_sigma.col(i) = x_hat_k_k_ + delta_x.col(i);
    }
    //predict states by running sigma points through nonlinear function
    Eigen::Matrix<double, 6, 13> x_hat_sigma_updated;
    for (int i = 0; i<(2*M+1); i++)
    {
        double wn = x_hat_sigma(0,i);
        double we = x_hat_sigma(1,i);
        double wd = x_hat_sigma(2,i);
        double qn = x_hat_sigma(3,i);
        double qe = x_hat_sigma(4,i);
        double qd = x_hat_sigma(5,i);
        Eigen::Matrix<double,3,3> q_hat;
        q_hat << 0, -qd, qe,
                qd, 0, -qn,
                -qe, qn, 0;
        Eigen::Matrix<double,3,3> w_hat;
        w_hat << 0, -wd, we,
                wd, 0, -wn,
                -we, wn, 0;
        Eigen::Matrix<double,3,1> q_;
        q_ << qn, qe, qd;
        Eigen::Matrix<double,3,1> w_;
        w_ << wn, we, wd;
        Eigen::Matrix<double,6,6> M_;
        M_ << (mp+mq)*I3, -mp*r*q_hat,
            mp*r*q_hat, mp*std::pow(r,2)*I3;
        Eigen::Matrix<double,6,3> U;
        U << I3, Zero3;
        Eigen::Matrix<double,6,1> F;
        F << (mp+mq)*g*k_ + mp*r*(std::pow(wn,2) + std::pow(we,2) + std::pow(wd,2))*q_,
        mp*r*g*q_hat*k_;
        Eigen::Matrix<double,3,3> Bq;
        Bq = Zero3;
        Eigen::Matrix<double,6,6> M_inv;
        M_inv = M_.inverse();
        Eigen::Matrix<double,6,3> MinvU = M_inv*U;
        Eigen::Matrix<double,3,3> Bw;
        Bw = MinvU.block(3,0,3,3);
        Eigen::Matrix<double,3,1> Aq;
        Aq = w_hat*q_;
        Eigen::Matrix<double,6,1> MinvF;
        MinvF = M_inv*F;
        Eigen::Matrix<double,3,1> Aw;
        Aw = MinvF.block(3,0,3,1);
        x_hat_sigma_updated.col(i) << (w_ + T*Aw + T*Bw*uk_), (q_ + T*Aq + T*Bq*uk_);
    }
    //combine updated sigma points into predicted state
    Eigen::Matrix<double,1,13> Wm;
    Eigen::Matrix<double,6,1> x_hat_k1_k_;
    x_hat_k1_k_ << 0, 0, 0, 0, 0, 0;
    Wm(0,0) = 1-M/(std::pow(alpha,2)*(M+kappa));
    for (int i = 1; i < (2*M+1); i++)
    {
        Wm(0,i) = 1/(2*std::pow(alpha,2)*(M+kappa));
    }
    for (int i = 0; i < (2*M+1); i++)
    {
        x_hat_k1_k_ += Wm(0,i)*x_hat_sigma_updated.col(i);
    }

    //combine updated sigma points into predicted covariance
    Eigen::Matrix<double,1,13> Wc;
    Eigen::Matrix<double,6,6> P_k1_k;
    P_k1_k << Zero3, Zero3, Zero3, Zero3;
    Wc(0,0) = (2-std::pow(alpha,2) + beta) - M/(std::pow(alpha,2)*(M+kappa));
    for (int i = 1; i < (2*M+1); i++)
    {
        Wc(0,i) = 1/(2*std::pow(alpha,2)*(M+kappa));
    }
    Eigen::Matrix<double,6,1> residual_;
    for (int i = 0; i < (2*M+1); i++)
    {
        residual_ = x_hat_sigma_updated.col(i)-x_hat_k1_k_;
        P_k1_k += Wc(0,i)*(residual_*residual_.transpose()) + Q;
    }
    predicted_mean = x_hat_k1_k_;
    predicted_cov = P_k1_k;

}
void Filter::update()
{
    Eigen::Matrix<double, 6, 1> x_hat_k_k1_;
    x_hat_k_k1_ = predicted_mean;
    Eigen::Matrix<double, 6, 6> P_k_k1;
    P_k_k1 = predicted_cov;
    //find sqrt of cP
    double c = std::pow(alpha,2)*(M+kappa);
    Eigen::Matrix<double, 6, 6> cP;
    cP = c*P_k_k1;
    Eigen::Matrix<double, 6, 6> L; //lower triangular of cP
    L = cP.llt().matrixL();
    //create sigma points
    Eigen::Matrix<double, 6, 13> delta_x;
    Eigen::Matrix<double, 6, 13> x_hat_sigma;
    Eigen::Matrix<double, 6, 1> zero_vector;
    zero_vector<<0,0,0,0,0,0;
    delta_x.col(0) = zero_vector;
    for (int i = 0; i<M; i++)
    {
        delta_x.col(i+1) = L.col(i);
        delta_x.col(i+M+1) = -1*L.col(i);
    }
    for (int i = 0; i<(2*M+1); i++)
    {
        x_hat_sigma.col(i) = x_hat_k_k1_ + delta_x.col(i);
    }
    //predict next measurement from sigma points; in this case, just use the same state
    Eigen::Matrix<double, 6, 13> y_hat_sigma;
    y_hat_sigma = x_hat_sigma;
    //create measurement weights, covariance weights
    Eigen::Matrix<double, 1, 13> W_m;
    Eigen::Matrix<double, 1, 13> W_c;
    W_c(0,0) = (2-std::pow(alpha,2) + beta) - M/(std::pow(alpha,2)*(M+kappa));
    W_m(0,0) = 1-M/(std::pow(alpha,2)*(M+kappa));
    for(int i = 1; i<(2*M+1); i++)
    {
        W_m(0,i) = 1/(2*std::pow(alpha,2)*(M+kappa));
        W_c(0,i) = 1/(2*std::pow(alpha,2)*(M+kappa));
    }
    //sum measurement weights
    Eigen::Matrix<double, 6, 1> y_hat_k_;
    y_hat_k_ << 0, 0, 0, 0, 0, 0;
    for (int i = 0; i < (2*M+1); i++)
    {
        y_hat_k_ += W_m(0,i)*y_hat_sigma.col(i);
    }
    //sum covariance weights
    Eigen::Matrix<double, 6, 6> Py;
    Py << Zero3, Zero3, Zero3, Zero3;
    Eigen::Matrix<double, 6, 1> y_residual;
    for (int i = 0; i<(2*M+1); i++)
    {
        y_residual = y_hat_sigma.col(i) - y_hat_k_;
        Py += W_c(0,i)*(y_residual*y_residual.transpose())+R;
    }
    //find cross-covariance
    Eigen::Matrix<double,6,6> Pxy;
    Pxy << Zero3, Zero3, Zero3, Zero3;
    Eigen::Matrix<double,6,1> xx_residual;
    Eigen::Matrix<double,6,1> yy_residual;
    for (int i = 1; i< (2*M+1); i++)
    {
        xx_residual = x_hat_sigma.col(i) - x_hat_k_k1_;
        yy_residual = y_hat_sigma.col(i) - y_hat_k_;
        Pxy = Pxy + 1/(2*std::pow(alpha,2)*(M+kappa))*(xx_residual*yy_residual.transpose());
    }
    //kalman gain
    Eigen::Matrix<double,6,6> K;
    Eigen::Matrix<double,6,6> Py_inv;
    Py_inv = Py.inverse();
    K = Pxy*Py_inv;
    //updated position, cov
    estimated_mean = x_hat_k_k1_ + K*(y_k_ - y_hat_k_);
    estimated_cov = P_k_k1 - K*Py*K.transpose();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Unscented_Kalman_Filter");
    Filter ukf;
    //ros::Rate filter_rate_hz(ukf.filter_rate);
    ros::spin();
    return 0;
}