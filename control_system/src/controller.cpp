#include "controller.h"
#define g 9.81
#define pi 3.14159
bool hover = true;
bool delayed_one_period = false;
bool gains_have_been_reset = false;
//reference variables, initialized to 0
double pn_ref = 0; double pe_ref = 0; double pd_ref = 0; double vn_ref = 0; double ve_ref = 0; double vd_ref = 0; double pn_ref_future = 0; double pe_ref_future = 0; double pd_ref_future = 0; double vn_ref_future = 0; double ve_ref_future = 0; double vd_ref_future = 0; double yaw_ref = 90/180*pi;
//current state variables, initialized to 0
double pn_ = 0; double pe_ = 0; double pd_ = 0; double vn_ = 0; double ve_ = 0; double vd_ = 0; double qn_ = 0.0; double qe_ = 1.0; double qd_ = 0; double wn_ = 1; double we_ = 1; double wd_ = 1;
double qn_filtered = 0; double qe_filtered = 0; double qd_filtered = 1.0;
double wn_filtered = 0; double we_filtered = 0; double wd_filtered = 0;
double real_roll = 0.0;
double real_pitch = 0.0;
double real_yaw = 0.0;
int filtered = 0;

double channel6_pwm = 1500;
using namespace std;



Controller::Controller() : nh_()
{
    controller_rate = 50; //50;
    mp = 0; //0.294;
    mq = 2.312;//2.520;
    r = 0.114;
    total_mass = mp + mq;
    thrust_max = 5.0*g;
    thrust_min = 0.1*g;
    roll_max_deg = 30;
    pitch_max_deg = 30;
    //publishers and subscribers
    input_pub = nh_.advertise <freyja_msgs::CtrlCommand>("/rpyt_command", 1, true);
    controller_debug_pub = nh_.advertise<control_system::ControllerDebug>("/controller_debug", 1, true);
    rc_sub = nh_.subscribe("/mavros/rc/in", 1, &Controller::rc_callback, this);
    reference_sub = nh_.subscribe("/reference_state", 1, &Controller::reference_callback, this);
    state_sub = nh_.subscribe("/current_state", 1, &Controller::state_callback, this);
    filtered_state_sub = nh_.subscribe("/filtered_current_state", 1, &Controller::filtered_state_callback, this);
    //defining gains and such
    lambda_x << 10, 0, 0,
                0, 10, 0,
                0, 0, 100; //previously diag [10,10,100]
    lambda_v << 10, 0, 0,
                0, 10, 0,
                0, 0, 10; //previously 10I
    lambda_q << -40, 0, 0,
                0, -40, 0,
                0, 0, -40; //previously -40I
    rho = 0.1; //previously 0.1
    alpha = 0.9; //previously 0.9
    phi = 3;//6; //previously 3, then 6
    phi_z = 6;//12; //previously 6, then 12
    phi_old = 0;
    phi_z_old = 0;
    T = 1/controller_rate;
    gamma_hat << 0, 0, 0;
    gamma_hat_old << 0, 0, 0;
    k_ << 0, 0, 1;
    I3 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    I6 << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    Zero3 << 0, 0, 0,
            0, 0, 0,
            0, 0, 0;
    abs_s_phi_old_ << 0, 0, 0;
    S_ << 0, 0, 0;
}

void Controller::define_values()//define values dynamically in terms of global variables for each iteration
{
    x_<<pn_, pe_, pd_;
    v_<<vn_, ve_, vd_;
    q_<<qn_, qe_, qd_;
    /*commented out for drone-only test
    q_filtered_ << qn_filtered, qe_filtered, qd_filtered;
    w_<<wn_, we_, wd_;
    w_filtered_ << wn_filtered, we_filtered, wd_filtered;*/
    x_ref_<<pn_ref, pe_ref, pd_ref;
    v_ref_<<vn_ref, ve_ref, vd_ref;
    x_ref_future_<<pn_ref_future, pe_ref_future, pd_ref_future;
    v_ref_future_<<vn_ref_future, ve_ref_future, vd_ref_future;
    //unfiltered
    /*q_hat<<0, -qd_, qe_,
        qd_, 0, -qn_,
        -qe_, qn_, 0;
    w_hat<<0, -wd_, we_,
        wd_, 0, -wn_,
        -we_, wn_, 0;*/
    //filtered
    /*commented out for drone-only test
    q_hat << 0, -qd_filtered, qe_filtered,
            qd_filtered, 0, -qn_filtered,
            -qe_filtered, qn_filtered, 0;
    w_hat << 0, -wd_filtered, we_filtered,
            wd_filtered, 0, -wn_filtered,
            -we_filtered, wn_filtered, 0;
    M<<(mp+mq)*I3, -mp*r*q_hat,
        mp*r*q_hat, mp*pow(r, 2)*I3;
    //F<<(mp+mq)*g*k_ + mp*r*(pow(wn_,2) + pow(we_,2) + pow(wd_,2))*q_,
        //mp*r*g*q_hat*k_;
    F<<(mp+mq)*g*k_ + mp*r*(pow(wn_filtered,2) + pow(we_filtered,2) + pow(wd_filtered,2))*q_filtered_,
        mp*r*g*q_hat*k_;
    U<<I3, Zero3;
    A_top6 = M.inverse()*F;
    //A<<A_top6, w_hat*q_;
    A << A_top6, w_hat*q_filtered_;
    B<<M.inverse()*U;
    Av = A.block(0,0,3,1);//block matrix of size 3x1, starting at indices 0,0
    Aw = A.block(3,0,3,1);
    Aq = A.block(6,0,3,1);
    Bx = B.block(0,0,3,3);
    Bw = B.block(3,0,3,3);*/
    /*added for drone-only test:*/
    Av = g*k_;
    Bx = 1/mq*I3;

}
//sgn function
double sgn(double x)
{
    if (x < 0)
    {
        return -1.0;
    }
    else if(x > 0)
    {
        return 1.0;
    }
    else
    {
        return 0.0;
    }
}
//saturation function
double sat(double x)
{
    if ( fabs(x) <= 1)
    {
        return x;
    }
    else
    {
        return sgn(x);
    }
}
//yaw error calculation
double calc_yaw_error(const double &a, const double &b)
{
    double yd = fmod(a - b + pi, 2*pi);
    yd = yd < 0 ? yd+=2*pi : yd;
    return yd - pi;
}
//callback function implementation
void Controller::rc_callback(const mavros_msgs::RCIn::ConstPtr &msg)
{
    channel6_pwm = msg->channels[5];
    if(channel6_pwm > 1750)
    {
        hover = false;
        gains_have_been_reset = false;
    }
    else
    {
        hover = true;
        delayed_one_period = false;
        if (!gains_have_been_reset) //this only runs once, right when hover turns true
        {
            gamma_hat << 0, 0, 0;
            gamma_hat_old << 0, 0, 0;
            gains_have_been_reset = true;
        }
    }
}
void Controller::reference_callback(const control_system::ReferenceState::ConstPtr &msg)
{
    pn_ref = msg -> reference_data[0];
    pe_ref = msg -> reference_data[1];
    pd_ref = msg -> reference_data[2];
    vn_ref = msg -> reference_data[3];
    ve_ref = msg -> reference_data[4];
    vd_ref = msg -> reference_data[5];
    pn_ref_future = msg -> reference_data[6];
    pe_ref_future = msg -> reference_data[7];
    pd_ref_future = msg -> reference_data[8];
    vn_ref_future = msg -> reference_data[9];
    ve_ref_future = msg -> reference_data[10];
    vd_ref_future = msg -> reference_data[11];
    yaw_ref = msg -> reference_data[12];
}
void Controller::state_callback(const freyja_msgs::CurrentState::ConstPtr  &msg)
{
    pn_ = msg -> pn;
    pe_ = msg -> pe;
    pd_ = msg -> pd;
    vn_ = msg -> vn;
    ve_ = msg -> ve;
    vd_ = msg -> vd;
    qn_ = msg -> qn;
    qe_ = msg -> qe;
    qd_ = msg -> qd;
    wn_ = msg -> wn;
    we_ = msg -> we;
    wd_ = msg -> wd;
    real_roll = msg->roll;
    real_pitch = msg->pitch;
    real_yaw = msg -> yaw;
}
void Controller::filtered_state_callback(const freyja_msgs::CurrentState::ConstPtr &msg)
{
    qn_filtered = msg -> qn;
    qe_filtered = msg -> qe;
    qd_filtered = msg -> qd;
    wn_filtered = msg -> wn;
    we_filtered = msg -> we;
    wd_filtered = msg -> wd;
    filtered = msg -> filtered;
}
//other functions implementation:
void Controller::compute_input(double& tn, double& te, double& td)
{
    //compute sliding surfaces
    //S_ = lambda_x*(x_-x_ref_) + lambda_v*(v_-v_ref_) + lambda_q*(q_- k_);
    S_ = lambda_x*(x_-x_ref_) + lambda_v*(v_-v_ref_);//commented out for drone-only test// + lambda_q*(q_filtered_- k_);
    cout<<"Computed sliding surfaces";
    Eigen::Matrix<double, 3, 1> sr_ = lambda_x*(x_ + T*v_ - x_ref_future_) + lambda_v*(v_ + T*Av - v_ref_future_);//commented out for drone-only test:// + lambda_q*(q_ + T*Aq - k_);
    //Eigen::Matrix<double, 3, 1> sr_ = lambda_x*(x_ + T*v_ - x_ref_future_) + lambda_v*(v_ + T*Av - v_ref_future_) + lambda_q*(q_filtered_ + T*Aq - k_);
    Eigen::Matrix<double, 3, 3> Bu = lambda_v*Bx;
    double S1 = S_(0,0);
    double S2 = S_(1,0);
    double S3 = S_(2,0);
    
    s_phi_ << S1 - (phi*sat(S1/phi)),
            S2 - (phi*sat(S2/phi)),
            S3 - (phi_z*sat(S3/phi_z)); //this is a really cool lil function that becomes 0 when S is inside the BL phi!

    abs_s_phi_ << fabs(s_phi_(0,0)),
                fabs(s_phi_(1,0)),
                fabs(s_phi_(2,0));
    
    sat_s << sat(S1/phi), 0, 0,
            0, sat(S2/phi), 0,
            0, 0, sat(S3/phi_z);
    //adaptive gain law

    gamma_hat = gamma_hat_old + (T*alpha/rho)*abs_s_phi_old_;
    //gamma_hat = gamma_hat_old + (T*alpha/rho)*abs_s_phi_;
    //calculate system input
    uas_ = -1*alpha*S_ + T*sat_s*gamma_hat;

    u_ = (-1/T)*(Bu.inverse()*(sr_ + uas_));
    //output of function: change values of tn, te, td.
    tn = u_(0,0);
    te = u_(1,0);
    td = u_(2,0);
    //publish debug values
    //update previous values for next iteration
    gamma_hat_old = gamma_hat;
    abs_s_phi_old_ = abs_s_phi_;

}

void Controller::apply_thrust_saturation(double& tx_des, double& ty_des, double& tz_des, double& tx_sat, double& ty_sat, double& tz_sat)
{
    //dumb saturation
    /*
    if(tz_des > -1*thrust_min)
    {
        tz_des = -1*thrust_min;
    }
    double t_norm =  sqrt(pow(tx_des, 2) +  pow(ty_des, 2) +  pow(tz_des, 2));
    if (t_norm > thrust_max)
    {
        double scaling_factor = t_norm/thrust_max;
        tz_sat = tz_des/scaling_factor;
        tx_sat = tx_des/scaling_factor;
        ty_sat = ty_des/scaling_factor;
    }
    else
    {
        tx_sat = tx_des;
        ty_sat = ty_des;
        tz_sat = tz_des;
    }*/
     //smart saturation

    //tz cannot be positive (down) and must be greater than minimum thrust
    if (tz_des > -1*thrust_min)
    {
        tz_des = -1*thrust_min;
    }
    
    double t_squared =  pow(tx_des, 2) +  pow(ty_des, 2) +  pow(tz_des, 2);
    //total thrust cannot be greater than maximum thrust
    if (t_squared >  pow(thrust_max, 2)) //total thrust saturated
    {
        //deal with z thrust first
        if ( fabs(tz_des) >= thrust_max) //z thrust saturated, cap it
        {
            tz_sat = -1*thrust_max;
        }
        else //z thrust not saturated, allow it
        {
            tz_sat = tz_des;
        }
        //allocate leftover thrust to x and y directions
        double t_residual =  pow(thrust_max, 2) -  pow(tz_sat, 2); //t_residual = tx^2 + ty^2
        if (t_residual == 0) //no thrust left for x and y OR both x and y zero
        {
            tx_sat = 0;
            ty_sat = 0;
        }
        else //nonzero residual- some thrust left for x and y
        {
            if (tx_des == 0) //residual is only ty
            {
                tx_sat = 0;
                ty_sat =  sqrt(t_residual)*sgn(ty_des);
            }
            else if(ty_des == 0) //residual is only tx
            {
                ty_sat = 0;
                tx_sat =  sqrt(t_residual)*sgn(tx_des);
            }
            else //tx and ty nonzero
            {
                tx_sat = sgn(tx_des)* sqrt(t_residual/(1 + ( pow(ty_des, 2)/ pow(tx_des, 2))));
                ty_sat = tx_sat*(ty_des/tx_des);
            }

        }
    }
    else //total thrust not saturated- allow it
    {
        tx_sat = tx_des;
        ty_sat = ty_des;
        tz_sat = tz_des;
    }
    

}

void Controller::apply_angle_saturation(double& roll_sat, double& pitch_sat, double& tx_sat, double& ty_sat, double& tz_sat)
{
    double roll_max_rad = pi*roll_max_deg/180;
    double pitch_max_rad = pi*pitch_max_deg/180;
    double t_squared =  pow(tx_sat, 2) +  pow(ty_sat, 2) +  pow(tz_sat, 2);
    double t_norm =  sqrt(t_squared);
    //desired values (may not be achievable)
    double roll_des =  asin(ty_sat/t_norm);
    double pitch_des =  atan(tx_sat/tz_sat);
    if( fabs(roll_des) <= roll_max_rad &&  fabs(pitch_des) <= pitch_max_rad)//neither angle saturated
    {
        roll_sat = roll_des;
        pitch_sat = pitch_des;
    }
    while(fabs(roll_des) > roll_max_rad || fabs(pitch_des) > pitch_max_rad)
    {
        if( fabs(roll_des) > roll_max_rad &&  fabs(pitch_des) <= pitch_max_rad)//roll saturated, pitch fine
        {
            double Rxy = tx_sat/ty_sat; //original tx/ty ratio
            //cap roll
            roll_sat = roll_max_rad*sgn(roll_des);
            //adjust pitch, tx, ty so that tx/ty is same ratio
            ty_sat = sgn(ty_sat)*sqrt(pow(sin(roll_sat)*tz_sat,2)/(1-pow(sin(roll_sat),2)*(1+pow(Rxy,2))));
            tx_sat = ty_sat*Rxy;
            pitch_sat = atan(tx_sat/tz_sat);
        }
        else if(fabs(roll_des) <= roll_max_rad &&  fabs(pitch_des) > pitch_max_rad)//pitch saturated, roll fine
        {
            double Ryx = ty_sat/tx_sat; //original ty/tx ratio
            //cap pitch
            pitch_sat = pitch_max_rad*sgn(pitch_des);
            //adjust roll, ty, tx
            tx_sat = tan(pitch_sat)*tz_sat;
            ty_sat = tx_sat*Ryx;
            roll_sat = asin(ty_sat/sqrt(pow(tx_sat,2) + pow(ty_sat, 2) + pow(tz_sat, 2)));
        }
        else//both angles saturated
        {
            //find bigger angle, scale it down
            if (fabs(roll_des) >= fabs(pitch_des))//roll larger than pitch
            {
                double Rxy = tx_sat/ty_sat; //original tx/ty ratio
                //cap roll
                roll_sat = roll_max_rad*sgn(roll_des);
                //adjust pitch, tx, ty so that tx/ty is same ratio
                ty_sat = sgn(ty_sat)*sqrt(pow(sin(roll_sat)*tz_sat,2)/(1-pow(sin(roll_sat),2)*(1+pow(Rxy,2))));
                tx_sat = ty_sat*Rxy;
                pitch_sat = atan(tx_sat/tz_sat);
            }
            else //pitch larger than roll
            {
                double Ryx = ty_sat/tx_sat; //original ty/tx ratio
                //cap pitch
                pitch_sat = pitch_max_rad*sgn(pitch_des);
                //adjust roll, ty, tx
                tx_sat = tan(pitch_sat)*tz_sat;
                ty_sat = tx_sat*Ryx;
                roll_sat = asin(ty_sat/sqrt(pow(tx_sat,2) + pow(ty_sat, 2) + pow(tz_sat, 2)));
            }
        }
        roll_des = roll_sat;
        pitch_des = pitch_sat;
    }
}

void Controller::publish_to_mavros(double roll, double pitch, double yaw, double thrust_mag)
{
    freyja_msgs::CtrlCommand input_msg;
    input_msg.roll = roll;
    input_msg.pitch = pitch;
    input_msg.yaw = -1*yaw; //important to have the -1 there! makes error ref-actual
    input_msg.thrust = thrust_mag;
    input_msg.ctrl_mode = 0b00001111;
    input_pub.publish(input_msg);
}

/*all the control logic and function calls are encapsulated in this control_loop_function to increase simplicity and readability of the main function*/
void Controller::control_loop_function()
{
    //declare sliding mode thrust variables
    double tn_sm = 0;
    double te_sm = 0;
    double td_sm = 0;
    //declare hover thrust variables
    double tn_hover = 0.0;
    double te_hover = 0.0;
    double td_hover = -1*total_mass*g;
    //declare stand-in variables for saturation
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double tn = 0.0;
    double te = 0.0;
    double td = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    //calculate yaw error for both hover and non-hover (this is the way Freyja does it)
    double yaw_tgt_hover = calc_yaw_error(real_yaw, 90/180.0*pi);
    double yaw_tgt = calc_yaw_error(real_yaw, yaw_ref);

    if(hover) //use hover thrust
    {
        //define matrices and all variables in terms of current state and reference variables
        define_values();
        //The thrusts we calculate are in NED; we need them in XYZ (body-frame after yaw) for saturation.
        double tx_hover = tn_hover*cos(real_yaw) + te_hover*sin(real_yaw);
        double ty_hover = -1*tn_hover*sin(real_yaw) + te_hover*cos(real_yaw);
        double tz_hover = td_hover;
        apply_thrust_saturation(tx_hover, ty_hover, tz_hover, tx, ty, tz);
        apply_angle_saturation(roll, pitch, tx, ty, tz);
        //tx, ty, tz are saturated thrust but in the body frame post-yaw (NED+yaw)
        //tn, te, td are saturated thrust in the NED frame.
        tn = tx*cos(real_yaw) - ty*sin(real_yaw);
        te = tx*sin(real_yaw) + ty*cos(real_yaw);
        td = tz;
        double thrust_magnitude = sqrt(pow(tx, 2) + pow(ty, 2) + pow(tz, 2));
        publish_to_mavros(roll, pitch, yaw_tgt_hover, thrust_magnitude);
    }
    else //use controller-calculated thrusts
    {
        
        if(!delayed_one_period) //delay and hover for one period before starting controller to allow new reference
        {
            define_values();
            double tx_hover = tn_hover*cos(real_yaw) + te_hover*sin(real_yaw);
            double ty_hover = -1*tn_hover*sin(real_yaw) + te_hover*cos(real_yaw);
            double tz_hover = td_hover;
            apply_thrust_saturation(tx_hover, ty_hover, tz_hover, tx, ty, tz);
            apply_angle_saturation(roll, pitch, tx, ty, tz);
            //tx, ty, tz are saturated thrust but in the body frame post-yaw (NED+yaw)
            //tn, te, td are saturated thrust in the NED frame.
            tn = tx*cos(real_yaw) - ty*sin(real_yaw);
            te = tx*sin(real_yaw) + ty*cos(real_yaw);
            td = tz;
            double thrust_magnitude = sqrt(pow(tx, 2) + pow(ty, 2) + pow(tz, 2));
            publish_to_mavros(roll, pitch, yaw_tgt_hover, thrust_magnitude);
            delayed_one_period = true;
        }
        else
        {
            //compute input function here: outputs tn, te, td
            //define matrices and all variables in terms of current state and reference variables
            define_values();
            compute_input(tn_sm, te_sm, td_sm);
            //transform to body fram with yaw rotation
            double tx_sm = tn_sm*cos(real_yaw) + te_sm*sin(real_yaw);
            double ty_sm = -1*tn_sm*sin(real_yaw) + te_sm*cos(real_yaw);
            double tz_sm = td_sm;
            apply_thrust_saturation(tx_sm, ty_sm, tz_sm, tx, ty, tz);
            apply_angle_saturation(roll, pitch, tx, ty, tz);
            //tx, ty, tz are saturated thrust but in the body frame post-yaw (NED+yaw)
            //tn, te, td are saturated thrust in the NED frame.
            tn = tx*cos(real_yaw) - ty*sin(real_yaw);
            te = tx*sin(real_yaw) + ty*cos(real_yaw);
            td = tz;
            double thrust_magnitude = sqrt(pow(tx, 2) + pow(ty, 2) + pow(tz, 2));
            publish_to_mavros(roll, pitch, yaw_tgt, thrust_magnitude);
        }        
    }
    //publish debug info
    control_system::ControllerDebug debug_msg;
    debug_msg.T = T;
    debug_msg.alpha = alpha;
    debug_msg.rho = rho;
    debug_msg.x_[0] = x_(0,0);
    debug_msg.x_[1] = x_(1,0);
    debug_msg.x_[2] = x_(2,0);
    debug_msg.v_[0] = v_(0,0);
    debug_msg.v_[1] = v_(1,0);
    debug_msg.v_[2] = v_(2,0);
    /*commented out for drone-only test:
    debug_msg.q_[0] = q_(0,0);
    debug_msg.q_[1] = q_(1,0);
    debug_msg.q_[2] = q_(2,0);
    debug_msg.w_[0] = w_(0,0);
    debug_msg.w_[1] = w_(1,0);
    debug_msg.w_[2] = w_(2,0);
    debug_msg.q_filtered_[0] = qn_filtered;
    debug_msg.q_filtered_[1] = qe_filtered;
    debug_msg.q_filtered_[2] = qd_filtered;
    debug_msg.w_filtered_[0] = wn_filtered;
    debug_msg.w_filtered_[1] = we_filtered;
    debug_msg.w_filtered_[2] = wd_filtered;*/
    debug_msg.x_ref_[0] = x_ref_(0,0);
    debug_msg.x_ref_[1] = x_ref_(1,0);
    debug_msg.x_ref_[2] = x_ref_(2,0);
    debug_msg.v_ref_[0] = v_ref_(0,0);
    debug_msg.v_ref_[1] = v_ref_(1,0);
    debug_msg.v_ref_[2] = v_ref_(2,0);
    //raw input (NED)
    debug_msg.u_n = tn_sm;
    debug_msg.u_e = te_sm;
    debug_msg.u_d = td_sm;
    //saturated input (NED+yaw)
    debug_msg.u_x_sat = tx;
    debug_msg.u_y_sat = ty;
    debug_msg.u_z_sat = tz;
    //saturated input (NED)
    debug_msg.u_n_sat = tn;
    debug_msg.u_e_sat = te;
    debug_msg.u_d_sat = td;
    //saturated roll, pitch angles
    debug_msg.roll_sat = roll*180/pi;
    debug_msg.pitch_sat = pitch*180/pi;
    debug_msg.s_n = S_(0,0);
    debug_msg.s_e = S_(1,0);
    debug_msg.s_d = S_(2,0);
    debug_msg.gamma_hat_n = gamma_hat(0,0);
    debug_msg.gamma_hat_e = gamma_hat(1,0);
    debug_msg.gamma_hat_d = gamma_hat(2,0);
    debug_msg.uas_n = uas_(0,0);
    debug_msg.uas_e = uas_(1,0);
    debug_msg.uas_d = uas_(2,0);
    debug_msg.s1_phi = s_phi_(0,0);
    debug_msg.s2_phi = s_phi_(1,0);
    debug_msg.s3_phi = s_phi_(2,0);
    debug_msg.abs_s1_phi = abs_s_phi_(0,0);
    debug_msg.abs_s2_phi = abs_s_phi_(1,0);
    debug_msg.abs_s3_phi = abs_s_phi_(2,0);
    debug_msg.hover = hover;
    debug_msg.roll_actual = real_roll*180/pi;
    debug_msg.pitch_actual = real_pitch*180/pi;
    debug_msg.yaw_actual = real_yaw;
   //commented out for drone-only test:// debug_msg.filtered = filtered;
    controller_debug_pub.publish(debug_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    Controller DASMC;
    ros::Rate controller_rate_hz(DASMC.controller_rate);
    while(DASMC.nh_.ok())
    {
        ros::spinOnce();
        DASMC.control_loop_function();
        controller_rate_hz.sleep();
    }
    return 0;
}