/* This is the trajectory generator node. The trajectory can be modified as needed; the main parameters to set are the 
stop time, the step (number of seconds per side), and the behavior in each segment (in the interpolate function). 
As written, the trajectory is a relative trajectory, meaning the "zero position" to start the trajectory is the 
current state when the trajectory is initialized. The trajectory is initialized by setting "hover" to false; the
way I've done that is with RC input on channel 6, but you can definitely write another function or node to do that too!

The bulk of the computation here is in the "interpolate" and "compute_trajectory" functions. The trajectory is divided into "segments",
which can be as long or short as you want them. The time, position, and velocity endpoints for each segment are established in the
t_n, x_n, y_n, z_n arrays (I didn't include velocity arrays because I assumed zero velocity at each point, but velocity arrays can be added
the same way for non-zero velocity endpoints). You can write the arrays manually or use a modulus function like I did to automatically compute
the needed values for each step in the arrays. Within each segment, the interpolation function uses third order polynomial interpolation, 
which means that for each component of position and velocity between the endpoints, the equations are:

x(t) = a1*t^3 + a2*t^2 + a3*t + a4
v(t) = dx/dt = 3*a1*t^2 + 2*a2*t + a3

Thus, to calculate the positions and velocities within each segment, we need to find the constants a1, a2, a3, and a4. This can be 
solved by setting up the 4 equations for x0(t0), xf(tf), v0(t0), vf(tf), then solving for the vector [a1, a2, a3, a4]. Again, note that
x, y, and z are solved for separately here. 

Once the constants are found for each segment of the trajectory and stored in arrays, the "compute_trajectory" function calculates
the trajectory by using the above equations for x(t) and y(t) with the constants calculated for each segment. And that's it! Feel free 
to write a higher-order trajectory like 5th order that takes acceleration endpoints too- I didn't find much benefit to a 5th order trajectory
over a 3rd order trajectory, but if your application requires finer trajectory control, go for it!

Finally, since the sliding mode controller I designed needs the current reference x_k and the future reference x_k+1, the future reference is
computed by shifting the time one step ahead and calculating the current reference with that time.

*************************
TO DO: add yaw trajectory

-JH

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/RCIn.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <sstream>
#include <iostream>
#include "freyja_msgs/CurrentState.h"
#include "control_system_msgs/ReferenceState.h"

#include <eigen3/Eigen/Dense>
#define pi 3.14159
using namespace Eigen;
using namespace std;

const double T = 0.02;
bool hover = true;
const ros::Duration T_rostime(T);
bool start_time_set = false;
ros::Time start_time;// = ros::Time::now();

//creating segments and interpolation constants- assume starting at t=0
const int endtime = 20; //trajectory duration, seconds
const int step = 5; //seconds per side
const int num_of_steps = endtime/step + 1;
double t_n[num_of_steps] = {};
double x_n[num_of_steps] = {};
double y_n[num_of_steps] = {};
double z_n[num_of_steps] = {};
double yaw_n[num_of_steps] = {};

double a1_x[num_of_steps-1] = {};
double a2_x[num_of_steps-1] = {};
double a3_x[num_of_steps-1] = {};
double a4_x[num_of_steps-1] = {};
double a1_y[num_of_steps-1] = {};
double a2_y[num_of_steps-1] = {};
double a3_y[num_of_steps-1] = {};
double a4_y[num_of_steps-1] = {};
double a1_z[num_of_steps-1] = {};
double a2_z[num_of_steps-1] = {};
double a3_z[num_of_steps-1] = {};
double a4_z[num_of_steps-1] = {};
double a1_yaw[num_of_steps-1] = {};
double a2_yaw[num_of_steps-1] = {};
double a3_yaw[num_of_steps-1] = {};
double a4_yaw[num_of_steps-1] = {};

//reference variables
double px_ref;
double py_ref;
double pz_ref;
double vx_ref;
double vy_ref;
double vz_ref;
double yaw_ref;
double px_ref_future;
double py_ref_future;
double pz_ref_future;
double vx_ref_future;
double vy_ref_future;
double vz_ref_future;
//current state variables
double px_current = 0;
double py_current = 0;
double pz_current = 0;
//offset variables- stores current state when trajectory initialized
double px_offset = 0;
double py_offset = 0;
double pz_offset = 0;
//set final value (normally 0, but for absolute trajectories the final point)
double px_origin = 0;
double py_origin = 0;
double pz_origin = 0;
double px_increment = -8.3;
double py_increment = 3.0;
double pz_increment = -2.5;
double px_final = 0;
double py_final = 0;
double pz_final = 0;//-2.4;
double yaw_final = pi/180*90;
bool armed = false;
bool origin_set = false;
//rc value
double channel6_pwm = 1500;

void rc_callback(const mavros_msgs::RCIn::ConstPtr &msg)
{
    channel6_pwm = msg->channels[5];
    if(channel6_pwm > 1750)
    {
        hover = false;
    }
    else
    {
        hover = true;
    }
}

void current_state_callback(const freyja_msgs::CurrentState::ConstPtr  &msg)
{
    px_current = msg -> pn;
    py_current = msg -> pe;
    pz_current = msg -> pd;
    armed = msg->armed;
    if (armed && !origin_set)
    {
        px_origin = px_current;
        py_origin = py_current;
        pz_origin = pz_current;
        px_final = 0;//px_origin + px_increment;
        py_final = 0;//py_origin + py_increment;
        pz_final = 0;//pz_origin + pz_increment;
        origin_set = true;
    }
    else if(!armed)
    {
        origin_set = false;
    }
}


void interpolate()
{
    //cout << "interpolating \n";
    //set waypoints: x pattern = 0,1,1,0; y pattern = 0,0,1,1; z pattern = 0,0,-1,-1
    //for hover- all 0s
    //for step- all 1 (or other scaled number)
    for (int i = 0; i< num_of_steps; i++)
    {
        t_n[i] = i*step;
        int b = i % 5; //4;
        if (b == 0)
        {
            x_n[i] = 0;
            y_n[i] = 0;
            z_n[i] = 0;
            yaw_n[i] = pi/180*90;//pi/180*0;
        }
        else if (b == 1)
        {
            x_n[i] = 0;
            y_n[i] = 0;
            z_n[i] = 0;
            yaw_n[i] = pi/180*90;//;
        }
        else if (b == 2)
        {
            x_n[i] = 0;
            y_n[i] = 0;
            z_n[i] = 0;
            yaw_n[i] = pi/180*90;//pi/180*180;
        }
        else if (b == 3) //b == 3
        {
            x_n[i] = 0;
            y_n[i] = 0;
            z_n[i] = 0;
            yaw_n[i] = pi/180*90;//pi/180*270;
        }
        else//b == 4
        {
            x_n[i] = 0;
            y_n[i] = 0;
            z_n[i] = 0;
            yaw_n[i] = pi/180*90;//pi/180*270;
        }
    }
    //remember, all velocities at endpoints are 0.
    //scaling
    /*
    x_n = 1*x_n;
    y_n = 1*y_n;
    z_n = 1*z_n;
    */
    //find interpolation constants at each step
    for (int i = 0; i<(num_of_steps-1); i++)
    {
        double t0 = t_n[i];
        double tf = t_n[i+1];
        double x0 = x_n[i];
        double xf = x_n[i+1];
        double y0 = y_n[i];
        double yf = y_n[i+1];
        double z0 = z_n[i];
        double zf = z_n[i+1];
        double u0 = 0;
        double uf = 0;
        double v0 = 0;
        double vf = 0;
        double w0 = 0;
        double wf = 0;
        double yaw0 = yaw_n[i];
        double yawf = yaw_n[i+1];
        MatrixXf A(4,4); 
        A(0,0) = pow(t0,3); A(0,1) = pow(t0,2); A(0,2) = t0; A(0,3) = 1;
        A(1,0) = pow(tf,3); A(1,1) = pow(tf,2); A(1,2) = tf; A(1,3) = 1;
        A(2,0) = 3*pow(t0,2); A(2,1) = 2*t0; A(2,2) = 1; A(2,3) = 0;
        A(3,0) = 3*pow(tf,2); A(3,1) = 2*tf; A(3,2) = 1; A(3,3) = 0;

        MatrixXf bx(4,1);
        bx(0,0) = x0; bx(1,0) = xf; bx(2,0) = u0; bx(3,0) = uf;
        MatrixXf by(4,1);
        by(0,0) = y0; by(1,0) = yf; by(2,0) = v0; by(3,0) = vf;
        MatrixXf bz(4,1);
        bz(0,0) = z0; bz(1,0) = zf; bz(2,0) = w0; bz(3,0) = wf;
        //add b vector for yaw too!


        //solve
        MatrixXf cx = A.partialPivLu().solve(bx);
        MatrixXf cy = A.partialPivLu().solve(by);
        MatrixXf cz = A.partialPivLu().solve(bz);
        a1_x[i] = cx(0,0);
        a2_x[i] = cx(1,0);
        a3_x[i] = cx(2,0);
        a4_x[i] = cx(3,0);

        a1_y[i] = cy(0,0);
        a2_y[i] = cy(1,0);
        a3_y[i] = cy(2,0);
        a4_y[i] = cy(3,0);

        a1_z[i] = cz(0,0);
        a2_z[i] = cz(1,0);
        a3_z[i] = cz(2,0);
        a4_z[i] = cz(3,0);

        a1_yaw[i] = yaw0;
    }

}   

void compute_trajectory()//computes relative trajectory
{
    //cout << "sending trajectory \n";
    if(start_time_set)
    {
        double t = (ros::Time::now() - start_time).toSec();
        double t_future = t + T;
        for (int i = 1; i<num_of_steps; i++) //find the smallest time stamp that the current time is less than
        {
            if (t<t_n[i])
            {
                px_ref = a1_x[i-1]*pow(t,3) + a2_x[i-1]*pow(t,2) + a3_x[i-1]*t + a4_x[i-1];
                py_ref = a1_y[i-1]*pow(t,3) + a2_y[i-1]*pow(t,2) + a3_y[i-1]*t + a4_y[i-1];
                pz_ref = a1_z[i-1]*pow(t,3) + a2_z[i-1]*pow(t,2) + a3_z[i-1]*t + a4_z[i-1];
                vx_ref = 3*a1_x[i-1]*pow(t,2)+ 2*a2_x[i-1]*t + a3_x[i-1];
                vy_ref = 3*a1_y[i-1]*pow(t,2)+ 2*a2_y[i-1]*t + a3_y[i-1];
                vz_ref = 3*a1_z[i-1]*pow(t,2) + 2*a2_z[i-1]*t + a3_z[i-1];
                yaw_ref = a1_yaw[i-1];

                break;
            }
        }
        if (t>=t_n[num_of_steps - 1])
        {   
            px_ref = px_final;
            py_ref = py_final;
            pz_ref = pz_final;
            vx_ref = 0;
            vy_ref = 0;
            vz_ref = 0;
            yaw_ref = pi/180*90;
        }   
        //repeat for t_future
        for (int i = 1; i<num_of_steps; i++) //find the smallest time stamp that the current time is less than
        {
            if (t_future<t_n[i])
            {
                px_ref_future = a1_x[i-1]*pow(t_future,3) + a2_x[i-1]*pow(t_future,2) + a3_x[i-1]*t_future + a4_x[i-1];
                py_ref_future = a1_y[i-1]*pow(t_future,3) + a2_y[i-1]*pow(t_future,2) + a3_y[i-1]*t_future + a4_y[i-1];
                pz_ref_future = a1_z[i-1]*pow(t_future,3) + a2_z[i-1]*pow(t_future,2) + a3_z[i-1]*t_future + a4_z[i-1];
                vx_ref_future = 3*a1_x[i-1]*pow(t_future,2)+ 2*a2_x[i-1]*t_future + a3_x[i-1];
                vy_ref_future = 3*a1_y[i-1]*pow(t_future,2)+ 2*a2_y[i-1]*t_future + a3_y[i-1];
                vz_ref_future = 3*a1_z[i-1]*pow(t_future,2) + 2*a2_z[i-1]*t_future + a3_z[i-1];
                break;
            }
        }
        if (t_future>=t_n[num_of_steps - 1])
        {
            px_ref_future = px_final;
            py_ref_future = py_final;
            pz_ref_future = pz_final;
            vx_ref_future = 0;
            vy_ref_future = 0;
            vz_ref_future = 0;
        }
    }
    else
    {
        px_ref_future = 0;
        py_ref_future = 0;
        pz_ref_future = 0;
        vx_ref_future = 0;
        vy_ref_future = 0;
        vz_ref_future = 0;
        yaw_ref = pi/180*90;
    }
    

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;
    double rate = 1/T;
    ros::Rate sample_rate(rate);//sample_rate(50);
    //subscribe to current state and rc input
    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 1, rc_callback);
    ros::Subscriber current_state_sub = nh.subscribe("/current_state", 1, current_state_callback);
    //publish reference state
    ros::Publisher reference_pub = nh.advertise<control_system_msgs::ReferenceState>("/reference_state", 1);
    
    while (nh.ok())
    {
        ros::spinOnce();
        if(!hover && !start_time_set)
        {
            interpolate();
            start_time = ros::Time::now();
            start_time_set = true;
            px_offset = px_current; //0;
            py_offset = py_current; //0;
            pz_offset = pz_current; //0;
        }
        else if(hover)
        {
            start_time_set = false;
        }
        compute_trajectory();
        control_system_msgs::ReferenceState reference_state_msg;
        reference_state_msg.px_ref = px_ref + px_offset;
        reference_state_msg.py_ref = py_ref + py_offset;
        reference_state_msg.pz_ref = pz_ref + pz_offset;
        reference_state_msg.vx_ref = vx_ref;
        reference_state_msg.vy_ref = vy_ref;
        reference_state_msg.vz_ref = vz_ref;
        reference_state_msg.px_ref_future = px_ref_future + px_offset;
        reference_state_msg.py_ref_future = py_ref_future + py_offset;
        reference_state_msg.pz_ref_future = pz_ref_future + pz_offset;
        reference_state_msg.vx_ref_future = vx_ref_future;
        reference_state_msg.vy_ref_future = vy_ref_future;
        reference_state_msg.vz_ref_future = vz_ref_future;
        reference_state_msg.yaw_ref = yaw_ref;
        
        reference_pub.publish(reference_state_msg);
        sample_rate.sleep();
    }
    
    return 0;
}

