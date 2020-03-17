#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include <ros/ros.h>
#include "ROVdynamic.h"
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class DynamicSimulator{
    private:

        ros::Publisher odom_pub;
        ros::Subscriber cmd_wrench_sub;
        std::shared_ptr<rov::ROV> rovdynamic_ptr;
        ct::core::Integrator<rov::ROV::STATE_DIM> * integrator;
        ct::core::StateVector<rov::ROV::STATE_DIM> x;
        bool first_received = true;
        double rostime_now; 
        ct::core::Time t_now;
        ct::core::Time t_final;
        
        double Ix, Iy, Iz;
        double m;
        double zG;
        double Xu, Xuu, Yv, Yvv, Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr;
        double Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot;
        double B, W;
          

    public:

        DynamicSimulator(ros::NodeHandle *n){
            DynamicSimulator::create_controlled_system();
            odom_pub = n->advertise<nav_msgs::Odometry>("/BodyROV01/odom", 10);
            cmd_wrench_sub = n->subscribe("/cmd_wrench", 10,  &DynamicSimulator::callback_cmd_wrench, this);
        }

        void create_controlled_system(){
            const size_t state_dim = rov::ROV::STATE_DIM;
            const size_t control_dim = rov::ROV::CONTROL_DIM;
           

            Ix = 0.16; Iy = 0.16; Iz = 0.16;
            m = 11.5;
            zG = 0.08;
            
            Xudot = 5.5;
            Yvdot = 12.7; 
            Zwdot = 14.57;
            Kpdot = 0.12;
            Mqdot = 0.12;
            Nrdot = 0.12;

            Xu = 4.03; Xuu = 18.18;  
            Yv = 6.22; Yvv = 21.66;  
            Zw = 5.18; Zww = 36.99;
            Kp = 0.07; Kpp = 1.55;
            Mq = 0.07; Mqq = 1.55;
            Nr = 0.07; Nrr = 1.55;   

            W = 112.8;
            B = 114.8;
            
            std::shared_ptr<rov::ROV> rovdynamic (new rov::ROV(Ix, Iy, Iz, m, zG, Xu, Xuu, Yv, 
                                                               Yvv, Zw, Zww, Kp, Kpp, Mq, Mqq,
                                                               Nr, Nrr, Xudot, Yvdot, Zwdot, 
                                                               Kpdot, Mqdot, Nrdot, B, W));

            
            this->rovdynamic_ptr = rovdynamic;
            ct::core::ControlVector<control_dim> manual_control;

            this->integrator = new ct::core::Integrator<rov::ROV::STATE_DIM>(this->rovdynamic_ptr);
          

        }

        void reset_time(){
            t_now = 0.0;
        }

        void callback_cmd_wrench(const geometry_msgs::Wrench::ConstPtr& msg){
            if (first_received) {
                DynamicSimulator::reset_time();
                first_received = false;
                rostime_now = ros::Time::now().toSec();
                x.setZero();
                return ;

            }
            const size_t state_dim = rov::ROV::STATE_DIM;
            const size_t control_dim = rov::ROV::CONTROL_DIM;
            ct::core::ControlVector<control_dim> manual_control;

            double fx = msg->force.x;
            double fy = msg->force.y;
            double fz = msg->force.z;
            double tz = msg->torque.z;


            manual_control(0) = fx;
            manual_control(1) = fy;
            manual_control(2) = fz;
            manual_control(3) = tz;

            this->rovdynamic_ptr->updateManualControl(manual_control);

            double dt = ros::Time::now().toSec() - rostime_now;
            t_final = t_now + dt;
            rostime_now = ros::Time::now().toSec();
            const size_t nSteps = 10;
            this -> integrator -> integrate_n_steps(x, t_now, nSteps, dt);
            t_now = t_final;

        }

        void publish_odom(){
            nav_msgs::Odometry odom;
            odom.child_frame_id = "BodyROV01";
            odom.header.frame_id = "qualisys";
            odom.header.stamp = ros::Time::now();
            
            tf2::Quaternion q;
            q.setRPY(x(3), x(4), x(5));

            odom.pose.pose.position.x = x(0);
            odom.pose.pose.position.y = x(1);
            odom.pose.pose.position.z = x(2);

           
            odom.pose.pose.orientation.x = q[0];
            odom.pose.pose.orientation.y = q[1];
            odom.pose.pose.orientation.z = q[2];
            odom.pose.pose.orientation.w = q[3]; 

            odom.twist.twist.linear.x = x(6);
            odom.twist.twist.linear.y = x(7); 
            odom.twist.twist.linear.z = x(8); 

            odom.twist.twist.linear.x = x(9);
            odom.twist.twist.linear.y = x(10); 
            odom.twist.twist.linear.z = x(11); 

            odom_pub.publish(odom);

        }

};


int main(int argc, char **argv){

    ros::init(argc, argv, "manual_control_simulator_node");
    ros::NodeHandle n;

    ros::NodeHandle private_node_handle("~");
    DynamicSimulator dynamic_simulator  = DynamicSimulator(&n);

    int rate = 100;
    ros::Rate r(rate);

    while (n.ok())

    {   
        ros::spinOnce();
        dynamic_simulator.publish_odom();
        r.sleep();
        
    }

}

