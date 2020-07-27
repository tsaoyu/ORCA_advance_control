#include <ct/optcon/optcon.h>
#include "ROVdynamic.h"
#include "configDir.h"

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace ct::core;
using namespace ct::optcon;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

class LQRController {

    public:
        typedef std::shared_ptr<ct::core::StateFeedbackController<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> ControllerPtr_t;
        typedef std::shared_ptr<ct::core::ADCodegenLinearizer<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> LinearSystemPtr_t;

        LQRController(ros::NodeHandle *n){
            pose_ref_sub = n->subscribe("/pose_ref",10, &LQRController::pose_ref_callback, this);
            cmd_wrench_pub = n->advertise<geometry_msgs::Wrench>("/cmd_wrench_lqr", 10);
            odom_sub = n->subscribe("/BodyROV01/odom", 10, &LQRController::odom_callback, this);
        }


        void create_lqr_controller(const ct::core::StateVector<rov::ROV::STATE_DIM>& x_init, 
                                   const ct::core::StateVector<rov::ROV::STATE_DIM>& x_ref){

            // Step 1: create two dynamics instances

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
            Bu = 114.8;

            std::shared_ptr<rov::ROV> rovdynamic (new 
            rov::ROV(Ix, Iy, Iz, m, zG, Xu, Xuu, Yv, Yvv, 
                    Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr, 
                    Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot, Bu, W));

            ct::core::ADCGScalar m_(m), zG_(zG), Ix_(Ix), Iy_(Iy), Iz_(Iz); 
            ct::core::ADCGScalar Xudot_(Xudot), Yvdot_(Yvdot), Zwdot_(Zwdot), Kpdot_(Kpdot), Mqdot_(Mqdot), Nrdot_(Nrdot);

            ct::core::ADCGScalar Xu_(Xu), Xuu_(Xuu), Yv_(Yv), Yvv_(Yvv), Zw_(Zw), Zww_(Zww);
            ct::core::ADCGScalar Kp_(Kp), Kpp_(Kpp), Mq_(Mq), Mqq_(Mqq), Nr_(Nr), Nrr_(Nrr);   

            ct::core::ADCGScalar W_(W), Bu_(Bu);



            std::shared_ptr<rov::tpl::ROV<ct::core::ADCGScalar>> rovdynamicAD (new 
            rov::tpl::ROV<ct::core::ADCGScalar>(Ix_, Iy_, Iz_, m_, zG_, Xu_, Xuu_, Yv_, Yvv_,
                                                Zw_, Zww_, Kp_, Kpp_, Mq_, Mqq_, Nr_, Nrr_, 
                                                Xudot_, Yvdot_, Zwdot_, Kpdot_, Mqdot_, Nrdot_, Bu_, W_));
            
            // Step 2: create an auto differentiallinearizer

            std::shared_ptr<ct::core::ADCodegenLinearizer<state_dim, control_dim>> adLinearizer(new 
            ct::core::ADCodegenLinearizer<state_dim, control_dim>(rovdynamicAD));

            adLinearizer->compileJIT();


            // Step 3: setup LQR controller 

            double t = 0;
            u(0) = 0.5;
            u(1) = 0.5;
            u(2) = 0.5;
            u(3) = 0.5; // Linearise around operation point

            auto A = adLinearizer->getDerivativeState(x_init, u, t);
            auto B = adLinearizer->getDerivativeControl(x_init, u, t);
            std::cout << "A matrix is: "<< "\n" << A << "\n";
            std::cout << "B matrix is: "<< "\n" << B << "\n" ;

            this->Linearizer = adLinearizer;

            ct::optcon::TermQuadratic<state_dim, control_dim> quadraticCost;
            quadraticCost.loadConfigFile(configDir + "/lqr_parameters.info", "intermediateCost");
            Q = quadraticCost.getStateWeight();
            R = quadraticCost.getControlWeight();
 
            std::cout << "Q matrix is: "<< "\n" << Q << "\n";
            std::cout << "R matrix is: "<< "\n" << R << "\n";

            ct::optcon::LQR<state_dim, control_dim> lqrSolver;
            lqrSolver.compute(Q, R, A, B, K);
            std::cout << "LQR gain matrix:" << std::endl << K << std::endl;

            size_t N = 50;
            FeedbackArray<state_dim, control_dim> u0_fb(N, K);
            ControlVectorArray<control_dim> u0_ff(N, ControlVector<control_dim>::Zero());
            StateVectorArray<state_dim> x_ref_init(N + 1, x_ref);

            ControllerPtr_t controller (new 
                ct::core::StateFeedbackController<state_dim, control_dim>(x_ref_init, u0_ff, u0_fb, 0.02));
            this->controller = controller;

        }

        void pose_message_converter(const geometry_msgs::PoseStamped::ConstPtr & msg,
                                  ct::core::StateVector<rov::ROV::STATE_DIM>& x)
            {
                tf2::Quaternion q(
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w
                    );

                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                x(0) = 0;
                x(1) = 0;
                x(2) = 0;

                x(3) = 0;
                x(4) = 0;
                x(5) = 0;

                x(6) = msg->pose.position.x;
                x(7) = msg->pose.position.y;
                x(8) = msg->pose.position.z;

                x(9) = roll;
                x(10) = pitch;
                x(11) = yaw;
            }

        void odom_message_converter(const nav_msgs::Odometry::ConstPtr & msg, 
                                    StateVector<rov::ROV::STATE_DIM>& x)
            {
                tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w
                    );

                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                x(0) = msg->twist.twist.linear.x;
                x(1) = msg->twist.twist.linear.y;
                x(2) = msg->twist.twist.linear.z;

                x(3) = msg->twist.twist.angular.x;
                x(4) = msg->twist.twist.angular.y;
                x(5) = msg->twist.twist.angular.z;

                x(6) = msg->pose.pose.position.x;
                x(7) = msg->pose.pose.position.y;
                x(8) = msg->pose.pose.position.z;

                x(9) = roll;
                x(10) = pitch;
                x(11) = yaw;

            }

        void odom_callback(const nav_msgs::Odometry::ConstPtr & msg){
            if (first_pass_) {
                first_pass_ =  false;
                LQRController::odom_message_converter(msg, this->x_now);
                return;
            }

            LQRController::odom_message_converter(msg, this->x_now);
        }



        void pose_ref_callback(const geometry_msgs::PoseStamped::ConstPtr & msg){
            
            if (controller_not_created_){
                if (first_pass_){
                    return;
                }
                LQRController::pose_message_converter(msg, this->x_ref);
                LQRController::create_lqr_controller(this->x_now, this->x_ref);
                controller_not_created_ = false;
                start_time = ros::Time::now().toSec();

                return; 
            }

            LQRController::pose_message_converter(msg, this->x_ref);
            
          
        }

       
        void publish_cmd_wrench(){

            if (controller_not_created_){
                return; // Only start to publish when the controller is created
            }
            const size_t state_dim = rov::ROV::STATE_DIM;
            const size_t control_dim = rov::ROV::CONTROL_DIM;
        
            current_time = ros::Time::now().toSec();
            ct::core::Time t = current_time - start_time;
            t_now = t;

            u = K * (this->x_ref - this->x_now);

            geometry_msgs::Wrench wrench;
            wrench.force.x = clip(u(0), -1, 1);
            wrench.force.y = clip(u(1), -1, 1);
            wrench.force.z = clip(u(2), -1, 1);
            wrench.torque.z = clip(u(3),-1, 1);

            cmd_wrench_pub.publish(wrench);
        }

        private:
            ros::Subscriber pose_ref_sub, odom_sub;
            ros::Publisher cmd_wrench_pub;
            ct::core::StateVector<rov::ROV::STATE_DIM> x_now;
            ct::core::StateVector<rov::ROV::STATE_DIM> x_ref;
            ct::core::ControlVector<rov::ROV::CONTROL_DIM> u;  

            double rostime_now;
            ct::core::Time t_now;
            ct::core::Time t_final;

            ControllerPtr_t controller;
            LinearSystemPtr_t Linearizer; 

            ct::core::StateMatrix<rov::ROV::STATE_DIM> Q ;
            ct::core::ControlMatrix<rov::ROV::CONTROL_DIM> R;
            ct::core::FeedbackMatrix<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM> K;

            double start_time;
            double current_time;  
            bool first_pass_ = true;
            bool controller_not_created_ = true;

            double Ix, Iy, Iz;
            double m;
            double zG;
            double Xu, Xuu, Yv, Yvv, Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr;
            double Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot;
            double Bu, W;

};      


int main(int argc, char** argv){
    

    ros::init(argc, argv, "rov_lqr_controller_node");
    ros::NodeHandle n;

    ros::NodeHandle private_node_handle("~");
    LQRController lqr_controller  = LQRController(&n);

    int rate = 50;
    ros::Rate r(rate);

    while (n.ok())

    {   
        ros::spinOnce();
        lqr_controller.publish_cmd_wrench();
        r.sleep();
        
    }
}
