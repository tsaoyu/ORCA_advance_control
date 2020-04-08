#include <ct/optcon/optcon.h>
#include "ROVdynamic.h"
#include "configDir.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace ct::core;
using namespace ct::optcon;


class MPCController {

    public:
        typedef std::shared_ptr<MPC<NLOptConSolver<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>>> MPCControllerPtr_t;
        typedef ct::core::StateFeedbackController<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM> Policy_t;
        typedef std::shared_ptr<CostFunctionQuadratic<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> CostFunc_t;

        MPCController(ros::NodeHandle *n){
            pose_ref_sub = n->subscribe("/pose_ref",10, &MPCController::pose_ref_callback, this);
            cmd_wrench_pub = n->advertise<geometry_msgs::Wrench>("/cmd_wrench", 10);
            odom_sub = n->subscribe("/BodyROV01/odom", 10, &MPCController::odom_callback, this);
        }


        void create_mpc_controller(const ct::core::StateVector<rov::ROV::STATE_DIM>& x_init, 
                                   const ct::core::StateVector<rov::ROV::STATE_DIM>& x_ref){

            // Step 1: create two rov dynamics instances
            // One for linearizer and one for optcon problem

            const size_t state_dim = rov::ROV::STATE_DIM;
            const size_t control_dim = rov::ROV::CONTROL_DIM;

            double Ix, Iy, Iz;
            double m;
            double zG;
            double Xu, Xuu, Yv, Yvv, Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr;
            double Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot;
            double Bu, W;

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

            // Step 2: create an auto differential linearizer

            std::shared_ptr<ct::core::ADCodegenLinearizer<state_dim, control_dim>> adLinearizer(new 
            ct::core::ADCodegenLinearizer<state_dim, control_dim>(rovdynamicAD));

            adLinearizer->compileJIT();
            

            // Step 3: read intermediate and final cost from configuration files

            std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
                new ct::optcon::TermQuadratic<state_dim, control_dim>());
            std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
                new ct::optcon::TermQuadratic<state_dim, control_dim>());

            bool verbose = true;
            intermediateCost->loadConfigFile(configDir + "/nlocCost.info", "intermediateCost", verbose);
            finalCost->loadConfigFile(configDir + "/nlocCost.info", "finalCost", verbose);

            std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
                new CostFunctionAnalytical<state_dim, control_dim>());

            this -> costFunc = costFunction;

            this->costFunc->addIntermediateTerm(intermediateCost, verbose);
            this->costFunc->addFinalTerm(finalCost, verbose);


            // Step 4: include lower and upper bound of  control constriants

            ControlVector<control_dim> ulow;
            ControlVector<control_dim> uhigh;
            ulow  << -1, -1,  -1,  -1;
            uhigh <<  1,  1 ,  1,   1;

            std::shared_ptr<ct::optcon::ControlInputConstraint<state_dim, control_dim>> controlInputBound(
                new ct::optcon::ControlInputConstraint<state_dim, control_dim>(ulow, uhigh));
            
            std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> inputBoxConstraints(
                 new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

            inputBoxConstraints->addIntermediateConstraint(controlInputBound, verbose);
            inputBoxConstraints->initialize();


            // Step 5: setup the non linear optimal control solver
      
            ct::core::Time timeHorizon = 5.0;  

            ContinuousOptConProblem<state_dim, control_dim> optConProblem(
                timeHorizon, x_init, rovdynamic, costFunction, adLinearizer);

            // optConProblem.setInputBoxConstraints(inputBoxConstraints);

            NLOptConSettings nloc_settings;
            nloc_settings.load(configDir + "/ilqrcSolver.info", true, "ilqr");
         
            size_t K = nloc_settings.computeK(timeHorizon);

            FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
            ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
            StateVectorArray<state_dim> x_ref_init(K + 1, x_ref);
            NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);

            NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, nloc_settings);
            iLQR.setInitialGuess(initController);

            // Step 6: get initial solution from non linear optimal controller

            iLQR.solve();
            ct::core::StateFeedbackController<state_dim, control_dim> initialSolution = iLQR.getSolution();

            ControlVector<control_dim> u;

            std::cout << initialSolution.x_ref()[8];

            initialSolution.computeControl(x_init, 0, u);

            std::cout << u;


            // Step 7: configure MPC control from configuration file

            NLOptConSettings ilqr_settings_mpc = nloc_settings;
            ilqr_settings_mpc.max_iterations = 1;
            ilqr_settings_mpc.printSummary = true;


            ct::optcon::mpc_settings mpc_settings;
            ct::optcon::loadMpcSettings(configDir + "/mpcSolver.info", mpc_settings);

            std::shared_ptr<MPC<NLOptConSolver<state_dim, control_dim>>> ilqr_mpc(new MPC<NLOptConSolver<state_dim, control_dim>>(optConProblem, ilqr_settings_mpc, mpc_settings));

            this -> mpc_controller = ilqr_mpc;
            this -> mpc_controller -> setInitialGuess(initialSolution);

            mpc_start_time = ros::Time::now().toSec();

        }

        void pose_message_converter(const geometry_msgs::PoseStamped::ConstPtr & msg, 
                                    StateVector<rov::ROV::STATE_DIM>& x)
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
                MPCController::odom_message_converter(msg, this->x_now);
                return;
            }

            MPCController::odom_message_converter(msg, this->x_now);
        }


        void pose_ref_callback(const geometry_msgs::PoseStamped::ConstPtr & msg){
            
            if (mpc_controller_not_created_){
                if (first_pass_){
                    return;
                }
                MPCController::pose_message_converter(msg, this->x_ref);
                MPCController::create_mpc_controller(this->x_now, this->x_ref);
                mpc_controller_not_created_ = false;
                return; 
            }

            MPCController::pose_message_converter(msg, this->x_ref);
            this->costFunc->updateReferenceState(this->x_ref);
        }

       
        void publish_cmd_wrench(){

            if (mpc_controller_not_created_){
                return; // Only start to publish when the mpc controller is created
            }
            const size_t control_dim = rov::ROV::CONTROL_DIM;
            ct::core::Time ts_newPolicy;
        
            current_time = ros::Time::now().toSec();
            ct::core::Time t = current_time - mpc_start_time;
            this->mpc_controller->prepareIteration(t);
            current_time = ros::Time::now().toSec();
            t = current_time - mpc_start_time;
            this->mpc_controller->finishIteration(this->x_now, t, newPolicy, ts_newPolicy);
            current_time = ros::Time::now().toSec();
            t = current_time - mpc_start_time;
            ControlVector<control_dim> u;

            newPolicy.computeControl(this->x_now, 0, u);
  
            geometry_msgs::Wrench wrench;
            wrench.force.x = u(0);
            wrench.force.y = u(1);
            wrench.force.z = u(2);
            wrench.torque.z = u(3);

            cmd_wrench_pub.publish(wrench);
        }

        private:
            ros::Subscriber pose_ref_sub, odom_sub;
            ros::Publisher cmd_wrench_pub;
            ct::core::StateVector<rov::ROV::STATE_DIM> x_now;
            ct::core::StateVector<rov::ROV::STATE_DIM> x_ref;


            double rostime_now;
            ct::core::Time t_now;
            ct::core::Time t_final;

            MPCControllerPtr_t mpc_controller;
            Policy_t newPolicy;
            CostFunc_t costFunc; 

            double mpc_start_time;
            double current_time;  
            bool first_pass_ = true;
            bool mpc_controller_not_created_ = true;

            double Ix, Iy, Iz;
            double m;
            double zG;
            double Xu, Xuu, Yv, Yvv, Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr;
            double Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot;
            double B, W;


};


int main(int argc, char** argv){
    std::cout<<configDir;

    ros::init(argc, argv, "mpc_controller_node");
    ros::NodeHandle n;

    ros::NodeHandle private_node_handle("~");
    MPCController mpc_controller  = MPCController(&n);

    int rate = 20;
    ros::Rate r(rate);

    while (n.ok())

    {   
        ros::spinOnce();
        mpc_controller.publish_cmd_wrench();
        r.sleep();
        
    }
}

