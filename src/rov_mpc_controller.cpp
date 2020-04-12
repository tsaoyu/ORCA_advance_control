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


class LQRController {

    public:
        typedef std::shared_ptr<ct::core::StateFeedbackController<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> ControllerPtr_t;
        typedef std::shared_ptr<rov::tpl::ROV<ct::core::ADCGScalar>> SystemPtr_t;
        typedef std::shared_ptr<ct::core::ADCodegenLinearizer<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> LinearSystemPtr_t;
        typedef std::shared_ptr<NLOptConSolver<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> NLOPPtr_t;
        typedef std::shared_ptr<MPC<NLOptConSolver<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>>> MPCPtr_t;
        typedef ct::core::StateFeedbackController<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM> PolicyPtr_t;
        typedef std::shared_ptr<CostFunctionQuadratic<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> CostFuncPtr_t;

        LQRController(ros::NodeHandle *n){
            pose_ref_sub = n->subscribe("/pose_ref",10, &LQRController::pose_ref_callback, this);
            cmd_wrench_pub = n->advertise<geometry_msgs::Wrench>("/cmd_wrench", 10);
            odom_sub = n->subscribe("/BodyROV01/odom", 10, &LQRController::odom_callback, this);
        }


        void create_dynamics(){
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
            
            this->rovdynamics = rovdynamic;

            ct::core::ADCGScalar m_(m), zG_(zG), Ix_(Ix), Iy_(Iy), Iz_(Iz); 
            ct::core::ADCGScalar Xudot_(Xudot), Yvdot_(Yvdot), Zwdot_(Zwdot), Kpdot_(Kpdot), Mqdot_(Mqdot), Nrdot_(Nrdot);

            ct::core::ADCGScalar Xu_(Xu), Xuu_(Xuu), Yv_(Yv), Yvv_(Yvv), Zw_(Zw), Zww_(Zww);
            ct::core::ADCGScalar Kp_(Kp), Kpp_(Kpp), Mq_(Mq), Mqq_(Mqq), Nr_(Nr), Nrr_(Nrr);   

            ct::core::ADCGScalar W_(W), Bu_(Bu);



            SystemPtr_t rovdynamicAD (new 
            rov::tpl::ROV<ct::core::ADCGScalar>(Ix_, Iy_, Iz_, m_, zG_, Xu_, Xuu_, Yv_, Yvv_,
                                                Zw_, Zww_, Kp_, Kpp_, Mq_, Mqq_, Nr_, Nrr_, 
                                                Xudot_, Yvdot_, Zwdot_, Kpdot_, Mqdot_, Nrdot_, Bu_, W_));
            

            // Step 2: create an auto differentiallinearizer

            LinearSystemPtr_t adLinearizer(new 
            ct::core::ADCodegenLinearizer<state_dim, control_dim>(rovdynamicAD));

            this->Linearizer = adLinearizer;
            adLinearizer->compileJIT();


        }

        void update_ilqr_controller(const ct::core::StateVector<rov::ROV::STATE_DIM>& x_init, 
                                   const ct::core::StateVector<rov::ROV::STATE_DIM>& x_ref){

            const size_t state_dim = rov::ROV::STATE_DIM;
            const size_t control_dim = rov::ROV::CONTROL_DIM;

            std::shared_ptr<TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>> termQuadraticAD_interm(
                new TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>);
            std::shared_ptr<TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>> termQuadraticAD_final(
                new TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>);

            termQuadraticAD_interm->loadConfigFile(configDir + "/ilqr_Cost.info", "intermediateCost");
            termQuadraticAD_final->loadConfigFile(configDir + "/ilqr_Cost.info", "finalCost");
            termQuadraticAD_interm->updateReferenceState(this->x_ref);
            termQuadraticAD_final->updateReferenceState(this->x_ref);

            std::shared_ptr<CostFunctionAD<state_dim, control_dim>> costFunctionAD (
                new CostFunctionAD<state_dim, control_dim>());

            // std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
            //     new CostFunctionQuadraticSimple<state_dim, control_dim>());
            costFunctionAD->addIntermediateADTerm(termQuadraticAD_interm);
            costFunctionAD->addFinalADTerm(termQuadraticAD_final);
            
            costFunctionAD->initialize();



            // Step 3: setup MPC controller

            ct::core::Time timeHorizon = 3.0;
            ContinuousOptConProblem<state_dim, control_dim> optConProblem(
                timeHorizon, x_init, this->rovdynamics, costFunctionAD, this->Linearizer);

            NLOptConSettings nloc_settings;
            nloc_settings.load(configDir + "/ilqr_nloc.info", true, "ilqr");

            size_t N = nloc_settings.computeK(timeHorizon);
            FeedbackArray<state_dim, control_dim> u0_fb(N, FeedbackMatrix<state_dim, control_dim>::Ones());
            ControlVectorArray<control_dim> u0_ff(N, ControlVector<control_dim>::Zero());
            StateVectorArray<state_dim> x_ref_init(N + 1, x_init);
            NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);

            NLOPPtr_t iLQR(new NLOptConSolver<state_dim, control_dim>(optConProblem, nloc_settings));
            this->nlop_problem = iLQR;
            this->nlop_problem->setInitialGuess(initController);
            this->nlop_problem->solve();

            ct::core::StateFeedbackController<state_dim, control_dim> solution = this->nlop_problem->getSolution();
            NLOptConSettings ilqr_settings_mpc = nloc_settings;
            // ... however, in MPC-mode, it makes sense to limit the overall number of iLQR iterations (real-time iteration scheme)
            ilqr_settings_mpc.max_iterations = 1;
            // and we limited the printouts, too.
            ilqr_settings_mpc.printSummary = false;
            // 2) settings specific to model predictive control. For a more detailed description of those, visit ct/optcon/mpc/MpcSettings.h
            ct::optcon::mpc_settings mpc_settings;
            mpc_settings.stateForwardIntegration_ = false;
            mpc_settings.postTruncation_ = false;
            mpc_settings.measureDelay_ = false;
            mpc_settings.delayMeasurementMultiplier_ = 1.0;
            mpc_settings.mpc_mode = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
            mpc_settings.coldStart_ = false;
            // STEP 2 : Create the iLQR-MPC object, based on the optimal control problem and the selected settings.
            MPCPtr_t ilqr_mpc(new MPC<NLOptConSolver<state_dim, control_dim>>(optConProblem, ilqr_settings_mpc, mpc_settings));

            // initialize it using the previously computed initial controller
            this->mpc = ilqr_mpc;
            this->mpc->setInitialGuess(solution);

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
                LQRController::create_dynamics();
                LQRController::update_ilqr_controller(this->x_now, this->x_ref);
                x_ref_current = this->x_ref;
                controller_not_created_ = false;
                start_time = ros::Time::now().toSec();

                return; 
            }

            LQRController::pose_message_converter(msg, this->x_ref);
            if (x_ref_current == this->x_ref){
                return;
                // If the reference is the same as last time received do nothing
            }
            else
            {
                LQRController::update_ilqr_controller(this->x_now, this->x_ref);
                x_ref_current = this->x_ref;
                start_time = ros::Time::now().toSec();
                // Otherwise create a new controller that start from current state to new reference point
                // The internal start time should also updated for the new controller
            }
          
        }

       
        void publish_cmd_wrench(){

            if (controller_not_created_){
                return; // Only start to publish when the controller is created
            }
            const size_t state_dim = rov::ROV::STATE_DIM;
            const size_t control_dim = rov::ROV::CONTROL_DIM;
        
            current_time = ros::Time::now().toSec();
            ct::core::Time t = current_time - start_time;
            this->mpc->prepareIteration(t);
            current_time = ros::Time::now().toSec();
            t = current_time - start_time;
            this->mpc->finishIteration(this->x_now, t, newPolicy, ts_newPolicy);
            current_time = ros::Time::now().toSec();
            t = current_time - start_time;
            ControlVector<control_dim> u;
            newPolicy.computeControl(this->x_now, t, u);

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
            ct::core::StateVector<rov::ROV::STATE_DIM> x_ref_current;
            ct::core::ControlVector<rov::ROV::CONTROL_DIM> u;  

            double rostime_now;
            ct::core::Time t_now;
            ct::core::Time t_final;
            ct::core::Time ts_newPolicy;

            ControllerPtr_t controller;
            LinearSystemPtr_t Linearizer;
            NLOPPtr_t nlop_problem;
            std::shared_ptr<rov::ROV> rovdynamics;
            MPCPtr_t mpc; 
            PolicyPtr_t newPolicy;
            CostFuncPtr_t costFunc; 

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
    

    ros::init(argc, argv, "rov_mpc_controller_node");
    ros::NodeHandle n;

    ros::NodeHandle private_node_handle("~");
    LQRController mpc_controller  = LQRController(&n);

    int rate = 50;
    ros::Rate r(rate);

    while (n.ok())

    {   
        ros::spinOnce();
        mpc_controller.publish_cmd_wrench();
        r.sleep();
        
    }
}
