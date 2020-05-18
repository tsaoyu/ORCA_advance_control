#include <ct/optcon/optcon.h>
#include "ROVdynamic.h"
#include "configDir.h"
#include "plotResult.h"


#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace ct::core;
using namespace ct::optcon;


class MPCController {

    public:
        typedef std::shared_ptr<ct::core::StateFeedbackController<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> ControllerPtr_t;
        typedef std::shared_ptr<rov::tpl::ROV<ct::core::ADCGScalar>> SystemPtr_t;
        typedef std::shared_ptr<ct::core::ADCodegenLinearizer<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> LinearSystemPtr_t;
        typedef std::shared_ptr<NLOptConSolver<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> NLOPPtr_t;
        typedef std::shared_ptr<MPC<NLOptConSolver<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>>> MPCPtr_t;
        typedef ct::core::StateFeedbackController<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM> PolicyPtr_t;
        typedef std::shared_ptr<CostFunctionQuadratic<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> CostFuncPtr_t;
        typedef std::shared_ptr<CostFunctionAD<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> CostFuncADPtr_t;
        // typedef std::shared_ptr<TermQuadratic<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM, double, ct::core::ADCGScalar>> TermPtr_t;
        typedef std::shared_ptr<ct::optcon::TermQuadratic<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> TermPtr_t;
        typedef std::shared_ptr<ct::optcon::TermSmoothAbs<rov::ROV::STATE_DIM, rov::ROV::CONTROL_DIM>> SmoothTermPtr_t;

        MPCController(ros::NodeHandle *n){
            pose_ref_sub = n->subscribe("/pose_ref",10, &MPCController::pose_ref_callback, this);
            cmd_wrench_pub = n->advertise<geometry_msgs::Wrench>("/cmd_wrench", 10);
            odom_sub = n->subscribe("/BodyROV01/odom", 10, &MPCController::odom_callback, this);
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
            this->Linearizer->compileJIT();


        }

        void update_mpc_controller(const ct::core::StateVector<rov::ROV::STATE_DIM>& x_init, 
                                   const ct::core::StateVector<rov::ROV::STATE_DIM>& x_ref){

            const size_t state_dim = rov::ROV::STATE_DIM;
            const size_t control_dim = rov::ROV::CONTROL_DIM;

            // TermPtr_t termQuadraticAD_interm(
            //     new TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>);
            // TermPtr_t termQuadraticAD_final(
            //     new TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>);

            // this->termQuad_interm = termQuadraticAD_interm;
            // this->termQuad_final = termQuadraticAD_final;

            // this->termQuad_interm->loadConfigFile(configDir + "/ilqr_Cost.info", "intermediateCost");
            // this->termQuad_final->loadConfigFile(configDir + "/ilqr_Cost.info", "finalCost");
            // this->termQuad_interm->updateReferenceState(this->x_ref);
            // this->termQuad_final->updateReferenceState(this->x_ref);
            
            // CostFuncADPtr_t costFunctionAD (new CostFunctionAD<state_dim, control_dim>());
            // this->costFuncAD = costFunctionAD;
            // this->costFuncAD->addIntermediateADTerm(this->termQuad_interm);
            // this->costFuncAD->addFinalADTerm(this->termQuad_final);
            
            // this->costFuncAD->initialize();


            std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
                new ct::optcon::TermQuadratic<state_dim, control_dim>());
            std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
                new ct::optcon::TermQuadratic<state_dim, control_dim>());

            this->termQuad_interm = intermediateCost;
            this->termQuad_final = finalCost;

            this->termQuad_interm->loadConfigFile(configDir + "/ilqr_Cost.info", "intermediateCost");
            this->termQuad_final->loadConfigFile(configDir + "/ilqr_Cost.info", "finalCost");

            this->termQuad_interm->updateReferenceState(this->x_ref);
            this->termQuad_final->updateReferenceState(this->x_ref);
            
            CostFuncPtr_t costFunction( new CostFunctionAnalytical<state_dim, control_dim>());

            this->costFunc = costFunction;

            this->costFunc->addIntermediateTerm(this->termQuad_interm);
            this->costFunc->addFinalTerm(this->termQuad_final);
            

            ControlVector<control_dim> ulow;
            ControlVector<control_dim> uhigh;
            ulow  << -1, -1 , -1,  -1;
            uhigh <<  1,  1 ,  1,   1;
            // ulow  << -100, -100,  -100,  -100;
            // uhigh <<  100,  100 ,  100,   100;


            std::shared_ptr<ct::optcon::ControlInputConstraint<state_dim, control_dim>> controlInputBound(
                new ct::optcon::ControlInputConstraint<state_dim, control_dim>(ulow, uhigh));
            
            std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> inputBoxConstraints(
                 new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());
            
            inputBoxConstraints->addIntermediateConstraint(controlInputBound, false);
            // Step 3: setup MPC controller

            float timeH;
            ct::core::loadScalar<float>(configDir + "/ilqr_nloc.info", "Horizon", timeH, "time");
            ct::core::Time timeHorizon = timeH;

            // ContinuousOptConProblem<state_dim, control_dim> optConProblem(
            //     timeHorizon, x_init, this->rovdynamics, this->costFuncAD, this->Linearizer);
            ContinuousOptConProblem<state_dim, control_dim> optConProblem(
                timeHorizon, x_init, this->rovdynamics, this->costFunc, this->Linearizer);
            optConProblem.setInputBoxConstraints(inputBoxConstraints);

            NLOptConSettings nloc_settings;
            nloc_settings.load(configDir + "/ilqr_nloc.info", true, "ilqr");

            size_t N = nloc_settings.computeK(timeHorizon);
            FeedbackArray<state_dim, control_dim> u0_fb(N, FeedbackMatrix<state_dim, control_dim>::Zero());
            ControlVector<control_dim> u0;
            u0(0)=0.2;
            u0(1)=0.2;
            u0(2)=0.2;
            u0(3)=0.2;
            ControlVectorArray<control_dim> u0_ff(N, u0);
            StateVectorArray<state_dim> x_ref_init(N + 1, x_init);
            NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);

            NLOPPtr_t NLOP(new NLOptConSolver<state_dim, control_dim>(optConProblem, nloc_settings));
            this->nlop_problem = NLOP;
            this->nlop_problem->setInitialGuess(initController);
            this->nlop_problem->solve();

            ct::core::StateFeedbackController<state_dim, control_dim> solution = this->nlop_problem->getSolution();
            NLOptConSettings nloc_settings_mpc = nloc_settings;
            // ... however, in MPC-mode, it makes sense to limit the overall number of NLOC iterations (real-time iteration scheme)
            nloc_settings_mpc.max_iterations = 10;
            // and we limited the printouts, too.
            nloc_settings_mpc.printSummary = true;
           
            // 2) settings specific to model predictive control. For a more detailed description of those, visit ct/optcon/mpc/MpcSettings.h
            ct::optcon::mpc_settings mpc_settings;
            // mpc_settings.stateForwardIntegration_ = true;
            // mpc_settings.postTruncation_ = true;
            // mpc_settings.measureDelay_ = true;
            // mpc_settings.delayMeasurementMultiplier_ = 1.0;
            // mpc_settings.mpc_mode = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
            // // mpc_settings.minimumTimeHorizonMpc_ = 2;
            // mpc_settings.coldStart_ = false;
            ct::optcon::loadMpcSettings(configDir + "/mpcSolver.info", mpc_settings);

            // STEP 2 : Create the NLOC-MPC object, based on the optimal control problem and the selected settings.
            MPCPtr_t nloc_mpc(new MPC<NLOptConSolver<state_dim, control_dim>>(optConProblem, nloc_settings_mpc, mpc_settings));

            // initialize it using the previously computed optimal controller
            this->mpc = nloc_mpc;
            this->mpc->setInitialGuess(solution);

        }
        void plot_solution(){
            const size_t state_dim = rov::ROV::STATE_DIM;
            const size_t control_dim = rov::ROV::CONTROL_DIM;

            ct::core::StateFeedbackController<state_dim, control_dim> solution = this->nlop_problem->getSolution();

            plotResultsROV<state_dim, control_dim>(solution.x_ref(),
                                                   solution.K(),
                                                   solution.uff(), 
                                                   solution.time());

            
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
                MPCController::odom_message_converter(msg, this->x_now);
                return;
            }

            MPCController::odom_message_converter(msg, this->x_now);
        }



        void pose_ref_callback(const geometry_msgs::PoseStamped::ConstPtr & msg){
            
            if (controller_not_created_){
                if (first_pass_){
                    return;
                }
                MPCController::pose_message_converter(msg, this->x_ref);
                MPCController::create_dynamics();
                MPCController::update_mpc_controller(this->x_now, this->x_ref);
                x_ref_current = this->x_ref;
                // MPCController::plot_solution();
                controller_not_created_ = false;
                start_time = ros::Time::now().toSec();

                return; 
            }

            MPCController::pose_message_converter(msg, this->x_ref);
            if (x_ref_current == this->x_ref){
                return;
                // If the reference is the same as last time received do nothing
            }
            else
            {
              
                this->termQuad_interm->updateReferenceState(this->x_ref);
                this->termQuad_final->updateReferenceState(this->x_ref);

                // this->costFuncAD->addIntermediateADTerm(this->termQuad_interm);
                // this->costFuncAD->addFinalADTerm(this->termQuad_interm);
                // this->costFuncAD->initialize();

                this->costFunc->addIntermediateTerm(this->termQuad_interm);
                this->costFunc->addFinalTerm(this->termQuad_final);
                     
                auto solver = this->mpc->getSolver();
                solver.changeCostFunction(this->costFuncAD);

                x_ref_current = this->x_ref;
                start_time = ros::Time::now().toSec();
                // The internal start time should also updated for the new controller
            }
          
        }

        void print_mpc_summary(){
            this->mpc->printMpcSummary();
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
            #ifdef DEBUG_MPC
            std::cout << "Current time: " << t << " Policy time stamp: " << ts_newPolicy << " Time horizon reached: " << this->mpc->timeHorizonReached() << std::endl;

            saveResultROV<state_dim, control_dim>(newPolicy.x_ref(),
                                                   newPolicy.K(),
                                                   newPolicy.uff(), 
                                                   newPolicy.time());
            #endif
            newPolicy.computeControl(this->x_now, t - ts_newPolicy, u);

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
            CostFuncADPtr_t costFuncAD;
            TermPtr_t termQuad_interm, termQuad_final;
            SmoothTermPtr_t termSmooth_interm;
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
    MPCController mpc_controller  = MPCController(&n);

    int rate = 50;
    ros::Rate r(rate);

    while (n.ok())

    {   
        ros::spinOnce();
        mpc_controller.publish_cmd_wrench();
        r.sleep();
        
    }
    mpc_controller.print_mpc_summary();
}

