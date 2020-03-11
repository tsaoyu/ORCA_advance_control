#include <ct/optcon/optcon.h>
#include "ROVdynamic.h"

using namespace ct::core;
using namespace ct::optcon;

int main(int argc, char** argv){
    const size_t state_dim = rov::ROV::STATE_DIM;
    const size_t control_dim = rov::ROV::CONTROL_DIM;

    // ct::core::ADCGScalar m(11.5), zG(0.02), Ix(0.16), Iy(0.16), Iz(0.16); 
    // ct::core::ADCGScalar Xudot(5.5), Yvdot(12.7), Zwdot(14.57), Kpdot(0.12), Mqdot(0.12), Nrdot(0.12);

    // ct::core::ADCGScalar Xu(4.03), Xuu(18.18), Yv(6.22), Yvv(21.66), Zw(5.18), Zww(36.99);
    // ct::core::ADCGScalar Kp(0.07), Kpp(1.55), Mq(0.07), Mqq(1.55), Nr(0.07), Nrr(1.55);   

    // ct::core::ADCGScalar W(112.8), Bu(114.8);
   
    // std::shared_ptr<rov::tpl::ROV<ct::core::ADCGScalar>> rovdynamic (new 
    // rov::tpl::ROV<ct::core::ADCGScalar>(Ix, Iy, Iz, m, zG, Xu, Xuu, Yv, Yvv, 
    //                                     Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr, 
    //                                     Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot, Bu, W));

    double Ix, Iy, Iz;
    double m;
    double zG;
    double Xu, Xuu, Yv, Yvv, Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr;
    double Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot;
    double B, W;

    m = 11.5; // ControlSimulator simulation = ControlSimulator();
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

    std::shared_ptr<rov::ROV> rovdynamic (new 
    rov::ROV(Ix, Iy, Iz, m, zG, Xu, Xuu, Yv, Yvv, 
            Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr, 
            Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot, B, W));

    std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer(
        new ct::core::SystemLinearizer<state_dim, control_dim>(rovdynamic)
    );
    
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
        new ct::optcon::TermQuadratic<state_dim, control_dim>());
    bool verbose = true;
    intermediateCost->loadConfigFile("/home/yu/Playground/ct_ws/src/ORCA_advance_control/src/nlocCost.info", "intermediateCost", verbose);
    finalCost->loadConfigFile("/home/yu/Playground/ct_ws/src/ORCA_advance_control/src/nlocCost.info", "finalCost", verbose);

    std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
        new CostFunctionAnalytical<state_dim, control_dim>());
    costFunction->addIntermediateTerm(intermediateCost);
    costFunction->addFinalTerm(finalCost);

    StateVector<state_dim> x0;

    x0.setRandom();  // in this example, we choose a random initial state x0
    ct::core::Time timeHorizon = 1.0;  // and a final time horizon in [sec]
    // STEP 1-E: create and initialize an "optimal control problem"
    ContinuousOptConProblem<state_dim, control_dim> optConProblem(
        timeHorizon, x0, rovdynamic, costFunction, adLinearizer);
    /* STEP 2: set up a nonlinear optimal control solver. */
    /* STEP 2-A: Create the settings.
     * the type of solver, and most parameters, like number of shooting intervals, etc.,
     * can be chosen using the following settings struct. Let's use, the iterative
     * linear quadratic regulator, iLQR, for this example. In the following, we
     * modify only a few settings, for more detail, check out the NLOptConSettings class. */
    NLOptConSettings nloc_settings;
    nloc_settings.load("/home/yu/Playground/ct_ws/src/ORCA_advance_control/src/nlocSolver.info", true, "ilqr");
    /* STEP 2-B: provide an initial guess */
    // calculate the number of time steps K
    size_t K = nloc_settings.computeK(timeHorizon);
    /* design trivial initial controller for iLQR. Note that in this simple example,
     * we can simply use zero feedforward with zero feedback gains around the initial position.
     * In more complex examples, a more elaborate initial guess may be required.*/
    FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Random());
    StateVectorArray<state_dim> x_ref_init(K + 1, x0);
    NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);
    // STEP 2-C: create an NLOptConSolver instance
    NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, nloc_settings);
    // set the initial guess
    iLQR.setInitialGuess(initController);
    // STEP 3: solve the optimal control problem
    iLQR.solve();
    // STEP 4: retrieve the solution
    ct::core::StateFeedbackController<state_dim, control_dim> solution = iLQR.getSolution();


}