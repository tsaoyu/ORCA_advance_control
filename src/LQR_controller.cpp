#include <ct/optcon/optcon.h>
#include "ROVdynamic.h"
int main(int argc, char** argv){
    const size_t state_dim = rov::ROV::STATE_DIM;
    const size_t control_dim = rov::ROV::CONTROL_DIM;

    ct::core::ADCGScalar m(11.5), zG(0.02), Ix(0.16), Iy(0.16), Iz(0.16); 
    ct::core::ADCGScalar Xudot(5.5), Yvdot(12.7), Zwdot(14.57), Kpdot(0.12), Mqdot(0.12), Nrdot(0.12);

    ct::core::ADCGScalar Xu(4.03), Xuu(18.18), Yv(6.22), Yvv(21.66), Zw(5.18), Zww(36.99);
    ct::core::ADCGScalar Kp(0.07), Kpp(1.55), Mq(0.07), Mqq(1.55), Nr(0.07), Nrr(1.55);   

    ct::core::ADCGScalar W(112.8), Bu(114.8);
   
    std::shared_ptr<rov::tpl::ROV<ct::core::ADCGScalar>> rovdynamic (new 
    rov::tpl::ROV<ct::core::ADCGScalar>(Ix, Iy, Iz, m, zG, Xu, Xuu, Yv, Yvv, 
                                        Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr, 
                                        Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot, Bu, W));

    ct::core::ADCodegenLinearizer<state_dim, control_dim> adLinearizer(rovdynamic);
    
    adLinearizer.compileJIT();

    ct::core::StateVector<state_dim> x;
    x.setZero();
    ct::core::ControlVector<control_dim> u;
    u.setZero();

    double t = 10.0;

    auto A = adLinearizer.getDerivativeState(x,u,t);
    auto B = adLinearizer.getDerivativeControl(x,u,t);

    ct::optcon::TermQuadratic<state_dim, control_dim> quadraticCost;
    ct::core::StateMatrix<state_dim> Q = ct::core::StateMatrix<state_dim>::Identity();
    ct::core::ControlMatrix<control_dim> R = ct::core::ControlMatrix<control_dim>::Identity();

    // quadraticCost.loadConfigFile("/home/yu/Playground/ct_ws/src/ORCA_advance_control/src/lqrCost.info", "termLQR");
    // auto Q = quadraticCost.stateSecondDerivative(x, u, t);    // x, u and t can be arbitrary here
    // auto R = quadraticCost.controlSecondDerivative(x, u, t);  // x, u and t can be arbitrary here

    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    ct::core::FeedbackMatrix<state_dim, control_dim> K;

    std::cout << "A: " << std::endl << A << std::endl << std::endl;
    std::cout << "B: " << std::endl << B << std::endl << std::endl;
    std::cout << "Q: " << std::endl << Q << std::endl << std::endl;
    std::cout << "R: " << std::endl << R << std::endl << std::endl;
    lqrSolver.compute(Q, R, A, B, K);
    std::cout << "LQR gain matrix:" << std::endl << K << std::endl;


}