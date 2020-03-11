#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include "ROVdynamic.h"


int main(int argc, char** argv){

    const size_t state_dim = rov::ROV::STATE_DIM;
    const size_t control_dim = rov::ROV::CONTROL_DIM;


    ct::core::StateVector<state_dim> x;
    x.setZero();
   
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

    std::shared_ptr<rov::ROV> rovdynamic (new rov::ROV(Ix, Iy, Iz, m, zG, Xu, Xuu, Yv, Yvv, Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr, Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot, B, W));

    ct::core::Integrator<state_dim> integrator(rovdynamic);

    double dt = 0.01;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 1000;
    integrator.integrate_n_steps(x, t0, nSteps, dt);
    // print the new state
    std::cout << "state after integration: " << x.transpose() << std::endl;
    return 0;

}

