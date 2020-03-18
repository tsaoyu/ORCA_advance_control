#include <ct/core/core.h>  // as usual, include CT
#include <ct/core/internal/traits/TraitSelector.h>
#include <algorithm>
#include <cmath>


namespace rov{

namespace tpl {

template <typename SCALAR>
class ROV : public ct::core::ControlledSystem<12, 4, SCALAR>
{
public:
    static const size_t STATE_DIM = 12;
    static const size_t CONTROL_DIM = 4;
 
    ROV(SCALAR Ix_, SCALAR Iy_, SCALAR Iz_, SCALAR m_, SCALAR zG_, SCALAR Xu_, SCALAR Xuu_, SCALAR Yv_, SCALAR Yvv_, SCALAR Zw_, SCALAR Zww_, SCALAR Kp_, SCALAR Kpp_, SCALAR Mq_, SCALAR Mqq_, SCALAR Nr_, SCALAR Nrr_, SCALAR Xudot_, SCALAR Yvdot_,SCALAR Zwdot_, SCALAR Kpdot_, SCALAR Mqdot_, SCALAR Nrdot_, SCALAR B_, SCALAR W_) 
    : Ix(Ix_), Iy(Iy_), Iz(Iz_), m(m_), zG(zG_), Xu(Xu_), Xuu(Xuu_), Yv(Yv_), Yvv(Yvv_), Zw(Zw_), Zww(Zww_), Kp(Kp_), Kpp(Kpp_), Mq(Mq_), Mqq(Mqq_), Nr(Nr_), Nrr(Nrr_), Xudot(Xudot_), Yvdot(Yvdot_), Zwdot(Zwdot_), Kpdot(Kpdot_), Mqdot(Mqdot_), Nrdot(Nrdot_), B(B_), W(W_){}
 
    ROV(const ROV& other) :Ix(other.Ix), Iy(other.Iy), Iz(other.Iz), m(other.m), zG(other.zG), Xu(other.Xu), Xuu(other.Xuu), Yv(other.Yv), Yvv(other.Yvv), Zw(other.Zw), Zww(other.Zww), Kp(other.Kp), Kpp(other.Kpp), Mq(other.Mq), Mqq(other.Mqq), Nr(other.Nr), Nrr(other.Nrr), Xudot(other.Xudot), Yvdot(other.Yvdot), Zwdot(other.Zwdot), Kpdot(other.Kpdot), Mqdot(other.Mqdot), Nrdot(other.Nrdot), B(other.B), W(other.W) {}
 
    ~ROV() {}
 
    ROV* clone() const
    {
        return new ROV(*this);   
    }
 
    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM, SCALAR>& s,
        const SCALAR& t,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& a,
        ct::core::StateVector<STATE_DIM, SCALAR>& derivative) override
    {   

        SCALAR x = s(0);
        SCALAR y = s(1);
        SCALAR z = s(2);

        SCALAR phi = s(3);
        SCALAR theta = s(4);
        SCALAR psi = s(5);

        SCALAR u = s(6);
        SCALAR v = s(7);
        SCALAR w = s(8);

        SCALAR p = s(9);
        SCALAR q = s(10);
        SCALAR r = s(11); 

        // Thruster model 
        // normalised control command between -1 and 1
        // Forward (X), Lateral(Y), Throttle(Z) and Yaw(yaw)
        // see https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_Motors6DOF.cpp


        SCALAR X = a(0);
        SCALAR Y = a(1);
        SCALAR Z = a(2);
        SCALAR yaw = a(3);


        SCALAR cpsi = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(psi);
        SCALAR spsi = ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(psi);
        SCALAR cphi = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(phi);
        SCALAR sphi = ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi);
        SCALAR cth = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta);
        SCALAR sth = ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta);

        SCALAR t1 = -40 + 80/(1 + ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(-4*(-X + Y + yaw)*(-X + Y + yaw)*(-X + Y + yaw)));
        SCALAR t2 = -40 + 80/(1 + ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(-4*(-X - Y - yaw)*(-X - Y - yaw)*(-X - Y - yaw)));
        SCALAR t3 = -40 + 80/(1 + ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(-4*(X + Y - yaw)*(X + Y - yaw)*(X + Y - yaw)));
        SCALAR t4 = -40 + 80/(1 + ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(-4*(X - Y + yaw)*(X - Y + yaw)*(X - Y + yaw)));
        SCALAR t5 = -40 + 80/(ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(4*Z*Z*Z) + 1);
        SCALAR t6 = -40 + 80/(ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(4*Z*Z*Z) + 1);


        SCALAR f1 = 0.707106781184743*t1 + 0.707106781184743*t2 - 0.70710678119196*t3 - 0.707106781191961*t4;
        SCALAR f2 = -0.707106781188352*t1 + 0.707106781188352*t2 - 0.707106781181135*t3 + 0.707106781181135*t4;
        SCALAR f3 = 1.0*t5 + 1.0*t6;
        SCALAR f4 = 0.0512652416358939*t1 - 0.0512652416358939*t2 + 0.0512652416358939*t3 - 0.0512652416358939*t4 + 0.1105*t5 - 0.1105*t6;
        SCALAR f5 = 0.0512652416358939*t1 + 0.0512652416358939*t2 - 0.0512652416358939*t3 - 0.0512652416358939*t4 + 0.00249999999997448*t5 - 0.00249999999997448*t6;
        SCALAR f6 = -0.166523646969496*t1 + 0.166523646969496*t2 + 0.175008928343413*t3 - 0.175008928343413*t4;

        
        // first part of state derivative is the velocity

        derivative(0) = -(m*zG*(W*sth*zG + f5 + p*psi*(Ix - Kpdot) - phi*r*(Iz - Nrdot) + theta*(Mq + Mqq*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(q)) - u*z*(Xudot - m) + w*x*(Zwdot - m)) + (Iy + Mqdot)*(-f1 + m*psi*v + sth*(B - W) + theta*w*(Zwdot - m) - x*(Xu + Xuu*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(u))))/(m*m*zG*zG - (Iy + Mqdot)*(Xudot + m));
        derivative(1) = (m*zG*(W*cth*sphi*zG + f4 + phi*(Kp + Kpp*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(p)) - psi*q*(Iy - Mqdot) + r*theta*(Iz - Nrdot) + v*z*(Yvdot - m) - w*y*(Zwdot - m)) + (Ix + Kpdot)*(cth*sphi*(B - W) + f2 + phi*w*(Zwdot - m) - psi*u*(Xudot - m) + y*(Yv + Yvv*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(v))))/(m*m*zG*zG - (Ix + Kpdot)*(Yvdot + m));
        derivative(2) = (-cphi*cth*(B - W) - f3 + phi*v*(Yvdot - m) - theta*u*(Xudot - m) - z*(Zw + Zww*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(w)))/(Zwdot + m);
        derivative(3) = (m*zG*(cth*sphi*(B - W) + f2 + phi*w*(Zwdot - m) - psi*u*(Xudot - m) + y*(Yv + Yvv*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(v))) + (Yvdot + m)*(W*cth*sphi*zG + f4 + phi*(Kp + Kpp*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(p)) - psi*q*(Iy - Mqdot) + r*theta*(Iz - Nrdot) + v*z*(Yvdot - m) - w*y*(Zwdot - m)))/(m*m*zG*zG - (Ix + Kpdot)*(Yvdot + m));
        derivative(4) = (m*zG*(-f1 + m*psi*v + sth*(B - W) + theta*w*(Zwdot - m) - x*(Xu + Xuu*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(u))) + (Xudot + m)*(W*sth*zG + f5 + p*psi*(Ix - Kpdot) - phi*r*(Iz - Nrdot) + theta*(Mq + Mqq*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(q)) - u*z*(Xudot - m) + w*x*(Zwdot - m)))/(m*m*zG*zG - (Iy + Mqdot)*(Xudot + m));
        derivative(5) = (-f6 + p*theta*(Ix - Kpdot) - phi*q*(Iy - Mqdot) - psi*(Nr + Nrr*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(r)) - u*y*(Xudot - m) + v*x*(Yvdot - m))/(Iz + Nrdot);

        // second part is the position

        derivative(6) = cpsi*cth*x - y*(cphi*spsi - cpsi*sphi*sth) + z*(cphi*cpsi*sth + sphi*spsi);
        derivative(7) = cth*spsi*x + y*(cphi*cpsi + sphi*spsi*sth) + z*(cphi*spsi*sth - cpsi*sphi);
        derivative(8) = cphi*cth*z + cth*sphi*y - sth*x;
        derivative(9) = (cphi*psi*sth + cth*phi + sphi*sth*theta)/cth;
        derivative(10) = cphi*theta - psi*sphi;
        derivative(11) = (cphi*psi + sphi*theta)/cth;
        
        // std::cout << std::showpos;
        // std::cout << std::showpoint;
        // std::cout << std::scientific;
        // std::cout << std::setprecision(4);

        // std::cout << "Commands:     " << std::setw(8) << X << " " << Y << " " << Z << " " << yaw << "\n";
        // std::cout << "Derivatives:  " << std::setw(8) << derivative(0) << " " << derivative(1) << " " << derivative(2) << " " << derivative(3) << " " << derivative(4) << " " << derivative(5) << " " << derivative(6) << " " << derivative(7) << " " << derivative(8) << " " << derivative(9) << " " << derivative(10) << " " << derivative(11) << " " << "\n";
        // std::cout << "States:       " << std::setw(8) << s(0) << " " << s(1) << " " << s(2) << " " << s(3) << " " << s(4) << " " << s(5) << " " << s(6) << " " << s(7) << " " << s(8) << " " << s(9) << " " << s(10) << " " << s(11) << " " << "\n";
        // std::cout << "Thrusts:      " << std::setw(8) << t1 << " " << t2 << " " << t3 << " " << t4 << " " << t5 << " " << t6 << "\n";
        // std::cout << "Force/Moment: " << std::setw(8) << f1 << " " << f2 << " " << f3 << " " << f4 << " " << f5 << " " << f6 << "\n";



    }

    void updateManualControl(const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control) override
    {
       ROV::manualcontrolAction_ = control;
       ROV::isManual_ = true;

    }

private:
    
    SCALAR Ix, Iy, Iz;
    SCALAR m;
    SCALAR zG;
    SCALAR Xu, Xuu, Yv, Yvv, Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr;
    SCALAR Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot;
    SCALAR B, W;

};

} // namespace rov

using ROV = tpl::ROV<double>;

} // namespace tpl