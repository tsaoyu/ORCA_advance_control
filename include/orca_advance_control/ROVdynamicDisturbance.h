#include <ct/core/core.h>  // as usual, include CT
#include <ct/core/internal/traits/TraitSelector.h>
#include <algorithm>
#include <cmath>


namespace rov{

namespace tpl {

template <typename SCALAR>
class ROV : public ct::core::ControlledSystem<18, 4, SCALAR>
{
public:
    static const size_t STATE_DIM = 18;
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

        SCALAR u = s(0);
        SCALAR v = s(1);
        SCALAR w = s(2);

        SCALAR p = s(3);
        SCALAR q = s(4);
        SCALAR r = s(5); 

        // SCALAR x = s(6);
        // SCALAR y = s(7);
        // SCALAR z = s(8);

        SCALAR phi = s(9);
        SCALAR theta = s(10);
        SCALAR psi = s(11);

        SCALAR d1 = s(12);
        SCALAR d2 = s(13);
        SCALAR d3 = s(14);

        SCALAR d4 = s(15);
        SCALAR d5 = s(16);
        SCALAR d6 = s(17);

       

        // Thruster model 
        // normalised control command between -1 to 1
        // Forward (X), Lateral(Y), Throttle(Z) and Yaw(yaw)
        // see https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_Motors6DOF.cpp


        SCALAR X =   -(a(0) - 0.01);
        SCALAR Y =   -(a(1) - 0.01);
        SCALAR Z =   -(a(2) - 0.01);
        SCALAR yaw = -(a(3) - 0.01);


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


        SCALAR f1 = 0.707106781184743*t1 + 0.707106781184743*t2 - 0.70710678119196*t3 - 0.707106781191961*t4 + d1;
        SCALAR f2 = -0.707106781188352*t1 + 0.707106781188352*t2 - 0.707106781181135*t3 + 0.707106781181135*t4 + d2;
        SCALAR f3 = 1.0*t5 + 1.0*t6 + d3;
        SCALAR f4 = 0.0512652416358939*t1 - 0.0512652416358939*t2 + 0.0512652416358939*t3 - 0.0512652416358939*t4 + 0.1105*t5 - 0.1105*t6 + d4; 
        SCALAR f5 = 0.0512652416358939*t1 + 0.0512652416358939*t2 - 0.0512652416358939*t3 - 0.0512652416358939*t4 + 0.00249999999997448*t5 - 0.00249999999997448*t6 + d5;
        SCALAR f6 = -0.166523646969496*t1 + 0.166523646969496*t2 + 0.175008928343413*t3 - 0.175008928343413*t4 + d6;

        
        // first part of state derivative is the velocity

        derivative(0) = -(m*zG*(W*sth*zG - f5 + p*r*(Ix - Kpdot) - p*r*(Iz - Nrdot) + q*(Mq + Mqq*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(q)) - u*w*(Xudot - m) + u*w*(Zwdot - m)) + (Iy + Mqdot)*(f1 + m*r*v + q*w*(Zwdot - m) + sth*(B - W) - u*(Xu + Xuu*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(u))))/(m*m*zG*zG - (Iy + Mqdot)*(Xudot + m));
        derivative(1) = (m*zG*(W*cth*sphi*zG - f4 + p*(Kp + Kpp*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(p)) - q*r*(Iy - Mqdot) + q*r*(Iz - Nrdot) + v*w*(Yvdot - m) - v*w*(Zwdot - m)) + (Ix + Kpdot)*(cth*sphi*(B - W) - f2 + p*w*(Zwdot - m) - r*u*(Xudot - m) + v*(Yv + Yvv*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(v))))/(m*m*zG*zG - (Ix + Kpdot)*(Yvdot + m));
        derivative(2) = (-cphi*cth*(B - W) + f3 + p*v*(Yvdot - m) - q*u*(Xudot - m) - w*(Zw + Zww*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(w)))/(Zwdot + m);
        derivative(3) = (m*zG*(cth*sphi*(B - W) - f2 + p*w*(Zwdot - m) - r*u*(Xudot - m) + v*(Yv + Yvv*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(v))) + (Yvdot + m)*(W*cth*sphi*zG - f4 + p*(Kp + Kpp*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(p)) - q*r*(Iy - Mqdot) + q*r*(Iz - Nrdot) + v*w*(Yvdot - m) - v*w*(Zwdot - m)))/(m*m*zG*zG - (Ix + Kpdot)*(Yvdot + m));
        derivative(4) = (m*zG*(f1 + m*r*v + q*w*(Zwdot - m) + sth*(B - W) - u*(Xu + Xuu*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(u))) + (Xudot + m)*(W*sth*zG - f5 + p*r*(Ix - Kpdot) - p*r*(Iz - Nrdot) + q*(Mq + Mqq*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(q)) - u*w*(Xudot - m) + u*w*(Zwdot - m)))/(m*m*zG*zG - (Iy + Mqdot)*(Xudot + m));
        derivative(5) = (f6 + p*q*(Ix - Kpdot) - p*q*(Iy - Mqdot) - r*(Nr + Nrr*ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(r)) - u*v*(Xudot - m) + u*v*(Yvdot - m))/(Iz + Nrdot);


        // second part is the position

        derivative(6) = cpsi*cth*u - v*(cphi*spsi - cpsi*sphi*sth) + w*(cphi*cpsi*sth + sphi*spsi);
        derivative(7) = cth*spsi*u + v*(cphi*cpsi + sphi*spsi*sth) + w*(cphi*spsi*sth - cpsi*sphi);
        derivative(8) = cphi*cth*w + cth*sphi*v - sth*u;
        derivative(9) = (cphi*r*sth + cth*p + q*sphi*sth)/cth;
        derivative(10) = cphi*q - r*sphi;
        derivative(11) = (cphi*r + q*sphi)/cth;
        

        derivative(12) = 0;
        derivative(13) = 0;
        derivative(14) = 0;
        derivative(15) = 0;
        derivative(16) = 0;
        derivative(17) = 0;
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
