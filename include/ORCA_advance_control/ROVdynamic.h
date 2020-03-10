#include <ct/core/core.h>  // as usual, include CT
#include <ct/core/internal/traits/TraitSelector.h>
#include <algorithm>
#include <cmath>


namespace rov{

namespace tpl {
// create a class that derives from ct::core::System
template <typename SCALAR>
class ROV : public ct::core::ControlledSystem<12, 6, SCALAR>
{
public:
    static const size_t STATE_DIM = 12;
    static const size_t CONTROL_DIM = 6;
    // constructor
    ROV(SCALAR Ix_, SCALAR Iy_, SCALAR Iz_, SCALAR m_, SCALAR zG_, SCALAR Xu_, SCALAR Xuu_, SCALAR Yv_, SCALAR Yvv_, SCALAR Zw_, SCALAR Zww_, SCALAR Kp_, SCALAR Kpp_, SCALAR Mq_, SCALAR Mqq_, SCALAR Nr_, SCALAR Nrr_, SCALAR Xudot_, SCALAR Yvdot_,SCALAR Zwdot_, SCALAR Kpdot_, SCALAR Mqdot_, SCALAR Nrdot_, SCALAR B_, SCALAR W_) 
    : Ix(Ix_), Iy(Iy_), Iz(Iz_), m(m_), zG(zG_), Xu(Xu_), Xuu(Xuu_), Yv(Yv_), Yvv(Yvv_), Zw(Zw_), Zww(Zww_), Kp(Kp_), Kpp(Kpp_), Mq(Mq_), Mqq(Mqq_), Nr(Nr_), Nrr(Nrr_), Xudot(Xudot_), Yvdot(Yvdot_), Zwdot(Zwdot_), Kpdot(Kpdot_), Mqdot(Mqdot_), Nrdot(Nrdot_), B(B_), W(W_){}
    // copy constructor
    ROV(const ROV& other) :Ix(other.Ix), Iy(other.Iy), Iz(other.Iz), m(other.m), zG(other.zG), Xu(other.Xu), Xuu(other.Xuu), Yv(other.Yv), Yvv(other.Yvv), Zw(other.Zw), Zww(other.Zww), Kp(other.Kp), Kpp(other.Kpp), Mq(other.Mq), Mqq(other.Mqq), Nr(other.Nr), Nrr(other.Nrr), Xudot(other.Xudot), Yvdot(other.Yvdot), Zwdot(other.Zwdot), Kpdot(other.Kpdot), Mqdot(other.Mqdot), Nrdot(other.Nrdot), B(other.B), W(other.W) {}
    // destructor
    ~ROV() {}
    // clone method
    ROV* clone() const
    {
        return new ROV(*this);  // calls copy constructor
    }
    // The system dynamics. We override this method which gets called by e.g. the Integrator
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

        // first part of state derivative is the position
        double cpsi = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(psi);
        double scpi = ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(psi);
        double cphi = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(phi);
        double sphi = ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi);
        double cth = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta);
        double sth = ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta);


        derivative(0) = w*(ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(psi) +
        ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(phi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(psi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta)) -
        v*(ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(phi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(psi) - 
        ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(psi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta)) +
        ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(psi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta)*u;
        
        derivative(1) = v*(ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(phi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(psi) + 
        ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(psi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta)) -
         w*(ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(psi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi)
        
        - ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(phi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(psi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta)) +
         ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta)*u*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(psi); 
        
        derivative(2) = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(phi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta)*w - u*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta) + ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta)*v*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi); 
        
        derivative(3) =  p + (ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(phi)*r*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta))/ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta) + (q*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta))/ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta) ;
        
        derivative(4) = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(phi)*q - r*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi); 
        
        derivative(5) = (ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(phi)*r + q*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi))/ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta);

        
        SCALAR f1 = a(0);
        SCALAR f2 = a(1);
        SCALAR f3 = a(2);
        SCALAR f4 = a(3);
        SCALAR f5 = a(4);
        SCALAR f6 = a(5);
        
        // Thruster model 
        // normalised control command between -1 and 1
        // Forward (X), Lateral(Y), Throttle(Z) and Yaw(psi)
        // see https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_Motors6DOF.cpp

        // SCALAR t1 =-a(0)+a(1)+a(2);
        // SCALAR t2 =-a(0)-a(1)-a(2);
        // SCALAR t3 = a(0)+a(1)-a(2);
        // SCALAR t4 = a(0)-a(1)+a(2);
        // SCALAR t5 = -a(3);
        // SCALAR t6 = -a(3);

        // t1 = math::max(math::min(1.0, t1), -1.0);
        // t2 = math::max(math::min(1.0, t2), -1.0);
        // t3 = math::max(math::min(1.0, t3), -1.0);
        // t4 = math::max(math::min(1.0, t4), -1.0);
        // t5 = math::max(math::min(1.0, t5), -1.0);
        // t6 = math::max(math::min(1.0, t6), -1.0);


        // t1 = -27.37257448 * math::pow(t1, 5)  -1.65738087* math::power(t1, 4) +  53.98829519 * math::power(t1, 3) +  7.34379862 * math::power(t1, 2) +  19.1608364* t1;
        // t2 = -27.37257448 * math::power(t2, 5)  -1.65738087* math::power(t2, 4) +  53.98829519 * math::power(t2, 3) +  7.34379862 * math::power(t2, 2) +  19.1608364* t2;
        // t3 = -27.37257448 * math::power(t3, 5)  -1.65738087* math::power(t3, 4) +  53.98829519 * math::power(t3, 3) +  7.34379862 * math::power(t3, 2) +  19.1608364* t3;
        // t4 = -27.37257448 * math::power(t4, 5)  -1.65738087* math::power(t4, 4) +  53.98829519 * math::power(t4, 3) +  7.34379862 * math::power(t4, 2) +  19.1608364* t4;
        // t5 = -27.37257448 * math::power(t5, 5)  -1.65738087* math::power(t5, 4) +  53.98829519 * math::power(t5, 3) +  7.34379862 * math::power(t5, 2) +  19.1608364* t5;
        // t6 = -27.37257448 * math::power(t6, 5)  -1.65738087* math::power(t6, 4) +  53.98829519 * math::power(t6, 3) +  7.34379862 * math::power(t6, 2) +  19.1608364* t6;


        // SCALAR f1 = 0.707 * t1 + 0.707 * t2 - 0.707 * t3 - 0.707 * t4;
        // SCALAR f2 =-0.707 * t1 + 0.707 * t2 - 0.707 * t3 + 0.707 * t4;
        // SCALAR f3 =-1.0 * t5 - 1.0 * t6;
        // SCALAR f4 = 0.06 * t1 - 0.06 * t2 + 0.06 * t3 - 0.06 * t4 -0.111 * t5 + 0.111 * t6;
        // SCALAR f5 = 0.06 * t1 + 0.06 * t2 - 0.06 * t3 - 0.06 *t4;
        // SCALAR f6 = -0.188 * t1 + 0.188 * t2 + 0.188 * t3 - 0.188 *t4 ;

        // second part is the velocity

        derivative(6) = ((Iy + Mqdot)*(f1 - q*(Zwdot*w + m*w) - u*(Xu + Xuu*ct::core::tpl::TraitSelector<SCALAR>::Trait::sqrt(u * u)) + ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta)*(B - W) + m*v*r))/(Iy*Xudot + Mqdot*Xudot - m*m*zG*zG + Iy*m + Mqdot*m) - (m*zG*(f5 + w*(Xudot*u - m*u) - u*(Zwdot*w - m*v) - q*(Mq + Mqq*ct::core::tpl::TraitSelector<SCALAR>::Trait::sqrt(q * q)) - r*(Ix*p - Kpdot*p) + p*(Iz*r - Nrdot*r) + W*zG*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta)))/(Iy*Xudot + Mqdot*Xudot - m*m*zG*zG + Iy*m + Mqdot*m) ; 

        derivative(7) = ((Ix + Kpdot)*(f2 + r*(Xudot*u - m*u) + p*(Zwdot*w + m*w) - v*(Yv + Yvv*ct::core::tpl::TraitSelector<SCALAR>::Trait::sqrt(v * v)) - ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi)*(B - W)))/(Ix*Yvdot + Kpdot*Yvdot - m*m*zG*zG + Ix*m + Kpdot*m) + (m*zG*(f4 - w*(Yvdot*v - m*v) + v*(Zwdot*w - m*w) - p*(Kp + Kpp*ct::core::tpl::TraitSelector<SCALAR>::Trait::sqrt(p * p)) + r*(Iy*q - Mqdot*q) - q*(Iz*r - Nrdot*r) + W*zG*ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi)))/(Ix*Yvdot + Kpdot*Yvdot - m*m*zG*zG + Ix*m + Kpdot*m);

        derivative(8) =  -(q*(Xudot*u - m*u) - f3 - p*(Yvdot*v - m*v) + w*(Zw + Zww*ct::core::tpl::TraitSelector<SCALAR>::Trait::sqrt(w * w)) + ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi)*(B - W))/(Zwdot + m);

        derivative(9) = ((Yvdot + m)*(f4 - w*(Yvdot*v - m*v) + v*(Zwdot*w - m*w) - p*(Kp + Kpp*ct::core::tpl::TraitSelector<SCALAR>::Trait::sqrt(p * p)) + r*(Iy*q - Mqdot*q) - q*(Iz*r - Nrdot*r) + W*zG*ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi)))/(Ix*Yvdot + Kpdot*Yvdot - m*m*zG*zG + Ix*m + Kpdot*m) + (m*zG*(f2 + r*(Xudot*u - m*u) + p*(Zwdot*w + m*w) - v*(Yv + Yvv*ct::core::tpl::TraitSelector<SCALAR>::Trait::sqrt(v * v)) - ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(theta)*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(phi)*(B - W)))/(Ix*Yvdot + Kpdot*Yvdot - m*m*zG*zG + Ix*m + Kpdot*m) ;

        derivative(10) = ((Xudot + m)*(f5 + w*(Xudot*u - m*u) - u*(Zwdot*w - m*v) - q*(Mq + Mqq*ct::core::tpl::TraitSelector<SCALAR>::Trait::sqrt(q * q)) - r*(Ix*p - Kpdot*p) + p*(Iz*r - Nrdot*r) + W*zG*ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta)))/(Iy*Xudot + Mqdot*Xudot - m*m*zG*zG + Iy*m + Mqdot*m) - (m*zG*(f1 - q*(Zwdot*w + m*w) - u*(Xu + Xuu*ct::core::tpl::TraitSelector<SCALAR>::Trait::sqrt(u * u)) + ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(theta)*(B - W) + m*v*r))/(Iy*Xudot + Mqdot*Xudot - m*m*zG*zG + Iy*m + Mqdot*m);

        derivative(11) =   (f6 - v*(Xudot*u - m*u) + u*(Yvdot*v - m*v) - r*(Nr + Nrr*ct::core::tpl::TraitSelector<SCALAR>::Trait::sqrt(r * r)) + q*(Ix*p - Kpdot*p) - p*(Iy*q - Mqdot*q))/(Iz + Nrdot);
    
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