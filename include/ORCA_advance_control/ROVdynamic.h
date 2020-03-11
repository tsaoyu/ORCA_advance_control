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


        derivative(0) = x*std::cos(psi)*std::cos(theta) + y*(std::sin(phi)*std::sin(theta)*std::cos(psi) - std::sin(psi)*std::cos(phi)) + z*(std::sin(phi)*std::sin(psi) + std::sin(theta)*std::cos(phi)*std::cos(psi));

        derivative(1) = x*std::sin(psi)*std::cos(theta) + y*(std::sin(phi)*std::sin(psi)*std::sin(theta) + std::cos(phi)*std::cos(psi)) - z*(std::sin(phi)*std::cos(psi) - std::sin(psi)*std::sin(theta)*std::cos(phi));

        derivative(2) = -x*std::sin(theta) + y*std::sin(phi)*std::cos(theta) + z*std::cos(phi)*std::cos(theta);

        derivative(3) = phi + psi*std::cos(phi)*std::tan(theta) + theta*std::sin(phi)*std::tan(theta);

        derivative(4) = -psi*std::sin(phi) + theta*std::cos(phi);

        derivative(5) = (psi*std::cos(phi) + theta*std::sin(phi))/std::cos(theta);

        SCALAR t1 = a(0);
        SCALAR t2 = a(1);
        SCALAR t3 = a(2);
        SCALAR t4 = a(3);
        SCALAR t5 = a(4);
        SCALAR t6 = a(5);
        
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

        derivative(6) =  -(m*zG*(W*zG*std::sin(theta) + p*psi*(Ix - Kpdot) - phi*r*(Iz - Nrdot) - t5 + theta*(Mq + Mqq*std::fabs(q)) - u*z*(Xudot - m) + w*x*(Zwdot - m)) + (Iy + Mqdot)*(m*psi*v + t1 + theta*w*(Zwdot - m) - x*(Xu + Xuu*std::fabs(u)) + (B - W)*std::sin(theta)))/(std::pow(m, 2)*std::pow(zG, 2) - (Iy + Mqdot)*(Xudot + m));

        derivative(7) = (m*zG*(W*zG*std::sin(phi)*std::cos(theta) + phi*(Kp + Kpp*std::fabs(p)) - psi*q*(Iy - Mqdot) + r*theta*(Iz - Nrdot) - t4 + v*z*(Yvdot - m) - w*y*(Zwdot - m)) + (Ix + Kpdot)*(phi*w*(Zwdot - m) - psi*u*(Xudot - m) - t2 + y*(Yv + Yvv*std::fabs(v)) + (B - W)*std::sin(phi)*std::cos(theta)))/(std::pow(m, 2)*std::pow(zG, 2) - (Ix + Kpdot)*(Yvdot + m));

        derivative(8) = (phi*v*(Yvdot - m) + t3 - theta*u*(Xudot - m) - z*(Zw + Zww*std::fabs(w)) + (-B + W)*std::cos(phi)*std::cos(theta))/(Zwdot + m);

        derivative(9) = (m*zG*(phi*w*(Zwdot - m) - psi*u*(Xudot - m) - t2 + y*(Yv + Yvv*std::fabs(v)) + (B - W)*std::sin(phi)*std::cos(theta)) + (Yvdot + m)*(W*zG*std::sin(phi)*std::cos(theta) + phi*(Kp + Kpp*std::fabs(p)) - psi*q*(Iy - Mqdot) + r*theta*(Iz - Nrdot) - t4 + v*z*(Yvdot - m) - w*y*(Zwdot - m)))/(std::pow(m, 2)*std::pow(zG, 2) - (Ix + Kpdot)*(Yvdot + m));

        derivative(10) = (m*zG*(m*psi*v + t1 + theta*w*(Zwdot - m) - x*(Xu + Xuu*std::fabs(u)) + (B - W)*std::sin(theta)) + (Xudot + m)*(W*zG*std::sin(theta) + p*psi*(Ix - Kpdot) - phi*r*(Iz - Nrdot) - t5 + theta*(Mq + Mqq*std::fabs(q)) - u*z*(Xudot - m) + w*x*(Zwdot - m)))/(std::pow(m, 2)*std::pow(zG, 2) - (Iy + Mqdot)*(Xudot + m));

        derivative(11) = (p*theta*(Ix - Kpdot) - phi*q*(Iy - Mqdot) - psi*(Nr + Nrr*std::fabs(r)) + t6 - u*y*(Xudot - m) + v*x*(Yvdot - m))/(Iz + Nrdot);
    
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