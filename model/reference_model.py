from __future__ import division
import numpy as np
from sympy import symbols, sin, cos, simplify, init_printing
from sympy.matrices import Matrix, eye, zeros, ones, diag
from sympy.printing.cxxcode import cxxcode

Ix, Iy, Iz = symbols('Ix Iy Iz')
m = symbols('m')
zG = symbols('zG')
Xu, Xuu, Yv, Yvv, Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr = symbols('Xu Xuu Yv Yvv Zw Zww Kp Kpp Mq Mqq Nr Nrr')
Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot = symbols('Xudot Yvdot Zwdot Kpdot Mqdot Nrdot')
B, W = symbols('B W')

x, y, z, phi, theta, psi = symbols('x y z phi theta psi')
u, v, w, p, q, r = symbols('u v w p q r')

nu = Matrix([x, y, z, phi, theta, psi])
t1, t2, t3,t4, t5, t6 = symbols('t1 t2 t3,t4 t5 t6')
tau = Matrix([t1, t2, t3,t4, t5, t6])
xG = 0
yG = 0

Ixy = 0
Ixz = 0
Iyx = 0

Iyz = 0
Izx = 0
Izy = 0


Mrb = Matrix([[m, 0, 0, 0, m * zG, -m * yG],
                [0, m, 0, -m * zG, 0, m * xG],
                [0, 0, m, m * yG, -m * xG, 0],
                [0, -m * zG, m * yG, Ix, -Ixy, -Ixz],
                [m * zG, 0, -m * xG, -Iyx, Iy, -Iyz],
                [-m * yG, m * xG, 0, -Izx, -Izy, Iz]])

Ma = diag(Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot)

M = Mrb + Ma

inM = M.inv()


cpsi = cos(psi)
spsi = sin(psi)
cphi = cos(phi)
sphi = sin(phi)
cth = cos(theta)
sth = sin(theta)


Crb = Matrix([[0, 0, 0,  0, m * w, -m * v],
                [0, 0, 0, -m * w, 0, m * u],
                [0, 0, 0,  m * v, -m * u, 0],
                [0, m * w, -m * v, 0, Iz * r, -Iy * q],
                [-m * w, 0, m * u, -Iz * r, 0, Ix * p],
                [m * v, -m * u, 0, Iy * q, -Ix * p, 0]])

Ca = Matrix([[0, 0, 0, 0, - Zwdot * w, 0],
                [0, 0, 0, Zwdot * w, 0, -Xudot * u],
                [0, 0, 0, -Yvdot * v, Xudot * u, 0],
                [0, -Zwdot * w, Yvdot * v, 0, -Nrdot * r, Mqdot * q],
                [Zwdot * w, 0, -Xudot * u, Nrdot * r, 0, -Kpdot * p],
                [-Yvdot * v, Xudot * u, 0, -Mqdot * q, Kpdot * p, 0]])

Dnu = diag(Xu, Yv, Zw, Kp, Mq, Nr)
Dnl = diag(Xuu * abs(u), Yvv * abs(v), Zww * abs(w),
                Kpp * abs(p), Mqq * abs(q), Nrr * abs(r))


geta = Matrix([ (W - B) * sth,
        -(W - B) * cth * sphi,
        -(W - B) * cth * cphi,
        zG * W * cth * sphi,
        zG * W * sth,
        0])

# nudot = inM.dot(tau - (Crb + Ca).dot(nu) - (Dnu + Dnl).dot(nu) - geta)
nudot = (inM * (tau - (Crb + Ca)* nu  - (Dnu + Dnl) * nu - geta))

J = Matrix([[cpsi * cth, - spsi * cphi + cpsi * sth * sphi, spsi * sphi + cpsi * cphi * sth, 0, 0, 0],
            [spsi * cth, cpsi * cphi + sphi * sth * spsi, - cpsi * sphi + sth * spsi * cphi, 0, 0, 0],
            [-sth, cth * sphi, cth * cphi, 0, 0, 0],
            [0, 0, 0, 1, sphi * sth / cth, cphi * sth / cth],
            [0, 0, 0, 0, cphi, -sphi],
            [0, 0, 0, 0, sphi / cth, cphi / cth]])


etadot = simplify(J * nu)

sim_nudot = simplify(nudot)
sim_etadot = simplify(etadot)

for n in sim_nudot:
    print(cxxcode(n, standard="c++11") + '\n')

for n in sim_etadot:
    print(cxxcode(n, standard="c++11") + '\n')