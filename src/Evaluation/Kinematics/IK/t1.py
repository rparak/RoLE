from sympy import symbols, init_printing, Matrix, eye, sin, cos, pi
init_printing(use_unicode=True)

# https://gist.github.com/mlaves/a60cbc5541bd6c9974358fbaad9e4c51
q1, q2, q3, q4, q5, q6, q7 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 theta_7')
joint_angles = [q1, q2, q3, q4, q5, q6, q7]

dh_craig = [
    {'a':  0,      'd': 0.333, 'alpha':  0,  },
    {'a':  0,      'd': 0,     'alpha': -pi/2},
    {'a':  0,      'd': 0.316, 'alpha':  pi/2},
    {'a':  0.0825, 'd': 0,     'alpha':  pi/2},
    {'a': -0.0825, 'd': 0.384, 'alpha': -pi/2},
    {'a':  0,      'd': 0,     'alpha':  pi/2},
    {'a':  0.088,  'd': 0.107, 'alpha':  pi/2},
]

DK = eye(4)

for i, (p, q) in enumerate(zip(reversed(dh_craig), reversed(joint_angles))):
    d = p['d']
    a = p['a']
    alpha = p['alpha']

    ca = cos(alpha)
    sa = sin(alpha)
    cq = cos(q)
    sq = sin(q)

    transform = Matrix(
        [
            [cq, -sq, 0, a],
            [ca * sq, ca * cq, -sa, -d * sa],
            [sa * sq, cq * sa, ca, d * ca],
            [0, 0, 0, 1],
        ]
    )

    DK = transform @ DK

DK.evalf(subs={
    'theta_1': 0,
    'theta_2': 0,
    'theta_3': 0,
    'theta_4': 0,
    'theta_5': 0,
    'theta_6': 0,
    'theta_7': 0,
})

A = DK[0:3, 0:4]  # crop last row
A = A.transpose().reshape(12,1)  # reshape to column vector A = [a11, a21, a31, ..., a34]

Q = Matrix(joint_angles)
J = A.jacobian(Q)  # compute Jacobian symbolically

import numpy as np
from sympy import lambdify

A_lamb = lambdify((q1, q2, q3, q4, q5, q6, q7), A, 'numpy')
J_lamb = lambdify((q1, q2, q3, q4, q5, q6, q7), J, 'numpy')


def incremental_ik(q, A, A_final, step=0.1, atol=1e-4):
    while True:
        delta_A = (A_final - A)
        if np.max(np.abs(delta_A)) <= atol:
            break
        J_q = J_lamb(q[0,0], q[1,0], q[2,0], q[3,0], q[4,0], q[5,0], q[6,0])
        J_q = J_q / np.linalg.norm(J_q)  # normalize Jacobian
        
        # multiply by step to interpolate between current and target pose
        #print(J_q.shape, (delta_A*step).shape)
        #print((np.linalg.pinv(J_q) @ (delta_A*step)).shape)
        delta_q = np.linalg.pinv(J_q) @ (delta_A*step)
        
        q = q + delta_q
        A = A_lamb(q[0,0], q[1,0],q[2,0],q[3,0],q[4,0],q[5,0],q[6,0])
    return q, np.max(np.abs(delta_A))

# define joint limits for the Panda robot
limits = [
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973)
]

# create initial pose
q_init = np.array([l+(u-l)/2 for l, u in limits], dtype=np.float64).reshape(7, 1)
A_init = A_lamb(*(q_init.flatten()))
print(A_init.reshape(3, 4, order='F'))

np.random.seed(0)

q_rand = np.array([np.random.uniform(l, u) for l, u in limits], dtype=np.float64).reshape(7, 1)
A_final = A_lamb(*(q_rand).flatten())
print(A_final.reshape(3, 4, order='F'))

q_final, error = incremental_ik(q_init, A_init, A_final, atol=1e-6)
print(q_final.flatten(), error)