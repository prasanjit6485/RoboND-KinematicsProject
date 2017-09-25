from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix

### Create symbols for joint variables
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

a12 = 0.35     # meters from urdf file
a23 = 1.25     # meters from urdf file
a34 = -0.054   # meters from urdf file

d01 = 0.75     # meters from urdf file
d34 = 1.50     # meters from urdf file
d67 = 0.303    # meters from urdf file

# DH Parameters
s = {alpha0:     0,  a0:   0, d1: d01, 
    alpha1: -pi/2,  a1: a12, d2:   0, q2: q2 -pi/2, 
    alpha2:     0,  a2: a23, d3:   0, 
    alpha3: -pi/2,  a3: a34, d4: d34,
    alpha4:  pi/2,  a4:   0, d5:   0,
    alpha5: -pi/2,  a5:   0, d6:   0,
    alpha6:     0,  a6:   0, d7: d67, q7: 0}

#### Homogeneous Transforms
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
              [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
              [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
              [                   0,                   0,            0,               1]])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
              [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
              [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
              [                   0,                   0,            0,               1]])
T1_2 = T1_2.subs(s)

T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
              [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
              [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
              [                   0,                   0,            0,               1]])
T2_3 = T2_3.subs(s)

T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
              [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
              [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
              [                   0,                   0,            0,               1]])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
              [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
              [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
              [                   0,                   0,            0,               1]])
T4_5 = T4_5.subs(s)

T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
              [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
              [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
              [                   0,                   0,            0,               1]])
T5_6 = T5_6.subs(s)

T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
              [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
              [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
              [                   0,                   0,            0,               1]])
T6_7 = T6_7.subs(s)

# Transform from base link to end effector
# T0_7 = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7)
# T0_2 = simplify(T0_1 * T1_2)
# T0_3 = simplify(T0_2 * T2_3)
# T0_4 = simplify(T0_3 * T3_4)
# T0_5 = simplify(T0_4 * T4_5)
# T0_6 = simplify(T0_5 * T5_6)
# T0_7 = simplify(T0_6 * T6_7)

T3_6 = simplify(T3_4 * T4_5 * T5_6)

print(T3_6)

T1_4 = simplify(T1_2 * T2_3 * T3_4)

print(T1_4)

# Compute inverse of R0_3 rotation matrix
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3_inv = R0_3.inv()

print(R0_3_inv)