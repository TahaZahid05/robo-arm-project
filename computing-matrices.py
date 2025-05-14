import sympy as sp

# 1) Symbols
theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta1 theta2 theta3 theta4 theta5')
l1, l2, l4, l5 = sp.symbols('l1 l2 l4 l5')
r11, r12, r13, r14 = sp.symbols('r11 r12 r13 r14')
r21, r22, r23, r24 = sp.symbols('r21 r22 r23 r24')
r31, r32, r33, r34 = sp.symbols('r31 r32 r33 r34')

# 2) Basic transforms
def rot_z(theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0, 0],
        [sp.sin(theta),  sp.cos(theta), 0, 0],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])
def trans_z(d):
    return sp.Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
def trans_x(a):
    return sp.Matrix([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
def rot_x(alpha):
    return sp.Matrix([
        [1, 0,           0,          0],
        [0, sp.cos(alpha), -sp.sin(alpha), 0],
        [0, sp.sin(alpha),  sp.cos(alpha), 0],
        [0, 0,           0,          1]
    ])


# Transformation matrices from your robot configuration
T01 = rot_z(theta1) * trans_z(l1) * rot_x(sp.pi/2)
T12 = rot_z(theta2) * trans_x(l2)
T23 = rot_z(theta3 + sp.pi/2) * rot_x(sp.pi/2)
T34 = rot_z(theta4) * trans_z(l4) * rot_x(sp.pi/2)  # Updated T34
T45 = rot_z(theta5) * trans_x(l5)
# Full transformation matrix T05
T05 = sp.simplify(T01 * T12 * T23 * T34 * T45)

# 5) Generic configuration matrix C
C = sp.Matrix([
    [r11, r12, r13, r14],
    [r21, r22, r23, r24],
    [r31, r32, r33, r34],
    [  0,   0,   0,   1 ]
])

# 6) Inverse of T01
T01_inv = sp.simplify(T01.inv())

# sp.pprint(T01_inv)

# 7) Compute T01_inv * T05 and T01_inv * C
result_T01_inv_T05 = sp.simplify(T01_inv * T05)
result_T01_inv_C   = sp.simplify(T01_inv * C)

# # 8) Display
# print("T01⁻¹ * T05 =")
# sp.pprint(result_T01_inv_T05)

# print("\nT01⁻¹ * C =")
# sp.pprint(result_T01_inv_C)

T12_inv = sp.simplify(T12.inv())

final_result_left = sp.simplify(T12_inv * result_T01_inv_T05)
final_result_right = sp.simplify(T12_inv * result_T01_inv_C)

sp.pprint(T12_inv)

print("T12^-1 * T01⁻¹ * T05 =")
sp.pprint(final_result_left)

print("\nT12^-1 * T01⁻¹ * C =")
sp.pprint(final_result_right)