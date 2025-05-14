import math

def calculate_thetas(matrix):
    # Given link lengths in cm
    L1 = 11.7
    L2 = 12.7
    L3 = 8.89
    L5 = 3.5

    # Extract matrix entries
    r = matrix

    # Calculate theta1 using r14 and r24
    theta1 = math.atan2(r[0][3], r[1][3])

    # Precompute sin and cos of theta1
    sin_theta1 = math.sin(theta1)
    cos_theta1 = math.cos(theta1)

    print(sin_theta1,cos_theta1)
    # Calculate theta5 using theta1, r11, r21, r12, r22
    numerator_theta5 = sin_theta1 * r[0][0] - cos_theta1 * r[1][0]
    denominator_theta5 = sin_theta1 * r[0][1] - cos_theta1 * r[1][1]
    print(numerator_theta5)
    print(denominator_theta5)
    theta5 = math.atan2(numerator_theta5, denominator_theta5)

    # Calculate theta2 using theta1 and theta5
    tan_theta5 = math.tan(theta5)
    term1_theta2_num = cos_theta1 * r[0][1] + sin_theta1 * r[1][1]
    term2_theta2_num = tan_theta5 * (cos_theta1 * r[0][0] + sin_theta1 * r[1][0])
    numerator_theta2 = -(term1_theta2_num + term2_theta2_num)
    denominator_theta2 = r[0][2] * tan_theta5 + r[2][1]
    theta2 = math.atan2(numerator_theta2, denominator_theta2)

    # Precompute sin and cos of theta2
    sin_theta2 = math.sin(theta2)
    cos_theta2 = math.cos(theta2)

    # Calculate rho for theta3
    term_rho1 = cos_theta1 * r[0][3] + sin_theta1 * r[1][3]
    term_rho2 = L1 - r[2][3]
    rho = cos_theta2 * term_rho1 - sin_theta2 * term_rho2 - L2

    # Calculate lambda for theta3
    lambda_ = -cos_theta2 * term_rho2 - sin_theta2 * term_rho1

    # Calculate beta for theta3
    term_beta = cos_theta1 * r[0][2] + sin_theta1 * r[1][2]
    beta = sin_theta2 * r[2][2] + cos_theta2 * term_beta

    # Calculate S for theta3
    term_S = cos_theta1 * r[0][2] + sin_theta1 * r[1][2]
    S = cos_theta2 * r[2][2] - sin_theta2 * term_S

    # Calculate theta3
    numerator_theta3 = (lambda_ - L5 * S) / L3
    denominator_theta3 = (rho - L5 * beta) / L3
    theta3 = math.atan2(numerator_theta3, denominator_theta3)

    # Calculate theta4
    term_theta4 = cos_theta1 * r[0][2] + sin_theta1 * r[1][2]
    theta4 = math.atan2(r[2][2], term_theta4) - theta3 - theta2

    # Convert all angles from radians to degrees
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)
    theta3_deg = math.degrees(theta3)
    theta4_deg = math.degrees(theta4)
    theta5_deg = math.degrees(theta5)

    return theta1_deg, theta2_deg, theta3_deg, theta4_deg, theta5_deg

print(calculate_thetas([[1,0,0,3],[0,1,0,3],[0,0,1,3],[0,0,0,1]]))