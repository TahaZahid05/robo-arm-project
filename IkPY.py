from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from math import radians, degrees
import numpy as np

# Define link lengths (in meters)
l1 = 0.13
l2 = 0.089
l4 = 0.035
l5 = 0.13

# Build the robot arm chain
robot_chain = Chain(name='5DOF_Arm', links=[
    OriginLink(),

    URDFLink(
        name="joint1",
        origin_translation=[0, 0, l1],
        origin_orientation=[radians(90), 0, 0],
        rotation=[0, 1, 0],
    ),
    URDFLink(
        name="joint2",
        origin_translation=[l2, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
    ),
    URDFLink(
        name="joint3",
        origin_translation=[0, 0, 0],
        origin_orientation=[radians(90), 0, 0],
        rotation=[0, 0, 1],
    ),
    URDFLink(
        name="joint4",
        origin_translation=[0, 0, l4],
        origin_orientation=[radians(90), 0, 0],
        rotation=[0, 0, 1],
    ),
    URDFLink(
        name="joint5",
        origin_translation=[l5, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
    ),
])

# Target end-effector position (X, Y, Z)
target_position = [0.15, 0.05, 0.2]

# Perform inverse kinematics
joint_angles = robot_chain.inverse_kinematics(target_position)

# Normalize angles to [-π, π] then convert to degrees
joint_angles = np.mod(joint_angles + np.pi, 2 * np.pi) - np.pi
joint_angles_deg = np.clip(np.degrees(joint_angles[1:]), 0, 180)  # exclude base

# Print results
print("Servo Joint Angles (0–180°):")
for i, angle in enumerate(joint_angles_deg, 1):
    print(f"Theta{i}: {angle:.2f}°")
