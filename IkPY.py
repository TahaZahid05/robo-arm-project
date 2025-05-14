from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from math import radians
import numpy as np

# Adjust lengths to match your robot
l1 = 0.13
l2 = 0.089
l4 = 0.035
l5 = 0.13

robot_chain = Chain(name='5DOF_Arm', links=[
    OriginLink(),  # Base link

    # Joint 1: Rotates around Z
    URDFLink(
        name="joint1",
        origin_translation=[0, 0, l1],  # Translation from the previous link
        origin_orientation=[radians(90), 0, 0],  # Initial orientation (Euler angles)
        rotation=[0, 0, 1],  # Axis of rotation (Z)
    ),
    # Joint 2: Rotates around Y
    URDFLink(
        name="joint2",
        origin_translation=[l2, 0, 0],
        origin_orientation=[0, 0, 0],  # No initial rotation
        rotation=[0, 1, 0],  # Axis of rotation (Y)
    ),
    # Joint 3: Rotates around Z
    URDFLink(
        name="joint3",
        origin_translation=[0, 0, 0],
        origin_orientation=[radians(90), 0, 0],
        rotation=[0, 0, 1],
    ),
    # Joint 4: Rotates around Y
    URDFLink(
        name="joint4",
        origin_translation=[0, 0, l4],
        origin_orientation=[radians(-90), 0, 0],
        rotation=[0, 1, 0],
    ),
    # Joint 5: Gripper joint
    URDFLink(
        name="joint5",
        origin_translation=[l5, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    ),
])

target_position = [0.15, 0.05, 0.2]
joint_angles = robot_chain.inverse_kinematics(target_position)
joint_angles = np.mod(joint_angles + np.pi, 2*np.pi) - np.pi  # Normalize to [-π, π]
joint_angles_degrees = np.degrees(joint_angles[1:])  # Exclude base link

print("Joint Angles (degrees):", joint_angles_degrees)