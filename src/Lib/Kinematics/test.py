import math

def scara_inverse_kinematics(x, y, z, theta):
    # SCARA robot parameters
    l1 = 1.0  # length of the first arm
    l2 = 1.0  # length of the second arm
    d3 = 0.5  # length of the prismatic joint

    # Convert the angle to radians
    theta = math.radians(theta)

    # Calculate the wrist position
    wx = x - l2 * math.cos(theta)
    wy = y - l2 * math.sin(theta)
    wz = z - d3

    # Calculate the first joint angle
    q1 = math.atan2(wy, wx)

    # Calculate the distance from the first joint to the wrist
    r = math.sqrt(wx ** 2 + wy ** 2)

    # Calculate the second joint angle
    cos_q2 = (r ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    sin_q2 = math.sqrt(1 - cos_q2 ** 2)
    q2 = math.atan2(sin_q2, cos_q2)

    # Calculate the third joint translation
    d3 = wz

    # Calculate the fourth joint angle
    q4 = theta - q1 - q2

    # Convert the joint angles to degrees
    q1 = math.degrees(q1)
    q2 = math.degrees(q2)
    q4 = math.degrees(q4)

    return q1, q2, q4, d3

# Test the inverse kinematics function
x = 2.0
y = 1.5
z = 0.75
theta = 45.0

q1, q2, q4, d3 = scara_inverse_kinematics(x, y, z, theta)
print(f"Joint angles: q1 = {q1:.2f} degrees, q2 = {q2:.2f} degrees, q4 = {q4:.2f} degrees")
print(f"Joint translation: d3 = {d3:.2f} units")
