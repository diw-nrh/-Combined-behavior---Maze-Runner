"""avoid_the_wall controller with odometry and return-to-home logic."""

import math
from controller import Robot, DistanceSensor, Motor

# Create the Robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0 # Time step in seconds

# --- Robot Parameters (for e-puck robot) ---
WHEEL_RADIUS = 0.0205  # Wheel radius in meters
AXLE_LENGTH = 0.052    # Distance between wheels in meters
MAX_SPEED = 6.28       # Maximum angular speed (rad/s)

# --- Position State Variables ---
x_pos = 0.0
y_pos = 0.0
theta = 0.0 # Heading angle in radians

# --- ส่วนที่เพิ่ม: ตัวแปรสำหรับเช็คการกลับบ้าน ---
home_threshold = 0.1  # ระยะห่างจากจุดเริ่มต้นที่จะถือว่า "ถึงแล้ว" (0.1 เมตร)
has_left_start_zone = False # สถานะว่าหุ่นยนต์เคยออกจากจุดเริ่มต้นแล้วหรือยัง

# Initialize distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# Initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# --- USER'S ORIGINAL FUNCTION (UNCHANGED) ---
def follow_wall(ps_values):
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED
    if ps_values[0] > 100 or ps_values[7] > 100 or ps_values[1] > 100:
        left_speed = 0
        right_speed = 0.5 * MAX_SPEED
    elif ps_values[2] > 150:
        left_speed = MAX_SPEED * 0.2
        right_speed = MAX_SPEED
    elif ps_values[2] < 100:
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED * 0.2
    return left_speed, right_speed

# Main loop
while robot.step(timestep) != -1:
    # Read sensor values
    ps_values = [ps[i].getValue() for i in range(8)]
    
    # Process sensor data and get motor speeds
    left_speed, right_speed = follow_wall(ps_values)
    
    # --- Odometry Calculation ---
    v_left = left_speed * WHEEL_RADIUS
    v_right = right_speed * WHEEL_RADIUS
    v = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / AXLE_LENGTH
    theta += omega * dt
    x_pos += v * math.cos(theta) * dt
    y_pos += v * math.sin(theta) * dt
    
    # Print the current position periodically
    if robot.getTime() % 0.2 < dt:
        print(f"Position: (X={x_pos:.3f} m, Y={y_pos:.3f} m), Heading: {math.degrees(theta):.2f}°")
    
    # --- ส่วนที่เพิ่ม: ตรวจสอบการกลับมายังจุดเริ่มต้น --- 🎯
    distance_from_home = math.sqrt(x_pos**2 + y_pos**2)

    # 1. เช็คว่าหุ่นยนต์ได้เคลื่อนที่ออกจากจุดเริ่มต้นไปแล้วหรือยัง
    if not has_left_start_zone and distance_from_home > home_threshold * 2:
        has_left_start_zone = True
        print("Robot has left the starting zone. Return-to-home is now active.")

    # 2. ถ้าเคยออกจากจุดเริ่มต้นแล้ว และกลับเข้ามาในระยะที่กำหนด ให้หยุด
    if has_left_start_zone and distance_from_home < home_threshold:
        print(f"🏁 Returned to starting point (distance: {distance_from_home:.3f} m). Stopping.")
        left_speed = 0
        right_speed = 0
        leftMotor.setVelocity(left_speed)
        rightMotor.setVelocity(right_speed)
        break # ออกจาก while loop เพื่อหยุดการทำงาน

    # Set motor velocities
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)

# Cleanup code is now handled before breaking the loop
print("Controller has finished.")