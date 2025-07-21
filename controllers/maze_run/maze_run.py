"""avoid_the_wall controller."""

from controller import Robot, DistanceSensor, Motor

# Create the Robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Initialize distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']

# Initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Enable sensors
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# Robot parameters
MAX_SPEED = 6.28  # Maximum angular speed (rad/s)

def follow_wall(ps_values):
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED
    if ps_values[0] < 100 and ps_values[7] < 100 and ps_values[2]<100 and ps_values[3]<100:
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
    elif ps_values[0] > 150:
        left_speed = -MAX_SPEED * 0.5
        right_speed = MAX_SPEED
    elif ps_values[0]<150:
        left_speed = MAX_SPEED
        right_speed = -MAX_SPEED * 0.5

    return left_speed, right_speed
# Main loop
while robot.step(timestep) != -1:
    # Read sensor values
    ps_values = [ps[i].getValue() for i in range(8)]
    
    # Process sensor data and get motor speeds
    left_speed, right_speed = follow_wall(ps_values)
    
    # Set motor velocities
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)

# Cleanup code
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)