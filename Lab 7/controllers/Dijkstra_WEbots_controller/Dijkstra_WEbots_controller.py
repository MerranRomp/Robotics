from controller import Robot
import numpy as np

#-------------------------------------------------------
# Open serial port to communicate with the microcontroller

import serial
try:
    # Change the port parameter according to your system
    ser =  serial.Serial(port='COM7', baudrate=115200, timeout=5) 
except:
    print("Communication failed. Check the cable connections and serial settings 'port' and 'baudrate'.")
    raise
    
#-------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED

# create the Robot instance for the simulation.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]

# states
states = ['forward', 'turn_right', 'turn_left', 'stop']
current_state = 'forward'

#-------------------------------------------------------
# Initialize devices

# proximity sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#-------------------------------------------------------
# Initialize GPS device
gps = robot.getDevice('gps')
gps.enable(timestep)
# Initialize Inertial Unit device
inertial_unit = robot.getDevice("inertial unit")
inertial_unit.enable(timestep)

#-------------------------------------------------------
# Main loop:

while robot.step(timestep) != -1: #timestep

    ############################################
    #                  See                     #
    ############################################

    # Update sensor readings
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    # Process sensor data
    line_right = gsValues[0] > 600
    line_center = gsValues[1] > 600
    line_left = gsValues[2] > 600
    
    # Read GPS position
    position = gps.getValues()
    x, y, z = position
    #read inertial position
    orientation = inertial_unit.getRollPitchYaw()
    yaw = orientation[2]  # yaw is heading (rotation around Z)
    
    # Build the message to be sent to the ESP32 with the ground
    # sensor data: 0 = line detected; 1 = line not detected
    # Build the line sensor message
    line_state = ''
    line_state += '1' if line_left else '0'
    line_state += '1' if line_center else '0'
    line_state += '1' if line_right else '0'
    
    
    # Construct message: line sensors + GPS (x, y)
    message = f"{line_state},{x:.2f},{y:.2f},{yaw:.2f}\n"
    msg_bytes = bytes(message, 'UTF-8')
    

    ############################################
    #                 Think                    #
    ############################################

    # Serial communication: if something is received, then update the current state
    if ser.in_waiting:
        value = str(ser.readline(), 'UTF-8').strip()
        print("Received from ESP32:", value)
        current_state = value

    # Update speed according to the current state
    if current_state == 'forward':
        leftSpeed = speed
        rightSpeed = speed
            
    if current_state == 'turn_right':
        leftSpeed = 1 * speed
        rightSpeed = -1 * speed

    if current_state == 'turn_left':
        leftSpeed = -1 * speed
        rightSpeed = 1 * speed
        
    if current_state == 'stop':
        leftSpeed = 0.0
        rightSpeed = 0.0
 

    ############################################
    #                  Act                     #
    ############################################

    # Update velocity commands for the motors
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
   
    # Print sensor message and current state for debugging
    print(f"Sensor message: {msg_bytes} - Current state: {current_state} - GPS position: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
    # Send message to the microcontroller 
    ser.write(msg_bytes)  

ser.close()