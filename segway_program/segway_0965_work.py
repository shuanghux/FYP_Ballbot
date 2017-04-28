#!/usr/bin/env python3

from __future__ import division
import subprocess
import time
import math
from collections import deque
#import numpy as np
#from numpy.linalg import inv


########################################################################
##
## File I/O functions
##
########################################################################

# Function for fast reading from sensor files
def FastRead(infile):
    infile.seek(0)
    value = int(infile.read().decode().strip())
    return (value)


# Function for fast writing to motor files
def FastWrite(outfile, value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush()


########################################################################
##
## Sensor Setup
##
########################################################################

# Make symlinks to sensors and motor for easy access from this python program
subprocess.call(['./makelinks.sh'])

# Open sensor files for (fast) reading
touchSensorValueRaw = open("ev3devices/in1/value0", "rb")
gyroSensorValueRaw = open("ev3devices/in2/value0", "rb")

# Set gyro to rate mode
with open('ev3devices/in2/mode', 'w') as f:
    f.write('GYRO-RATE')

########################################################################
##
## Motor Setup
##
########################################################################

# Open sensor files for (fast) reading
motorEncoderLeft = open("ev3devices/outD/position", "rb")
motorEncoderRight = open("ev3devices/outA/position", "rb")

# Open motor files for (fast) writing
motorDutyCycleLeft = open("ev3devices/outD/duty_cycle_sp", "w")
motorDutyCycleRight = open("ev3devices/outA/duty_cycle_sp", "w")


# Function to set the duty cycle of the motors
def SetDuty(motorDutyFileHandle, duty):
    # Clamp the value between -100 and 100
    duty = min(max(duty, -100), 100)
    # Apply the signal to the motor
    FastWrite(motorDutyFileHandle, duty)


# Reset the motors
with open('ev3devices/outA/command', 'w') as f:
    f.write('reset')
with open('ev3devices/outD/command', 'w') as f:
    f.write('reset')
time.sleep(0.01)

# Set motors in run-direct mode
with open('ev3devices/outA/command', 'w') as f:
    f.write('run-direct')
with open('ev3devices/outD/command', 'w') as f:
    f.write('run-direct')

########################################################################
##
## Definitions and Initialization variables
##
########################################################################


# Timing settings for the program
loopTimeMiliSec = 10  # Time of each loop, measured in miliseconds.
loopTimeSec = loopTimeMiliSec / 1000  # Time of each loop, measured in seconds.
motorAngleHistoryLength = 3  # Number of previous motor angles we keep track of.
loopCount = 0  # Loop counter, starting at 0

# Math constants
radiansPerDegree = math.pi / 180  # The number of radians in a degree.

# Platform specific constants and conversions
degPerSecondPerRawGyroUnit = 1  # For the LEGO EV3 Gyro in Rate mode, 1 unit = 1 deg/s
radiansPerSecondPerRawGyroUnit = degPerSecondPerRawGyroUnit * radiansPerDegree  # Express the above as the rate in rad/s per gyro unit
degPerRawMotorUnit = 1  # For the LEGO EV3 Large Motor 1 unit = 1 deg
radiansPerRawMotorUnit = degPerRawMotorUnit * radiansPerDegree  # Express the above as the angle in rad per motor unit
RPMperPerPercentSpeed = 1.7  # On the EV3, "1% speed" corresponds to 1.7 RPM (if speed control were enabled)
degPerSecPerPercentSpeed = RPMperPerPercentSpeed * 360 / 60  # Convert this number to the speed in deg/s per "percent speed"
radPerSecPerPercentSpeed = degPerSecPerPercentSpeed * radiansPerDegree  # Convert this number to the speed in rad/s per "percent speed"

# The rate at which we'll update the gyro offset (precise definition given in docs)
gyroDriftCompensationRate = 0.1 * loopTimeSec * radiansPerSecondPerRawGyroUnit

# A deque (a fifo array) which we'll use to keep track of previous motor positions, which we can use to calculate the rate of change (speed)
motorAngleHistory = deque([0], motorAngleHistoryLength)

# State feedback control gains (aka the magic numbers)
gainMotorAngle = 10  # For every radian we are ahead of the reference,           apply this amount of duty cycle
gainGyroAngle = 1482  # For every radian (57 degrees) we lean forward,            apply this amount of duty cycle.
gainMotorAngularSpeed = 19  # For every radian/s drive faster than the reference value, apply this amount of duty cycle
gainGyroRate = 180  # For every radian/s we fall forward,                       apply this amount of duty cycle.
gainMotorAngleErrorAccumulated = 2  # For every radian x s of accumulated motor angle,          apply this amount of duty cycle

# Variables representing physical signals (more info on these in the docs)
motorAngleRaw = 0  # The angle of "the motor", measured in raw units (degrees for the EV3). We will take the average of both motor positions as "the motor" angle, wich is essentially how far the middle of the robot has traveled.
motorAngle = 0  # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
motorAngleReference = 0  # The reference angle of the motor. The robot will attempt to drive forward or backward, such that its measured position equals this reference (or close enough).
motorAngleError = 0  # The error: the deviation of the measured motor angle from the reference. The robot attempts to make this zero, by driving toward the reference.
motorAngleErrorAccumulated = 0  # We add up all of the motor angle error in time. If this value gets out of hand, we can use it to drive the robot back to the reference position a bit quicker.
motorAngularSpeed = 0  # The motor speed, estimated by how far the motor has turned in a given amount of time
motorAngularSpeedReference = 0  # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
motorAngularSpeedError = 0  # The error: the deviation of the motor speed from the reference speed.
motorDutyCycle = 0  # The 'voltage' signal we send to the motor. We calulate a new value each time, just right to keep the robot upright.
gyroRateRaw = 0  # The raw value from the gyro sensor in rate mode.
gyroRate = 0  # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
gyroEstimatedAngle = 0  # The gyro doesn't measure the angle of the robot, but we can estimate this angle by keeping track of the gyroRate value in time
gyroOffset = 0  # Over time, the gyro rate value can drift. This causes the sensor to think it is moving even when it is perfectly still. We keep track of this offset.

########################################################################
##
## Declare Kalman parameters
##
########################################################################
dt = loopTimeSec
"""
sigma = 0.4
x_pre = np.matrix([[0], [0]])
F = np.matrix([[1, dt], [0, 1]])
u = 0
B = np.matrix([[dt * dt / 2], [dt]])
Q = np.matrix([[dt * dt * dt / 3, dt * dt / 2], [dt * dt / 2, dt]]) * sigma * sigma
R = 1 / 12 * np.eye(2)
H = np.eye(2)
I = np.eye(2)
K = np.matrix([[0.0724, 0],[0, 0.0724]])
P = np.matrix([[0.0065, 0],[0, 0.0065]])
S = np.matrix([[0.089, 0],[0, 0.089]])
#P_pre = np.matrix([[0.0065, 0], [0, 0.0065]])
"""
x_pre1, x_pre2 = 0, 0

########################################################################
##
## Calibrate Gyro
##
########################################################################

print("-----------------------------------")
print("Calibrating...")

# As you hold the robot still, determine the average sensor value of 100 samples
gyroRateCalibrateCount = 100
for i in range(gyroRateCalibrateCount):
    gyroOffset = gyroOffset + FastRead(gyroSensorValueRaw)
    time.sleep(0.01)
gyroOffset = gyroOffset / gyroRateCalibrateCount

# Print the result
print("GyroOffset: ", gyroOffset)
print("-----------------------------------")
print("GO!")
print("-----------------------------------")

########################################################################
##
## MAIN LOOP (Press Touch Sensor to stop the program)
##
########################################################################

# Initial touch sensor value
touchSensorPressed = FastRead(touchSensorValueRaw)

# Remember start time because we want to set a world record
tProgramStart = time.clock()

while not touchSensorPressed:

    ###############################################################
    ##  Loop info
    ###############################################################
    loopCount = loopCount + 1
    tLoopStart = time.clock()

    ###############################################################
    ##
    ##  Driving and Steering. Modify this section as you like to
    ##  make your segway go anywhere!
    ##
    ###############################################################

    # Read e.g. your PS2 controller here. Be sure you don't drag the loop too long

    # Or just balance in place:
    speed = 0
    steering = 0

    ###############################################################
    ##  Reading the Gyro.
    ###############################################################
    gyroRateRaw = FastRead(gyroSensorValueRaw)
    gyroRate = (gyroRateRaw - gyroOffset) * radiansPerSecondPerRawGyroUnit

    ###############################################################
    ##  Reading the Motor Position
    ###############################################################

    motorAngleRaw = (FastRead(motorEncoderLeft) + FastRead(motorEncoderRight)) / 2
    motorAngle = motorAngleRaw * radiansPerRawMotorUnit

    motorAngularSpeedReference = speed * radPerSecPerPercentSpeed
    motorAngleReference = motorAngleReference + motorAngularSpeedReference * loopTimeSec

    motorAngleError = motorAngle - motorAngleReference

    ###############################################################
    ##  Computing Motor Speed
    ###############################################################

    motorAngularSpeed = (motorAngle - motorAngleHistory[0]) / (motorAngleHistoryLength * loopTimeSec)
    motorAngularSpeedError = motorAngularSpeed - motorAngularSpeedReference;
    motorAngleHistory.append(motorAngle)
    """
    ###############################################################
    ## Kalman Filtering 1 unedited
    ###############################################################
    # Predict
    x_inter = F * x_pre
    P_inter = F * P_pre * F.T + Q#SM: can go
    # Update
    S = H * P_inter * H.T + R#SM: can go
    K = P_inter * H.T * np.linalg.inv(S)#SM: can go
    y = np.matrix([[gyroEstimatedAngle], [gyroRate]])
    x_curr = x_inter + K * (y - x_inter)
    P_curr = (I - K * H) * P_inter#SM: can go
    # shift
    x_pre = x_curr.copy()
    P_pre = P_curr.copy()#SM: can go
    """
    ###############################################################
    ## Kalman Filtering 1 edited WITHOUT USING NUMPY LIB
    ###############################################################
    # Predict
    x_inter1, x_inter2 = x_pre1 + (dt * x_pre2), x_pre2
    #x_inter = F * x_pre
    # Update
    #y = np.matrix([[gyroEstimatedAngle], [gyroRate]])
    #x_curr1, x_curr2 = x_inter1 + K * (y - x_inter)
    # In this case the kalman set to 0.965
    x_curr1, x_curr2 = x_inter1 + (0.965 * (gyroEstimatedAngle - x_inter1)), x_inter2 + (0.965 * (gyroRate - x_inter2))
    # shift
    x_pre1, x_pre2 = x_curr1, x_curr2
    gyroEstimatedAngle, gyroRate = x_curr1, x_curr2
    ###############################################################
    ##  Computing the motor duty cycle value
    ###############################################################

    motorDutyCycle = (gainGyroAngle * gyroEstimatedAngle
                      + gainGyroRate * gyroRate
                      + gainMotorAngle * motorAngleError
                      + gainMotorAngularSpeed * motorAngularSpeedError
                      + gainMotorAngleErrorAccumulated * motorAngleErrorAccumulated)

    ###############################################################
    ##  Apply the signal to the motor, and add steering
    ###############################################################

    SetDuty(motorDutyCycleRight, motorDutyCycle + steering)
    SetDuty(motorDutyCycleLeft, motorDutyCycle - steering)

    ###############################################################
    ##  Update angle estimate and Gyro Offset Estimate
    ###############################################################

    gyroEstimatedAngle = gyroEstimatedAngle + gyroRate * loopTimeSec
    gyroOffset = (1 - gyroDriftCompensationRate) * gyroOffset + gyroDriftCompensationRate * gyroRateRaw

    ###############################################################
    ##  Update Accumulated Motor Error
    ###############################################################

    motorAngleErrorAccumulated = motorAngleErrorAccumulated + motorAngleError * loopTimeSec

    ###############################################################
    ##  Read the touch sensor (the kill switch)
    ###############################################################

    touchSensorPressed = FastRead(touchSensorValueRaw)

    ###############################################################
    ##  Busy wait for the loop to complete
    ###############################################################

    while (time.clock() - tLoopStart < loopTimeSec):
        time.sleep(0.0001)

        ########################################################################
##
## Closing down & Cleaning up
##
########################################################################

# See if we have that world record
tProgramEnd = time.clock()

# Turn off the motors
FastWrite(motorDutyCycleLeft, 0)
FastWrite(motorDutyCycleRight, 0)

# Calculate loop time
tLoop = (tProgramEnd - tProgramStart) / loopCount
print("Loop time:", tLoop * 1000, "ms")

# Print a stop message
print("-----------------------------------")
print("STOP")
print("-----------------------------------")
