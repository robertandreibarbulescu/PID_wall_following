# -*- coding: utf-8 -*-
"""
Created on 30th November 2020

@author: Robert B.
"""

"""
Make sure to have the server side running in V-REP by having the following command executed just once, at simulation start:

simRemoteApi.start(19999)

then start simulation, and run this program.

IMPORTANT: for each successful call to simxStart, there should be a corresponding call to simxFinish at the end!
"""

try:
    import sim as vrep      #import as vrep for older versions;
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import math
import sys
import random               #needed in order to perform the random wander;
import matplotlib as mpl    #used for image plotting;
         
class Robot():

    def __init__(self):

        # Setup Motors
        res, self.leftMotor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking)
        res, self.rightMotor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking)

        # Setup Sonars

        res, self.frontLeftSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor6',vrep.simx_opmode_blocking)
        res, self.frontRightSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1',vrep.simx_opmode_blocking)
        res, self.sideSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor0',vrep.simx_opmode_blocking)
        res, self.rightSonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor15',vrep.simx_opmode_blocking)

        
        #We are using both the back and front sensors in order to avoid obstacles;

        # Start Sonars
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,self.frontLeftSonar,vrep.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,self.frontRightSonar,vrep.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,self.rightSonar,vrep.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,self.sideSonar,vrep.simx_opmode_streaming)

        time.sleep(2)

        #Starting sensors, front and back;

    def getDistanceReading(self, objectHandle):
        # Get reading from sensor
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,objectHandle,vrep.simx_opmode_buffer)

        if detectionState == 1:
            # return magnitude of detectedPoint
            return math.sqrt(sum(i**2 for i in detectedPoint))
        else:
            # return another value that we know cannot be true and handle it (use a large number so that if you do 'distance < reading' it will work)
            return 9999

    def stop(self):
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, 0, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, 0, vrep.simx_opmode_blocking)

    def turn(self, turnVelocity):
        # turnVelocity < 0 = turn left
        # turnVelocity > 0 = turn right
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, turnVelocity, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, -turnVelocity, vrep.simx_opmode_blocking)

    def move(self, velocity):
        # velocity < 0 = reverse
        # velocity > 0 = forward
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, velocity, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, velocity, vrep.simx_opmode_blocking)
        
    def turnArc(self, leftMotorVelocity, rightMotorVelocity):
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, leftMotorVelocity, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, rightMotorVelocity, vrep.simx_opmode_blocking)

        
print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')
    
    robot = Robot()
    # for loop to retrieve readings from the sensors we will be using for obstacles avoidance;
    for i in range(10):
        print(robot.getDistanceReading(robot.frontLeftSonar))
        print(robot.getDistanceReading(robot.frontRightSonar))
        print(robot.getDistanceReading(robot.rightSonar))
        print(robot.getDistanceReading(robot.sideSonar))

        time.sleep(0.2)

    # Function used to returnn a random integer within the range of 0 and 100 for the random wander;
    randomNumber = random.randint(0, 100)     
    
    # PID Controller constants;
    Kp = 00656 # Proportional gain;
    Kd = 0.0002   # Derivative gain;
    Ki = 0.0   # Integral gain
    dt = 0.2    # Time constant;

    # Variables initialization;
    cp = 0
    current_error = 0
    previous_error = 0
    sum_error = 0
    previous_error_derivative = 0
    current_error_derivative = 0

    
    # Distance variables initialization;
    d1 = robot.getDistanceReading(robot.sideSonar)
    d2 = robot.getDistanceReading(robot.rightSonar)


    dist = d1+d2/2              # distance between the robot and the wall;
    angle = d1 - d2             # angle with the wall;
    dist_to_wall = math.cos(angle) == d1/d2        # desired distance between robot and the wall;

    # Loop execution (nested while statement);
    while True:
        # the robot will move untill it detects an object;
        robot.move(1)
        
        res,detectionStateR,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,robot.rightSonar,vrep.simx_opmode_buffer)
    
        current_error = dist_to_wall - dist

        sum_error += sum_error + current_error*dt
    
        
        current_error_derivative = (current_error - previous_error) / dt
        
        previous_error = current_error
       
        # PID Controller 
        cp += Kp * current_error + Ki * sum_error + Kd * current_error_derivative
        

        # testing
        # cp = max(min(0, cp), 1) # uncomment for limiting the output for values between 0 and 1;
        
        # cp = abs (cp)   # used to make any output value a positive value;
        
        print("error {} cp {} distance {}".format(current_error, cp, dist))
        # if.. elif ..else statements that allows us to check for multiple expressions;
        if robot.getDistanceReading(robot.frontLeftSonar) <= dist_to_wall:       # threshold value
            robot.turnArc(-cp, cp)                                # velocity adjustment 
            print("Wall detected in front")
            # the robot will trun if a wall is detected;
            
        elif robot.getDistanceReading(robot.frontRightSonar) <= dist_to_wall:    # threshold value
            robot.turnArc(-cp, cp)                                # velocity adjustment 
            print("Wall detected in front")
            # the robot will trun if a wall is detected;
            
            
        elif robot.getDistanceReading(robot.rightSonar) <= dist_to_wall and detectionStateR==1:       # threshold value
            robot.turnArc(cp, angle)                          # velocity adjustment 
            print("Change direction to the Right")
            # the robot will turn if object detected on the right within the threshold value;
            
        elif robot.getDistanceReading(robot.rightSonar) >= dist_to_wall and detectionStateR==1:         # threshold value
            robot.turnArc(angle, cp)                              # velocity adjustment 
            print("Maintain positionon on the Left")
            # the robot will turn to adjust direction;
            
        elif robot.getDistanceReading(robot.rightSonar) >= dist_to_wall:         # threshold value
            robot.turnArc(cp, cp)                              # velocity adjustment 
            print("Maintain positionon on the Left")
            # the robot will turn to adjust direction;

        elif robot.getDistanceReading(robot.rightSonar) == dist_to_wall  and robot.getDistanceReading(robot.sideSonar) == dist_to_wall and detectionStateR==1:         # threshold value
            robot.move(1)                              # velocity adjustment 
            print("Forward Maintain Velocity")
            # the robot will turn to adjust direction;     
            
            
            
        elif robot.getDistanceReading(robot.frontRightSonar) <= dist_to_wall and detectionStateR==1:       # threshold value
            robot.turnArc(cp, angle)                          # velocity adjustment 
            print("Change direction to the Right")
            # the robot will turn if object detected on the right within the threshold value;
            
        elif robot.getDistanceReading(robot.frontRightSonar) >= dist_to_wall  and detectionStateR==1:         # threshold value
            robot.turnArc(angle, cp)                              # velocity adjustment 
            print("Maintain positionon on the Left")
            # the robot will turn to adjust direction;
            
        elif robot.getDistanceReading(robot.frontRightSonar) >= dist_to_wall:         # threshold value
            robot.turnArc(cp, cp)                              # velocity adjustment 
            print("Maintain positionon on the Left")
            # the robot will turn to adjust direction;
            


        elif robot.getDistanceReading(robot.sideSonar) <= dist_to_wall and detectionStateR==1:       # threshold value
            robot.turnArc(cp, angle)                          # velocity adjustment 
            print("Change direction to the Right")
            # the robot will turn if object detected on the right within the threshold value;
            
        elif robot.getDistanceReading(robot.sideSonar) >= dist_to_wall  and detectionStateR==1:         # threshold value
            robot.turnArc(angle, cp)                              # velocity adjustment 
            print("Maintain positionon on the Left")
            # the robot will turn to adjust direction;
            
        elif robot.getDistanceReading(robot.sideSonar) >= dist_to_wall:         # threshold value
            robot.turnArc(cp, cp)                              # velocity adjustment 
            print("Maintain positionon on the Left")
            # the robot will turn to adjust direction;
            
        elif robot.getDistanceReading(robot.sideSonar) == dist_to_wall and robot.getDistanceReading(robot.rightSonar) == dist_to_wall and detectionStateR==1:         # threshold value
            robot.move(1)                              # velocity adjustment 
            print("Forward Maintain Velocity")
            # the robot will turn to adjust direction;

            
        else:
            robot.turnArc(randomNumber - 1, randomNumber + 1)
            # The robot will take a random turn untill it detects an object;


    robot.stop()

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)
    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
sys.exit('Could not connect')











