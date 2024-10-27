from enum import IntEnum
import numpy as np
from utils import constrain, PID, ExpMovingAverageFilter, map_value

class Axes(IntEnum):
    FORWARD = 0
    STRAFE  = 1
    DEPTH   = 2
    ROLL    = 3
    PITCH   = 4    
    YAW     = 5

class ThrustersNames(IntEnum):
    # Y-frame specific
    H_FRONT_LEFT    = 0
    H_FRONT_RIGHT   = 1
    H_REAR          = 2
    V_FRONT_LEFT    = 3
    V_FRONT_RIGHT   = 4
    V_REAR          = 5

class ControlSystem:
    def __init__(self):
        # Thrusters calibration values
        self.__thrustersDirCorr = [1, 1, 1, 1, 1, 1]
        self.__thrustersOrder = [ThrustersNames.H_FRONT_LEFT, 
                                 ThrustersNames.H_FRONT_RIGHT,
                                 ThrustersNames.H_REAR, 
                                 ThrustersNames.V_FRONT_LEFT,
                                 ThrustersNames.V_FRONT_RIGHT,
                                 ThrustersNames.V_REAR]
        self.__trustersXValues = [-100, 100]
        self.__thrusterIncrement = 0.5
        # (forward,strafe,depth,roll,pitch,yaw)
        self.__axesInputs   = [0, 0, 0, 0, 0, 0]
        self.__axesValues   = [0, 0, 0, 0, 0, 0]        
        self.__stabs        = [0, 0, 0, 0, 0, 0]
        self.__PIDValues    = [0, 0, 0, 0, 0, 0]
        self.__PIDs = [None, 
                       None, 
                       PID(10, 0, 0, 0),
                       PID(10, 0, 0, 0),
                       PID(10, 0, 0, 0),
                       PID(10, 0, 0, 0)]
        self.__filters = [None, 
                          None, 
                          ExpMovingAverageFilter(0.8),
                          None, 
                          None, 
                          None]
        # (hor1,hor2,hor3,ver1,ver2,ver3)
        self.__thrustersOutputsSetpoints    = [0, 0, 0, 0, 0, 0]
        self.__thrustersOutputs             = [0, 0, 0, 0, 0, 0]
        # Operating period
        self.__dt = 1/500

    def __updateControl(self):
        # Update PIDs calculations
        self.__updatePID()
        # Calculate thrusters control values
        self.__calculateHorizontalThrust()
        self.__calculateVerticalThrust()
        # Calibrate values
        self.__thrustersCalibrate()
        # Make thrusters build up speed smoothly
        self.__smoothRPMBuildUp()

    def __updatePID(self):
        if not ((np.abs(self.__axesInputs[Axes.DEPTH])<1) and self.__stabs[Axes.DEPTH]):
            self.setPIDSetpoint(Axes.DEPTH, self.__axesValues[Axes.DEPTH])
        
        if np.abs(self.__axesInputs[Axes.YAW]) >= 1 or not self.__stabs[Axes.YAW] : 
            self.setPIDSetpoint(Axes.YAW, self.__axesValues[Axes.YAW])

        yawPID = self.__PIDs[Axes.YAW].update(self.__axesValues[Axes.YAW], self.__dt) if (np.abs(self.__axesInputs[Axes.YAW]) < 1) and self.__stabs[Axes.YAW] else 0 
        rollPID = self.__PIDs[Axes.ROLL].update(self.__axesValues[Axes.ROLL], self.__dt) if self.__stabs[Axes.ROLL] else 0
        pitchPID = self.__PIDs[Axes.PITCH].update(self.__axesValues[Axes.PITCH], self.__dt) if self.__stabs[Axes.PITCH] else 0
        depthPID = -self.__PIDs[Axes.DEPTH].update(self.__axesValues[Axes.DEPTH], self.__dt) if self.__stabs[Axes.DEPTH] else 0

        self.__PIDValues[Axes.YAW] = constrain(yawPID, -100, 100)
        self.__PIDValues[Axes.ROLL] = constrain(rollPID, -100, 100)
        self.__PIDValues[Axes.PITCH] = constrain(pitchPID, -100, 100)
        self.__PIDValues[Axes.DEPTH] = constrain(depthPID, -100, 100)

    def __calculateHorizontalThrust(self):

        # HFL = self.__axesInputs[ControlAxes.STRAFE] / 2 + np.sqrt(3) * self.__axesInputs[ControlAxes.FORWARD] / 2 + 0.32 * (self.__axesInputs[ControlAxes.YAW] + self.__PIDValues[ControlAxes.YAW])
        # HFR = - self.__axesInputs[ControlAxes.STRAFE] / 2 + np.sqrt(3) * self.__axesInputs[ControlAxes.FORWARD]  / 2 - 0.32 * (self.__axesInputs[ControlAxes.YAW] + self.__PIDValues[ControlAxes.YAW])
        # HRR = - self.__axesInputs[ControlAxes.STRAFE] + 0.32 * (self.__axesInputs[ControlAxes.YAW] + self.__PIDValues[ControlAxes.YAW])
        # alt thrust calculatation
        HFL = self.__axesInputs[Axes.STRAFE] / 2 + self.__axesInputs[Axes.FORWARD] + 0.32 * (self.__axesInputs[Axes.YAW] + self.__PIDValues[Axes.YAW])
        HFR = - self.__axesInputs[Axes.STRAFE] / 2 + self.__axesInputs[Axes.FORWARD] - 0.32 * (self.__axesInputs[Axes.YAW] + self.__PIDValues[Axes.YAW])
        HRR = - self.__axesInputs[Axes.STRAFE] + 0.32 * (self.__axesInputs[Axes.YAW] + self.__PIDValues[Axes.YAW])

        self.__thrustersOutputsSetpoints[self.__thrustersOrder.index(ThrustersNames.H_FRONT_LEFT)] = constrain(HFL, -100, 100)
        self.__thrustersOutputsSetpoints[self.__thrustersOrder.index(ThrustersNames.H_FRONT_RIGHT)] = constrain(HFR, -100, 100)
        self.__thrustersOutputsSetpoints[self.__thrustersOrder.index(ThrustersNames.H_REAR)] = constrain(HRR, -100, 100)
        
    def __calculateVerticalThrust(self):
        VFL = self.__PIDValues[Axes.ROLL] + self.__PIDValues[Axes.DEPTH] + self.__PIDValues[Axes.PITCH] + self.__axesInputs[Axes.DEPTH]
        VFR = -self.__PIDValues[Axes.ROLL] + self.__PIDValues[Axes.DEPTH] + self.__PIDValues[Axes.PITCH] + self.__axesInputs[Axes.DEPTH]
        VRR = -self.__PIDValues[Axes.PITCH] + self.__PIDValues[Axes.DEPTH] + self.__axesInputs[Axes.DEPTH]

        self.__thrustersOutputsSetpoints[self.__thrustersOrder.index(ThrustersNames.V_FRONT_LEFT)] = constrain(VFL, -100, 100)
        self.__thrustersOutputsSetpoints[self.__thrustersOrder.index(ThrustersNames.V_FRONT_RIGHT)] = constrain(VFR, -100, 100)
        self.__thrustersOutputsSetpoints[self.__thrustersOrder.index(ThrustersNames.V_REAR)] = constrain(VRR, -100, 100)

    def __thrustersCalibrate(self):
        if not (len(self.__thrustersOutputsSetpoints) == len(self.__thrustersDirCorr)):
            return None
        reThrusters = self.__thrustersOutputsSetpoints
        for i in range(len(self.__thrustersOutputsSetpoints)):
            reThrusters[i] = map_value(reThrusters[i], -100, 100, self.__trustersXValues[0], self.__trustersXValues[1]) * self.__thrustersDirCorr[i]
        self.__thrustersOutputsSetpoints = reThrusters
    
    def __smoothRPMBuildUp(self):
        for i in range(6):
            inc = self.__thrusterIncrement if self.__thrustersOutputsSetpoints[i] > 0 else 0-self.__thrusterIncrement
            self.__thrustersOutputs[i] += inc if abs(self.__thrustersOutputs[i]) < abs(self.__thrustersOutputsSetpoints[i]) else 0
            self.__thrustersOutputs[i] -= inc if abs(self.__thrustersOutputs[i]) > abs(self.__thrustersOutputsSetpoints[i]) else 0
            if (self.__thrustersOutputsSetpoints[i] > 0 and self.__thrustersOutputs[i] < 0) or (self.__thrustersOutputsSetpoints[i] < 0 and self.__thrustersOutputs[i] > 0):
                self.__thrustersOutputs[i] = 0
            if self.__thrustersOutputsSetpoints[i] == 0:
                self.__thrustersOutputs[i] = 0
            if abs(abs(self.__thrustersOutputs[i]) - abs(self.__thrustersOutputsSetpoints[i])) < self.__thrusterIncrement:
                self.__thrustersOutputs[i] = self.__thrustersOutputsSetpoints[i]
    
    # Setters
    def setPIDConstants(self, controlAxis: Axes, constants):
        if self.__PIDs[controlAxis] is not None:
            Kp, Ki, Kd = constants
            self.__PIDs[controlAxis].setConstants(Kp, Ki, Kd)

    def setPIDSetpoint(self, controlAxis: Axes, setpoint): 
        if self.__PIDs[controlAxis] is not None:       
            self.__PIDs[controlAxis].setSetpoint(setpoint)   

    def setAxesInputs(self, inputs):
        self.__axesInputs = inputs

    def setAxisInput(self, controlAxis: Axes, input):
        self.__axesInputs[controlAxis] = input

    def setAxesValues(self, inputs):
        self.__axesValues = inputs
        for i in range(6):
            if self.__filters[i] is not None: 
                self.__axesValues[i] = self.__filters[i].update(inputs[i])

    def setAxisValue(self, controlAxis: Axes, input):
        self.__axesValues[controlAxis] = input
        if self.__filters[controlAxis] is not None: 
            self.__axesValues[controlAxis] = self.__filters[controlAxis].update(input)

    def setStabilizations(self, inputs):
        self.__stabs = inputs

    def setStabilization(self, controlAxis: Axes, input):
        self.__stabs[controlAxis] = input

    def setdt(self, dt):
        self.__dt = dt

    def setThrustersCalibrationValues(self, thrustersDirCorr, thrustersOrder, trustersXValues, thrusterIncrement):
        self.__thrustersDirCorr = thrustersDirCorr
        self.__thrustersOrder = thrustersOrder
        self.__trustersXValues = trustersXValues
        self.__thrusterIncrement = thrusterIncrement  

    # Getters
    def getPIDSetpoint(self, controlAxis: Axes): 
        if self.__PIDs[controlAxis] is not None:       
            return self.__PIDs[controlAxis].setpoint
        return 0

    def getThrustersControls(self):
        self.__updateControl()
        return self.__thrustersOutputs

    def getThrusterControl(self, controlAxis: Axes):        
        self.__updateControl()
        return self.__thrustersOutputs[controlAxis]
    
    def getAxisValue(self, controlAxis: Axes):
        return self.__axesValues[controlAxis]
    
    def getAxesValues(self):
        return self.__axesValues
    