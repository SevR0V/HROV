from enum import IntEnum
import numpy as np
from utils import constrain, PID, ExpMovingAverageFilter, map_value

class Axes(IntEnum):
    FORWARD     = 0
    STRAFE      = 1
    DEPTH       = 2
    ROLL        = 3
    PITCH       = 4    
    YAW         = 5

class ThrustersNames(IntEnum):
    # HROV specific
    H_FORW_TOP      = 0
    H_FORW_BOT      = 1
    H_SIDE_FRONT    = 2
    H_SIDE_REAR     = 3
    V_RIGHT         = 4
    V_LEFT          = 5

class ControlSystem:
    def __init__(self):
        # Thrusters calibration values
        self.__thrustersDirCorr = [1, 1, 1, 1, 1, 1]
        self.__thrustersOrder = [ThrustersNames.H_FORW_TOP, 
                  ThrustersNames.H_FORW_BOT,
                  ThrustersNames.H_SIDE_FRONT, 
                  ThrustersNames.H_SIDE_REAR,
                  ThrustersNames.V_RIGHT,
                  ThrustersNames.V_LEFT]
        self.__trustersXValues = [-100, 100]
        self.__thrusterIncrement = 0.5
        # (forward,strafe,depth,roll,pitch,yaw)
        self.__axesInputs     = [0, 0, 0, 0, 0, 0]
        self.__axesValues     = [0, 0, 0, 0, 0, 0]        
        self.__stabs          = [0, 0, 0, 0, 0, 0]
        self.__PIDValues      = [0, 0, 0, 0, 0, 0]
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
        self.__motsOutputsSetpoint = [0, 0, 0, 0, 0, 0]
        self.__motsOutputsReal = [0, 0, 0, 0, 0, 0]
        # Operating period
        self.__dt = 1/500



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

    def __calculateThrust(self):
        self.__motsOutputsSetpoint[self.__thrustersOrder.index(ThrustersNames.H_FORW_TOP)] = constrain(self.__axesInputs[Axes.FORWARD] - self.__PIDValues[Axes.PITCH], -100, 100)
        self.__motsOutputsSetpoint[self.__thrustersOrder.index(ThrustersNames.H_FORW_BOT)] = constrain(self.__axesInputs[Axes.FORWARD] + self.__PIDValues[Axes.PITCH], -100, 100)
        self.__motsOutputsSetpoint[self.__thrustersOrder.index(ThrustersNames.H_SIDE_FRONT)] = constrain(self.__axesInputs[Axes.STRAFE] + self.__axesInputs[Axes.YAW] + self.__PIDValues[Axes.YAW], -100, 100)
        self.__motsOutputsSetpoint[self.__thrustersOrder.index(ThrustersNames.H_SIDE_REAR)] = constrain(self.__axesInputs[Axes.STRAFE] - self.__axesInputs[Axes.YAW] - self.__PIDValues[Axes.YAW], -100, 100)
        self.__motsOutputsSetpoint[self.__thrustersOrder.index(ThrustersNames.V_RIGHT)] = constrain(self.__axesInputs[Axes.DEPTH] - self.__PIDValues[Axes.ROLL], -100, 100)
        self.__motsOutputsSetpoint[self.__thrustersOrder.index(ThrustersNames.V_LEFT)] = constrain(self.__axesInputs[Axes.DEPTH] + self.__PIDValues[Axes.ROLL], -100, 100)

    
    def __thrustersCalibrate(self):
        if not (len(self.__motsOutputsSetpoint) == len(self.__thrustersDirCorr)) and not (len(self.__motsOutputsSetpoint) == len(self.__thrustersOrder)):
            return None
        reThrusters = self.__motsOutputsSetpoint
        for i in range(len(self.__motsOutputsSetpoint)):
            reThrusters[i] = map_value(reThrusters[i], -100, 100, self.__trustersXValues[0], self.__trustersXValues[1]) * self.__thrustersDirCorr[i]
        self.__motsOutputsSetpoint = reThrusters

    def __updateControl(self):
        self.__updatePID()
        self.__calculateThrust()
        self.__thrustersCalibrate()

        for i in range(6):
            inc = self.__thrusterIncrement if self.__motsOutputsSetpoint[i] > 0 else 0-self.__thrusterIncrement
            self.__motsOutputsReal[i] += inc if abs(self.__motsOutputsReal[i]) < abs(self.__motsOutputsSetpoint[i]) else 0
            self.__motsOutputsReal[i] -= inc if abs(self.__motsOutputsReal[i]) > abs(self.__motsOutputsSetpoint[i]) else 0
            if (self.__motsOutputsSetpoint[i] > 0 and self.__motsOutputsReal[i] < 0) or (self.__motsOutputsSetpoint[i] < 0 and self.__motsOutputsReal[i] > 0):
                self.__motsOutputsReal[i] = 0
            if self.__motsOutputsSetpoint[i] == 0:
                self.__motsOutputsReal[i] = 0
            if abs(abs(self.__motsOutputsReal[i]) - abs(self.__motsOutputsSetpoint[i])) < self.__thrusterIncrement:
                self.__motsOutputsReal[i] = self.__motsOutputsSetpoint[i]
    
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
        return self.__motsOutputsReal

    def getThrusterControl(self, controlAxis: Axes):
        self.__updateControl()
        return self.__motsOutputsReal[controlAxis]
    
    def getAxisValue(self, controlAxis: Axes):
        return self.__axesValues[controlAxis]
    
    def getAxesValues(self):
        return self.__axesValues