from enum import IntEnum
import numpy as np
from utils import constrain, PID, ExpMovingAverageFilter, map_value


class ControlSystem:
    def __init__(self):
        # Thrusters calibration values
        self.__thrustersDirCorr = [1, 1, 1, 1, 1, 1]
        self.__thrustersOrder = [ControlSystem.Thrusters.H_FORW_TOP, 
                  ControlSystem.Thrusters.H_FORW_BOT,
                  ControlSystem.Thrusters.H_SIDE_FRONT, 
                  ControlSystem.Thrusters.H_SIDE_REAR,
                  ControlSystem.Thrusters.V_RIGHT,
                  ControlSystem.Thrusters.V_LEFT]
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

    class ControlAxes(IntEnum):
        FORWARD     = 0
        STRAFE      = 1
        DEPTH       = 2
        ROLL        = 3
        PITCH       = 4    
        YAW         = 5

    class Thrusters(IntEnum):
        # HROV specific
        H_FORW_TOP      = 0
        H_FORW_BOT      = 1
        H_SIDE_FRONT    = 2
        H_SIDE_REAR     = 3
        V_RIGHT         = 4
        V_LEFT          = 5

    def __updatePID(self):
        if not ((np.abs(self.__axesInputs[self.ControlAxes.DEPTH])<1) and self.__stabs[self.ControlAxes.DEPTH]):
            self.setPIDSetpoint(self.ControlAxes.DEPTH, self.__axesValues[self.ControlAxes.DEPTH])
        
        if np.abs(self.__axesInputs[self.ControlAxes.YAW]) >= 1 or not self.__stabs[self.ControlAxes.YAW] : 
            self.setPIDSetpoint(self.ControlAxes.YAW, self.__axesValues[self.ControlAxes.YAW])

        yawPID = self.__PIDs[self.ControlAxes.YAW].update(self.__axesValues[self.ControlAxes.YAW], self.__dt) if (np.abs(self.__axesInputs[self.ControlAxes.YAW]) < 1) and self.__stabs[self.ControlAxes.YAW] else 0 
        rollPID = self.__PIDs[self.ControlAxes.ROLL].update(self.__axesValues[self.ControlAxes.ROLL], self.__dt) if self.__stabs[self.ControlAxes.ROLL] else 0
        pitchPID = self.__PIDs[self.ControlAxes.PITCH].update(self.__axesValues[self.ControlAxes.PITCH], self.__dt) if self.__stabs[self.ControlAxes.PITCH] else 0
        depthPID = -self.__PIDs[self.ControlAxes.DEPTH].update(self.__axesValues[self.ControlAxes.DEPTH], self.__dt) if self.__stabs[self.ControlAxes.DEPTH] else 0

        self.__PIDValues[self.ControlAxes.YAW] = constrain(yawPID, -100, 100)
        self.__PIDValues[self.ControlAxes.ROLL] = constrain(rollPID, -100, 100)
        self.__PIDValues[self.ControlAxes.PITCH] = constrain(pitchPID, -100, 100)
        self.__PIDValues[self.ControlAxes.DEPTH] = constrain(depthPID, -100, 100)

    def __calculateThrust(self):
        self.__motsOutputsSetpoint[self.Thrusters.H_FORW_TOP] = constrain(self.__axesInputs[self.ControlAxes.FORWARD] - self.__PIDValues[self.ControlAxes.PITCH], -100, 100)
        self.__motsOutputsSetpoint[self.Thrusters.H_FORW_BOT] = constrain(self.__axesInputs[self.ControlAxes.FORWARD] + self.__PIDValues[self.ControlAxes.PITCH], -100, 100)
        self.__motsOutputsSetpoint[self.Thrusters.H_SIDE_FRONT] = constrain(self.__axesInputs[self.ControlAxes.STRAFE] + self.__axesInputs[self.ControlAxes.YAW] + self.__PIDValues[self.ControlAxes.YAW], -100, 100)
        self.__motsOutputsSetpoint[self.Thrusters.H_SIDE_REAR] = constrain(self.__axesInputs[self.ControlAxes.STRAFE] - self.__axesInputs[self.ControlAxes.YAW] - self.__PIDValues[self.ControlAxes.YAW], -100, 100)
        self.__motsOutputsSetpoint[self.Thrusters.V_RIGHT] = constrain(self.__axesInputs[self.ControlAxes.DEPTH] - self.__PIDValues[self.ControlAxes.ROLL], -100, 100)
        self.__motsOutputsSetpoint[self.Thrusters.V_LEFT] = constrain(self.__axesInputs[self.ControlAxes.DEPTH] + self.__PIDValues[self.ControlAxes.ROLL], -100, 100)

    
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
    def setPIDConstants(self, controlAxis: ControlAxes, constants):
        if self.__PIDs[controlAxis] is not None:
            Kp, Ki, Kd = constants
            self.__PIDs[controlAxis].setConstants(Kp, Ki, Kd)

    def setPIDSetpoint(self, controlAxis: ControlAxes, setpoint): 
        if self.__PIDs[controlAxis] is not None:       
            self.__PIDs[controlAxis].setSetpoint(setpoint)   

    def setAxesInputs(self, inputs):
        self.__axesInputs = inputs

    def setAxisInput(self, controlAxis: ControlAxes, input):
        self.__axesInputs[controlAxis] = input

    def setAxesValues(self, inputs):
        self.__axesValues = inputs
        for i in range(6):
            if self.__filters[i] is not None: 
                self.__axesValues[i] = self.__filters[i].update(inputs[i])

    def setAxisValue(self, controlAxis: ControlAxes, input):
        self.__axesValues[controlAxis] = input
        if self.__filters[controlAxis] is not None: 
            self.__axesValues[controlAxis] = self.__filters[controlAxis].update(input)

    def setStabilizations(self, inputs):
        self.__stabs = inputs

    def setStabilization(self, controlAxis: ControlAxes, input):
        self.__stabs[controlAxis] = input

    def setdt(self, dt):
        self.__dt = dt

    def setThrustersCalibrationValues(self, thrustersDirCorr, thrustersOrder, trustersXValues, thrusterIncrement):
        self.__thrustersDirCorr = thrustersDirCorr
        self.__thrustersOrder = thrustersOrder
        self.__trustersXValues = trustersXValues
        self.__thrusterIncrement = thrusterIncrement       

    # Getters
    def getPIDSetpoint(self, controlAxis: ControlAxes): 
        if self.__PIDs[controlAxis] is not None:       
            return self.__PIDs[controlAxis].setpoint
        return 0

    def getThrustersControls(self):
        self.__updateControl()
        return self.__motsOutputsReal

    def getThrusterControl(self, controlAxis: ControlAxes):
        self.__updateControl()
        return self.__motsOutputsReal[controlAxis]
    
    def getAxisValue(self, controlAxis: ControlAxes):
        return self.__axesValues[controlAxis]
    
    def getAxesValues(self):
        return self.__axesValues