from enum import IntEnum
import numpy as np
from utils import constrain, PID, ExpMovingAverageFilter, map_value


class HROVControlSystem:
    def __init__(self, thrustersDirCorr, thrustersOrder, trustersXValues):
        self.thrustersDirCorr = thrustersDirCorr
        self.thrustersOrder = thrustersOrder
        self.trustersXValues = trustersXValues
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
        self.__motsOutputs = [0, 0, 0, 0, 0, 0]
        self.__dt = 1/500

    class ControlAxes(IntEnum):
        FORWARD     = 0
        STRAFE      = 1
        DEPTH       = 2
        ROLL        = 3
        PITCH       = 4    
        YAW         = 5

    class Thrusters(IntEnum):
        # Y-frame specific
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
        self.__motsOutputs[self.Thrusters.H_FORW_TOP] = constrain(self.__axesInputs[self.ControlAxes.FORWARD] - self.__PIDValues[self.ControlAxes.PITCH], -100, 100)
        self.__motsOutputs[self.Thrusters.H_FORW_BOT] = constrain(self.__axesInputs[self.ControlAxes.FORWARD] + self.__PIDValues[self.ControlAxes.PITCH], -100, 100)
        self.__motsOutputs[self.Thrusters.H_SIDE_FRONT] = constrain(self.__axesInputs[self.ControlAxes.STRAFE] + self.__axesInputs[self.ControlAxes.YAW] + self.__PIDValues[self.ControlAxes.YAW], -100, 100)
        self.__motsOutputs[self.Thrusters.H_SIDE_REAR] = constrain(self.__axesInputs[self.ControlAxes.STRAFE] - self.__axesInputs[self.ControlAxes.YAW] - self.__PIDValues[self.ControlAxes.YAW], -100, 100)
        self.__motsOutputs[self.Thrusters.V_RIGHT] = constrain(self.__axesInputs[self.ControlAxes.DEPTH] - self.__PIDValues[self.ControlAxes.ROLL], -100, 100)
        self.__motsOutputs[self.Thrusters.V_LEFT] = constrain(self.__axesInputs[self.ControlAxes.DEPTH] + self.__PIDValues[self.ControlAxes.ROLL], -100, 100)

    
    def __thrustersCalibrate(self):
        if not (len(self.__motsOutputs) == len(self.thrustersDirCorr)) and not (len(self.__motsOutputs) == len(self.thrustersOrder)):
            return None
        reThrusters = [0.0] * len(self.__motsOutputs)
        for i in range(len(self.__motsOutputs)):
            reThrusters[i] = self.__motsOutputs[self.thrustersOrder[i]]
            reThrusters[i] = map_value(reThrusters[i], -100, 100, self.trustersXValues[0], self.trustersXValues[1]) * self.thrustersDirCorr[i]
        self.__motsOutputs = reThrusters

    def __updateControl(self):
        self.__updatePID()
        self.__calculateThrust()
        self.__thrustersCalibrate()
    
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

    # Getters
    def getPIDSetpoint(self, controlAxis: ControlAxes): 
        if self.__PIDs[controlAxis] is not None:       
            return self.__PIDs[controlAxis].setpoint
        return 0

    def getMotsControls(self):
        self.__updateControl()
        return self.__motsOutputs

    def getMotControl(self, controlAxis: ControlAxes):
        self.__updateControl()
        return self.__motsOutputs[controlAxis]
    
    def getAxisValue(self, controlAxis: ControlAxes):
        return self.__axesValues[controlAxis]
    
    def getAxesValues(self):
        return self.__axesValues