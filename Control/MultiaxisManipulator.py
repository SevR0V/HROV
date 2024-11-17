from multipledispatch import dispatch
from enum import IntEnum

class GripState(IntEnum):
    UWMANIPULATOR_GRIP_OPEN = 0
    UWMANIPULATOR_GRIP_CLOSE = 1
    UWMANIPULATOR_GRIP_STOP = 2


class ManipAxis:
    def __init__(self):
        self.controlAngle = None
        self.telemetryAngle = None
        self.currentP1 = None
        self.currentP2 = None
        self.voltage = None
    
    def getTelemetryAngle(self):
        return self.telemetryAngle
    def getControlAngle(self):
        return self.controlAngle
    def getCurrent(self):
        return [self.currentP1, self.currentP2]
    def getVoltage(self):
        return self.voltage
       
    def setControlAngle(self, angle):
        self.controlAngle = angle
    
    def setTelemetryAngle(self, angle):
        self.telemetryAngle = angle

    @dispatch(float, float)
    def setCurrents(self, currentP1, currentP2):
        self.currentP1 = currentP1
        self.currentP2 = currentP2
    @dispatch(list)
    def setCurrents(self, currents):
        self.currentP1 = currents[0]
        self.currentP2 = currents[1]

    def setVoltage(self, voltage):
        self.voltage = voltage

class ManipGrip:
    def __init__(self):
        self.state = 0
    def open(self):
        self.state = GripState.UWMANIPULATOR_GRIP_OPEN
    def close(self):
        self.state = GripState.UWMANIPULATOR_GRIP_CLOSE
    def stop(self):
        self.state = GripState.UWMANIPULATOR_GRIP_STOP
    def getState(self):
        return self.state
    @dispatch(int)
    def setState(self, state):
        self.state = state
    @dispatch(float)
    def setState(self, state):
        self.state = state

class MultiaxisManipulator:
    def __init__(self, numAxes: int):
        self.axes = [ManipAxis]*numAxes
        self.grip = ManipGrip
        self.numAxes = numAxes
        self.updateControlFlags = [False]*numAxes
        self.updateTelemetryFlags = [[False, False]]*numAxes

    def setControlFlags(self, flags):
        if not len(flags) == self.numAxes:
            print("Incorrect Axes Number")
            return
        self.updateControlFlags = flags
    
    def setTelemetryFlags(self, flags):
        if not len(flags) == self.numAxes:
            print("Incorrect Axes Number")
            return
        self.updateTelemetryFlags = flags

    def setControlAngleAll(self, angles: list[float]):
        if not len(angles) == self.numAxes:
            print("Incorrect Axes Number")
            return
        for i, axis in enumerate(self.axes):
            axis.setControlAngle(angles[i])

    def setAxisControlAngle(self, axisIndex, angle):
        if not axisIndex in range(self.numAxes):
            print("Wrong axis index")
            return
        self.axes[axisIndex].setControlAngle(angle)

    def setTelemetryAngleAll(self, angles: list[float]):
        if not len(angles) == self.numAxes:
            print("Incorrect Axes Number")
            return
        for i, axis in enumerate(self.axes):
            axis.setTelemetryAngle(angles[i])

    def setAxisTelemetryAngle(self, axisIndex, angle):
        if not axisIndex in range(self.numAxes):
            print("Wrong axis index")
            return
        self.axes[axisIndex].setTelemetryAngle(angle)

    def setVoltageAll(self, voltages: list[float]):
        if not len(voltages) == self.numAxes:
            print("Incorrect Axes Number")
            return
        for i, axis in enumerate(self.axes):
            axis.setVoltage(voltages[i])

    def setAxisVoltage(self, axisIndex, voltage):
        if not axisIndex in range(self.numAxes):
            print("Wrong axis index")
            return
        self.axes[axisIndex].setVoltage(voltage)

    def setCurrentsAll(self, currents: list[list[float]]):
        if not len(currents) == self.numAxes:
            print("Incorrect Axes Number")
            return
        for i, axis in enumerate(self.axes):
            axis.setCurrents(currents[i])

    @dispatch(int, list)
    def setAxisCurrent(self, axisIndex, currents: list[float]):
        if not axisIndex in range(self.numAxes):
            print("Wrong axis index")
        self.axes[axisIndex].setCurrents(currents)

    @dispatch(int, float, float)
    def setAxisCurrent(self, axisIndex, currentP1, currentP2):
        if not axisIndex in range(self.numAxes):
            print("Wrong axis index")
        self.axes[axisIndex].setCurrents(currentP1, currentP2)

    def setGripState(self, state):
        self.grip.setState(state)

    def getControlFlags(self):
        return self.updateControlFlags
    
    def getTelemetryFlags(self):
        return self.updateTelemetryFlags

    def getTelemetryAngles(self):
        angles = [0.0]*self.numAxes
        for i, axis in enumerate(self.axes):
            angles[i] = axis.getTelemetryAngle()
        return [angles]

    def getAxisTelemetryAngle(self, axisIndex):
        if not axisIndex in range(self.numAxes):
            print("Wrong axis index")
        return self.axes[axisIndex].getTelemetryAngle()
    
    def getControlAngles(self):
        angles = [0.0]*self.numAxes
        for i, axis in enumerate(self.axes):
            angles[i] = axis.getControlAngle()
        return [angles]

    def getAxisControlAngle(self, axisIndex):
        if not axisIndex in range(self.numAxes):
            print("Wrong axis index")
        return self.axes[axisIndex].getControlAngle()

    def getVoltages(self):
        voltages = [0.0]*self.numAxes
        for i, axis in enumerate(self.axes):
            voltages[i] = axis.getVoltage()
        return [voltages]

    def getAxisVoltage(self, axisIndex):
        if not axisIndex in range(self.numAxes):
            print("Wrong axis index")
        return self.axes[axisIndex].getVoltage()

    def getCurrents(self):
        currents = [[0.0, 0.0]]*self.numAxes
        for i, axis in enumerate(self.axes):
            currents[i] = axis.getCurrent()
        return [currents]

    def getAxisCurrent(self, axisIndex):
        if not axisIndex in range(self.numAxes):
            print("Wrong axis index")
        return self.axes[axisIndex].getCurrent()
    
    def getGripState(self):
        state = self.getGripState() + 1
        return state

    def getAxisNumber(self):
        return self.numAxes