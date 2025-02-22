from wpilib import MotorControllerGroup

class DifferentialDrive:
    def __init__(self, leftGroup: MotorControllerGroup, rightGroup: MotorControllerGroup):
        self._leftGroup = leftGroup
        self._rightGroup = rightGroup
        self._leftSpeed = 0
        self._rightSpeed = 0

    def setLeft(self, speed: float):
        self._leftSpeed = speed
        self._update()

    def setRight(self, speed: float):
        self._rightSpeed = speed
        self._update()

    def setSpeed(self, speed: float):
        self._rightSpeed = speed
        self._leftSpeed = speed
        self._update()

    def _update(self):
        self._leftGroup.set(self._leftSpeed)
        self._rightGroup.set(self._rightSpeed)