from wpilib import TimedRobot
from wpilib import Joystick
from wpilib import MotorControllerGroup
from wpilib import PWMVictorSPX
from diffDrive import DifferentialDrive
import wpilib
import time
from rev import SparkMax, SparkLowLevel

class MyRobot(TimedRobot):
    def robotInit(self):
        """Robot initialization code goes here"""
        # Motor controllers (using CAN IDs for Spark MAX controllers)
        self.front_left_motor = SparkMax(3, SparkLowLevel.MotorType.kBrushless)
        self.rear_left_motor = SparkMax(5, SparkLowLevel.MotorType.kBrushless)
        self.front_right_motor = SparkMax(2, SparkLowLevel.MotorType.kBrushless)
        self.rear_right_motor = SparkMax(4, SparkLowLevel.MotorType.kBrushless)
 
        # Create motor groups for left and right
        self.left_motor_group = MotorControllerGroup(self.front_left_motor, self.rear_left_motor)
        self.right_motor_group = MotorControllerGroup(self.front_right_motor, self.rear_right_motor)

        # Create a differential drive object
        self.drive = DifferentialDrive(self.left_motor_group, self.right_motor_group)

        # Joystick (used for driving)
        self.joystick = Joystick(0)  # Joystick 0 for tank drive

        # Timer for autonomous mode
        self.auto_timer = time.time()

    def autonomousInit(self):
        """Called once when autonomous mode starts"""
        self.auto_timer = time.time()  # Reset the timer when autonomous starts

    def autonomousPeriodic(self):
        """Called periodically during autonomous"""
        elapsed_time = time.time() - self.auto_timer  # Track elapsed time

        if elapsed_time < 2.0:  # Move forward for 2 seconds
            self.drive.setSpeed(0.5)  # Move both motors forward at half speed
        else:
            self.drive.setSpeed(0)  # Stop the robot after 2 seconds

    def teleopPeriodic(self):
        """Called periodically during operator control (teleop mode)."""
        left_speed = self.joystick.getY()  # Left joystick (Y-axis)
        right_speed = self.joystick.getRawAxis(5)  # Right joystick (X-axis)

        self.drive.setLeft(self.joystick.getY())
        self.drive.setRight(self.joystick.getRawAxis(5))

if __name__ == "__main__":
    wpilib.run(MyRobot)
