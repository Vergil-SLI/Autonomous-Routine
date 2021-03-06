from wpilib import run, TimedRobot, Joystick
import math
from wpimath.controller import PIDController
from drivetrain import Drivetrain


class Robot(TimedRobot):
    
    joy1=Joystick(0)
    
    def __init__(self):
        super().__init__()
        self.drivetrain = Drivetrain()
        self.stage = 1
            

    def robotInit(self):
        pass

    def robotPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        speed=-self.joy1.getRawAxis(1)*.4
        turn=self.joy1.getRawAxis(4)*.2
        #print(f"Speed: {speed} Turn: {turn}")
        self.drivetrain.set(speed+turn, speed-turn)

    def autonomousInit(self):
        pass
    
    def autonomousPeriodic(self):
        self.stage = self.drivetrain.auto_routine(self.stage)
        # print(self.stage)
        

if __name__ == "__main__":
    run(Robot)
