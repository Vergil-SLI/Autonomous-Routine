import commands2
import ctre
from wpimath.controller import PIDController
import math


class Drivetrain(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()
        self.gyro = ctre.PigeonIMU(10)
        self.gyro.setYaw(0,0)
        
        self.m_left = ctre.TalonSRX(0)
        self.m_left_2 = ctre.VictorSPX(1)
        self.m_left_3 = ctre.VictorSPX(2)
        self.m_right = ctre.TalonSRX(3)
        self.m_right_2 = ctre.VictorSPX(4)
        self.m_right_3 = ctre.VictorSPX(5)
        
        self.m_left_2.follow(self.m_left)
        self.m_left_3.follow(self.m_left)
        
        self.m_right_2.follow(self.m_right)
        self.m_right_3.follow(self.m_right)
        
        self.m_right_encoder=ctre.CANCoder(9)
        self.m_left_encoder=ctre.CANCoder(11)
        
        # self.turn_controller = PIDController(0.001, 0, 0)
        # self.turn_controller.enableContinuousInput(0, 360)
        # self.turn_controller.setTolerance(2)
        
        self.linear_controller = PIDController(0.0028, 0.0001, 0)
        self.linear_controller.setTolerance(15)
        
        
        self.m_left_encoder.setPosition(0)
        self.linear_controller.setSetpoint(0)
        

    def set(self, left: float, right: float):
        self.m_left.set(ctre.ControlMode.PercentOutput, left)
        self.m_right.set(ctre.ControlMode.PercentOutput, -right)
    
    def auto_routine(self, stage):
        # got straight for 7 ft
        if (stage == 1):
            self.linear_controller.setSetpoint(84 * 360 / (4 * math.pi)) # 84
            linear_motion = self.linear_controller.calculate(measurement = self.m_left_encoder.getPosition())
            self.set(linear_motion, linear_motion)
            
            if self.linear_controller.atSetpoint():
                print("done")
                stage += 1
            
            print("setpoint: ", 84 * 360 / (4 * math.pi), "current spot: ", self.m_left_encoder.getPosition())
        
        # # turn 90 degrees
        # if(stage == 2):
        #     self.turn_controller.setSetpoint(90)
        #     turn_motion = self.turn_controller.calculate(measurement = self.gyro.getYaw())
        #     self.set(-turn_motion, turn_motion)
            
        #     if (self.turn_controller.atSetpoint()):
        #         stage += 1
        
        # # got straight for 2 ft
        # if(stage == 3):
        #     self.linear_controller.setSetpoint(self.linear_controller.getSetpoint() + 24 * 360 / (4 * math.pi))
        #     linear_motion = self.linear_controller.calculate(measurement = self.m_left_encoder.getPosition())
        #     self.set(linear_motion, linear_motion)
            
        #     if self.linear_controller.atSetpoint():
        #         stage += 1
        
        # if (stage == 4):
        #     self.m_left_encoder.setPosition(0)
        #     self.linear_controller.setSetpoint(0)
        #     linear_motion = self.linear_controller.calculate(measurement = self.m_left_encoder.getPosition())
        #     self.set(linear_motion, linear_motion)
            
        
        return stage
    

    def periodic(self):
        pass
