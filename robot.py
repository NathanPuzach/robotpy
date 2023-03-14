# ----------------------------------------------------------------------------
# Copyright (c) 2017-2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
# ----------------------------------------------------------------------------

import rev
import wpilib
from wpilib.drive import DifferentialDrive
import navx
import pathplannerlib
import wpimath
import wpimath.controller
import robotpy_apriltag
import math
import photonvision
from networktables import NetworkTables
import commands2

#import apriltag
import logging

logging.basicConfig(level=logging.DEBUG)

def run():
    raise ValueError()

class Robot(wpilib.TimedRobot):
    sd = NetworkTables.getTable("SmartDashboard")
    NetworkTables.initialize()
    
    

    def robotInit(self):
        self.constraints = pathplannerlib.PathConstraints(1,1)
        self.reverse = False
        self.makingpath = pathplannerlib.PathPlanner.loadPath("Straight Back Blue", self.constraints, self.reverse)
        self.ramsetecontroller = wpimath.controller.RamseteController()
        self.translation = wpimath.geometry.Translation2d(1.96, 2.18)
        self.rotation = wpimath.geometry.Rotation2d(math.pi)
        self.pose = wpimath.geometry.Pose2d(self.translation,self.rotation)
        self.kinematics = wpimath.kinematics.DifferentialDriveKinematics(0.0254)
        self.leftPID = wpimath.controller.PIDController(0,0,0,0.2)
        self.rightPID = wpimath.controller.PIDController(0,0,0,0.2)
        self.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters()
        self.wheelspeeds = 
        self.output =
        self.command = commands2.RamseteCommand(self.makingpath, self.pose, self.ramsetecontroller, self.kinematics, self.leftPID, self.rightPID, )
        
        self.translation_a = wpimath.geometry.Translation3d(0.25,0.25,0.25)
        self.rotation_a = wpimath.geometry.Rotation3d(0,0,0)
        self.robottocam = wpimath.geometry.Transform3d(self.translation_a, self.rotation_a)
        self.upload = robotpy_apriltag.AprilTagField(1)
        self.currentfield = robotpy_apriltag.loadAprilTagLayoutField(self.upload)
        self.strategy = photonvision.PoseStrategy(2)
        self.limelight = photonvision.PhotonCamera("Limelight1")
        self.campair1 = (self.limelight, self.robottocam)
        self.cameras = [self.campair1]
        self.poseestimator = photonvision.RobotPoseEstimator(self.currentfield,self.strategy, self.cameras)

       
        self.forwards = 0
        self.backwards = 1
        self.pneumatichub = wpilib.PneumaticHub(1)
        
        self.sol = wpilib.DoubleSolenoid( wpilib.PneumaticsModuleType.REVPH ,self.forwards, self.backwards)

        self.leftfrontMotor = rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless)
        self.leftbackMotor = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)
        self.rightfrontMotor = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)
        self.rightbackMotor = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
        self.rot1Motor = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)
        self.rot2Motor = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
        self.armMotor = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
        # The RestoreFactoryDefaults method can be used to reset the
        # configuration parameters in the SPARK MAX to their factory default
        # state. If no argument is passed, these parameters will not persist
        # between power cycles
        self.leftfrontMotor.restoreFactoryDefaults()
        self.leftbackMotor.restoreFactoryDefaults()
        self.rightfrontMotor.restoreFactoryDefaults()
        self.rightbackMotor.restoreFactoryDefaults()
        self.rot1Motor.restoreFactoryDefaults()
        self.rot2Motor.restoreFactoryDefaults()
        self.armMotor.restoreFactoryDefaults()
        self.rot2Motor.setInverted(True)
        self.rot1Motor.setInverted(True)

        self.arm = wpilib.MotorControllerGroup(self.rot1Motor,self.rot2Motor)
        self.left = wpilib.MotorControllerGroup(self.leftfrontMotor, self.leftbackMotor)
        self.right = wpilib.MotorControllerGroup(self.rightfrontMotor, self.rightbackMotor)

        self.driveTrain = DifferentialDrive(self.left, self.right)
        self.driveTrain.setExpiration(0.1)
        
        self.reverse = wpilib.DoubleSolenoid.Value.kReverse
        self.forward = wpilib.DoubleSolenoid.Value.kForward
        self.off =  wpilib.DoubleSolenoid.Value.kOff

        self.navx = navx.AHRS(wpilib.SerialPort.Port.kUSB)
        
        self.l_stick = wpilib.Joystick(0)

        self.timer = wpilib.Timer()
        

    def teleopPeriodic(self):
        if self.l_stick.getRawButton(3):
            self.driveTrain.tankDrive((self.l_stick.getRawAxis(1) * -1.00), (self.l_stick.getRawAxis(5) * 1.00))
        else:
            self.driveTrain.tankDrive((self.l_stick.getRawAxis(1) * -0.70), (self.l_stick.getRawAxis(5) * 0.70))

        if self.l_stick.getRawButton(2):
            self.sol.set(self.reverse)
        elif self.l_stick.getRawButton(1):
            self.sol.set(self.forward)
        else:
            self.sol.set(self.off)
        
        if self.l_stick.getRawAxis(2) > 0:
            self.arm.set(0.3)
        elif self.l_stick.getRawAxis(3) > 0:    
            self.arm.set(-0.3)
        else:
            self.arm.set(0)
        
        if self.l_stick.getRawButton(6): 
            self.armMotor.set(0.35)
        elif self.l_stick.getRawButton(5):
            self.armMotor.set(-0.35) 
        else:
            self.armMotor.set(0)
 
        


            
        
        
        
    


if __name__ == "__main__":
    wpilib.run(Robot)