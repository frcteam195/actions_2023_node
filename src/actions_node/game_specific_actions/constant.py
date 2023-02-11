from enum import Enum

ARM_POSITION_DELTA_THRESHOLD = 0.05
WRIST_POSITION_DELTA_THRESHOLD = 0.05
INTAKE_ACTUATION_TIME = 0.25
ARM_EXTENSION_ACTUATION_TIME = 0.25

class ArmPosition(int, Enum):
    #Do not change these values
    Unchanged = -20,
    HighCube = 0
    MidCube = 1
    LowCube = 2
    HighCone = 3
    MidCone = 4
    LowCone = 5
    Hybrid = 6
    Ground = 7
    InRobot = 8

class ArmExtensionPosition(int, Enum):
    #Do not change these values
    Unchanged = -20,
    Retracted = 0,
    Extended = 1, 

class WristPosition(float, Enum):
    Unchanged = -20.0,
    ##########################
    #Change these values
    Zero = 0.0,
    Left_90 = 0.25,
    Left_180 = 0.5,
    Right_90 = -0.25,
    Right_180 = -0.5
    ##########################



ArmUpperJointForwardPositions = {
    ArmPosition.Unchanged:-20.0,
    ##########################
    #Change these values
    ArmPosition.HighCube:0.25,
    ArmPosition.MidCube:0.25,
    ArmPosition.LowCube:0.25,
    ArmPosition.HighCone:0.25,
    ArmPosition.MidCone:0.25,
    ArmPosition.LowCone:0.25,
    ArmPosition.Hybrid:0.25,
    ArmPosition.Ground:0.25,
    ArmPosition.InRobot:0.25
    ##########################
}

ArmUpperJointReversePositions = {
    ArmPosition.Unchanged:-20.0,
    ##########################
    #Change these values
    ArmPosition.HighCube:-0.25,
    ArmPosition.MidCube:-0.25,
    ArmPosition.LowCube:-0.25,
    ArmPosition.HighCone:-0.25,
    ArmPosition.MidCone:-0.25,
    ArmPosition.LowCone:-0.25,
    ArmPosition.Hybrid:-0.25,
    ArmPosition.Ground:-0.25,
    ArmPosition.InRobot:-0.25
    ##########################
}

ArmLowerJointForwardPositions = {
    ArmPosition.Unchanged:-20.0,
    ##########################
    #Change these values
    ArmPosition.HighCube:0.1,
    ArmPosition.MidCube:0.1,
    ArmPosition.LowCube:0.1,
    ArmPosition.HighCone:0.1,
    ArmPosition.MidCone:0.1,
    ArmPosition.LowCone:0.1,
    ArmPosition.Hybrid:0.1,
    ArmPosition.Ground:0.1,
    ArmPosition.InRobot:0.1
    ##########################
}

ArmLowerJointReversePositions = {
    ArmPosition.Unchanged:-20.0,
    ##########################
    #Change these values
    ArmPosition.HighCube:-0.1,
    ArmPosition.MidCube:-0.1,
    ArmPosition.LowCube:-0.1,
    ArmPosition.HighCone:-0.1,
    ArmPosition.MidCone:-0.1,
    ArmPosition.LowCone:-0.1,
    ArmPosition.Hybrid:-0.1,
    ArmPosition.Ground:-0.1,
    ArmPosition.InRobot:-0.1
    ##########################
}