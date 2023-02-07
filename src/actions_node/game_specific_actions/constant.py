from enum import Enum

POSITION_DELTA_THRESHOLD = 0.05
INTAKE_ACTUATION_TIME = 0.25
ARM_EXTENSION_ACTUATION_TIME = 0.25

class ArmUpperJointForwardPositions(float, Enum):
    HighCube = 0.25
    MidCube = 0.25
    LowCube = 0.25
    HighCone = 0.25
    MidCone = 0.25
    LowCone = 0.25
    Hybrid = 0.25
    Ground = 0.25
    InRobot = 0

class ArmUpperJointReversePositions(float, Enum):
    HighCube = -0.25
    MidCube = -0.25
    LowCube = -0.25
    HighCone = -0.25
    MidCone = -0.25
    LowCone = -0.25
    Hybrid = -0.25
    Ground = -0.25
    InRobot = 0

class ArmLowerJointForwardPositions(float, Enum):
    HighCube = 0.3
    MidCube = 0.3
    LowCube = 0.3
    HighCone = 0.3
    MidCone = 0.3
    LowCone = 0.3
    Hybrid = 0.3
    Ground = 0.3
    InRobot = 0

class ArmLowerJointReversePositions(float, Enum):
    HighCube = -0.3
    MidCube = -0.3
    LowCube = -0.3
    HighCone = -0.3
    MidCone = -0.3
    LowCone = -0.3
    Hybrid = -0.3
    Ground = -0.3
    InRobot = 0