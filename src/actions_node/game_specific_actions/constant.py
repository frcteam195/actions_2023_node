from enum import Enum

ARM_POSITION_DELTA_THRESHOLD = 0.05
ARM_EXTENSION_ACTUATION_TIME = 0.25

class ArmExtensionPosition(int, Enum):
    """
    Extension position enumeration for the arm.
    Do not change these values.
    """
    Unchanged = -20
    Retracted = 0
    Extended = 1

class RollerState(int, Enum):
    """
    State enumeration for the intake rollers.
    """
    Off = 0
    Intake_Cone = 1
    Intake_Cube = 2
    Outtake_Cone = 3
    Outtake_Cube = 4
