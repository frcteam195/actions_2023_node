from actions_node.default_actions.Action import Action
from actions_node.default_actions.SeriesAction import SeriesAction
from actions_node.default_actions.ParallelAction import ParallelAction
from actions_node.default_actions.WaitAction import WaitAction
from actions_node.game_specific_actions.MoveArmAction import MoveArmAction
from actions_node.game_specific_actions.MoveWristAction import MoveWristAction
from actions_node.game_specific_actions.MoveArmExtensionAction import MoveArmExtensionAction
from actions_node.game_specific_actions.IntakeAction import IntakeAction
from actions_node.game_specific_actions.constant import ArmPosition, ArmExtensionPosition, WristPosition
from actions_node.game_specific_actions import constant

def ComplexActionExample() -> Action:
    return SeriesAction([InRobotAction(),
                         WaitAction(0.5),
                         ParallelAction([HighCubeAction(True), MoveArmExtensionAction(ArmExtensionPosition.Extended)]), #This will put the arm out and go to high cube at the same time
                         ParallelAction([GroundAction(True), IntakeAction(False, 2)]), #This will put the arm the ground position and run the intake unpinched for 2 seconds
                         WaitAction(0.5),
                         GroundAction(False)
                         ])

def WristStraight() -> Action:
    return MoveWristAction(WristPosition.Zero)

def WristLeft90() -> Action:
    return MoveWristAction(WristPosition.Left_90)

def WristLeft180() -> Action:
    return MoveWristAction(WristPosition.Left_180)

def WristRight90() -> Action:
    return MoveWristAction(WristPosition.Right_90)

def WristRight180() -> Action:
    return MoveWristAction(WristPosition.Right_180)

def HighCubeAction(reversed : bool) -> Action:
    return SeriesAction([MoveArmAction(ArmPosition.HighCube, reversed),
                         MoveArmExtensionAction(ArmExtensionPosition.Extended)
                         ])

def MidCubeAction(reversed : bool) -> Action:
    return MoveArmAction(ArmPosition.MidCube, reversed)

def LowCubeAction(reversed : bool) -> Action:
    return MoveArmAction(ArmPosition.LowCube, reversed)

def HighConeAction(reversed : bool) -> Action:
    return MoveArmAction(ArmPosition.HighCone, reversed)

def MidConeAction(reversed : bool) -> Action:
    return MoveArmAction(ArmPosition.MidCone, reversed)

def LowConeAction(reversed : bool) -> Action:
    return MoveArmAction(ArmPosition.LowCone, reversed)

def HybridAction(reversed : bool) -> Action:
    return MoveArmAction(ArmPosition.Hybrid, reversed)

def InRobotAction() -> Action:
    return MoveArmAction(ArmPosition.InRobot, False)

def GroundAction(reversed: bool) -> Action:
    return MoveArmAction(ArmPosition.Ground, reversed)