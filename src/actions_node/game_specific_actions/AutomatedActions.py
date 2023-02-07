from actions_node.default_actions.Action import Action
from actions_node.default_actions.SeriesAction import SeriesAction
from actions_node.default_actions.ParallelAction import ParallelAction
from actions_node.default_actions.WaitAction import WaitAction
from actions_node.game_specific_actions.MoveArmAction import MoveArmAction
from actions_node.game_specific_actions.constant import ArmLowerJointForwardPositions, ArmLowerJointReversePositions, ArmUpperJointForwardPositions, ArmUpperJointReversePositions
from actions_node.game_specific_actions import constant

def ComplexActionExample() -> Action:
    return SeriesAction([InRobotAction(),
                         WaitAction(0.5),
                         ParallelAction([HighCubeAction(True), ExtendArmAction()]), #This will put the arm out and go to high cube at the same time
                         WaitAction(0.5),
                         GroundAction(False)
                         ])

def ExtendArmAction() -> Action:
    #TODO: Implement
    pass
    
def HighCubeAction(reversed : bool) -> Action:
    print("HighCubeAction")
    return MoveArmAction(ArmUpperJointReversePositions.HighCube if reversed else ArmUpperJointForwardPositions.HighCube,
                         ArmLowerJointReversePositions.HighCube if reversed else ArmLowerJointForwardPositions.HighCube,
                         constant.POSITION_DELTA_THRESHOLD)

def MidCubeAction(reversed : bool) -> Action:
    return MoveArmAction(ArmUpperJointReversePositions.MidCube if reversed else ArmUpperJointForwardPositions.MidCube,
                         ArmLowerJointReversePositions.MidCube if reversed else ArmLowerJointForwardPositions.MidCube,
                         constant.POSITION_DELTA_THRESHOLD)

def LowCubeAction(reversed : bool) -> Action:
    return MoveArmAction(ArmUpperJointReversePositions.LowCube if reversed else ArmUpperJointForwardPositions.LowCube,
                         ArmLowerJointReversePositions.LowCube if reversed else ArmLowerJointForwardPositions.LowCube,
                         constant.POSITION_DELTA_THRESHOLD)

def HighConeAction(reversed : bool) -> Action:
    return MoveArmAction(ArmUpperJointReversePositions.HighCone if reversed else ArmUpperJointForwardPositions.HighCone,
                         ArmLowerJointReversePositions.HighCone if reversed else ArmLowerJointForwardPositions.HighCone,
                         constant.POSITION_DELTA_THRESHOLD)

def MidConeAction(reversed : bool) -> Action:
    return MoveArmAction(ArmUpperJointReversePositions.MidCone if reversed else ArmUpperJointForwardPositions.MidCone,
                         ArmLowerJointReversePositions.MidCone if reversed else ArmLowerJointForwardPositions.MidCone,
                         constant.POSITION_DELTA_THRESHOLD)

def LowConeAction(reversed : bool) -> Action:
    return MoveArmAction(ArmUpperJointReversePositions.LowCone if reversed else ArmUpperJointForwardPositions.LowCone,
                         ArmLowerJointReversePositions.LowCone if reversed else ArmLowerJointForwardPositions.LowCone,
                         constant.POSITION_DELTA_THRESHOLD)

def HybridAction(reversed : bool) -> Action:
    return MoveArmAction(ArmUpperJointReversePositions.Hybrid if reversed else ArmUpperJointForwardPositions.Hybrid,
                         ArmLowerJointReversePositions.Hybrid if reversed else ArmLowerJointForwardPositions.Hybrid,
                         constant.POSITION_DELTA_THRESHOLD)

def InRobotAction() -> Action:
    return MoveArmAction(ArmUpperJointForwardPositions.InRobot,
                         ArmLowerJointForwardPositions.InRobot,
                         constant.POSITION_DELTA_THRESHOLD)

def GroundAction(reversed: bool) -> Action:
    return MoveArmAction(ArmUpperJointReversePositions.Ground if reversed else ArmUpperJointForwardPositions.Ground,
                         ArmLowerJointReversePositions.Ground if reversed else ArmLowerJointForwardPositions.Ground,
                         constant.POSITION_DELTA_THRESHOLD)