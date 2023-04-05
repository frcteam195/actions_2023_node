from actions_node.default_actions.Action import Action
from actions_node.default_actions.SeriesAction import SeriesAction
from actions_node.default_actions.ParallelAction import ParallelAction
from actions_node.default_actions.WaitAction import WaitAction
from actions_node.game_specific_actions.MoveArmAction import MoveArmAction
from actions_node.game_specific_actions.MoveWristAction import MoveWristAction
from actions_node.game_specific_actions.MoveArmExtensionAction import MoveArmExtensionAction
from actions_node.game_specific_actions.IntakeAction import IntakeAction
from actions_node.game_specific_actions.OuttakeAction import OuttakeAction
from actions_node.game_specific_actions.StopIntakeAction import StopIntakeAction
from actions_node.game_specific_actions.constant import ArmPosition, ArmExtensionPosition, WristPosition
from actions_node.game_specific_actions import constant

from ck_ros_msgs_node.msg import Arm_Goal

def ScoreCubeHigh(side: int) -> Action:
    """
    Moves to high cube position and outtake.
    """
    return SeriesAction([
        MoveArmAction(Arm_Goal.HIGH_CUBE, side),
        OuttakeAction(True, 0.2)
    ])

def ScoreConeHigh(side: int, wrist_goal: int = 0) -> Action:
    """
    Moves to high cone position, outtakes, and unpinches.
    """
    return SeriesAction([
        MoveArmAction(Arm_Goal.PRE_SCORE, side, wrist_goal, 7, 7),
        MoveArmAction(Arm_Goal.HIGH_CONE, side, wrist_goal),
        # WaitAction(0.30),
        StopIntakeAction(False, 0.1)
    ])

def ScoreCubeMiddle(side: int) -> Action:
    """
    Moves to middle cube position and outtake.
    """
    return SeriesAction([
        MoveArmAction(Arm_Goal.MID_CUBE, side),
        OuttakeAction(True, 0.2)
    ])

def ScoreConeMiddle(side: int, wrist_goal: int = 0) -> Action:
    """
    Moves to middle cone position, outtakes, and unpinches.
    """
    return SeriesAction([
        MoveArmAction(Arm_Goal.MID_CONE, side, wrist_goal),
        WaitAction(0.30),
        StopIntakeAction(False, 0.10)
    ])

def IntakeConeGround(side: int) -> Action:
    """
    Moves to the ground intake position and runs the intake.
    """
    return SeriesAction([
        ParallelAction([
            MoveArmAction(Arm_Goal.GROUND_CONE, side),
            IntakeAction(True, 0)
        ]),
        IntakeAction(True, 0.5)
    ])

def IntakeDeadCone(side: int, wrist: int) -> Action:
    """
    Intakes a dead cone.
    """
    return SeriesAction([
        IntakeAction(True, -1, 0.35),
        MoveArmAction(Arm_Goal.GROUND_DEAD_CONE, side, wrist, 5, 5),
        IntakeAction(True, 0.25, 0.35)
    ])
