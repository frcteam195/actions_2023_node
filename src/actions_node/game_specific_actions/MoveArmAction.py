from actions_node.default_actions.Action import Action
from ck_ros_msgs_node.msg import Arm_Goal, Arm_Status
import rospy
from frc_robot_utilities_py_node.SubsystemController import SubsystemController
from ck_utilities_py_node.ckmath import *
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem
from actions_node.game_specific_actions import constant


class MoveArmAction(Action):
    """
    Action to control the arm and wrist in autonomous.
    """
    arm_subsystem = SubsystemController[Arm_Goal, Arm_Status]('ArmGoal', Arm_Goal, 'ArmStatus', Arm_Status)

    def __init__(self, arm_goal: int, arm_goal_side: int, base_deviation_tolerance_deg: int = -1, upper_deviation_tolerance_deg: int = -1, wrist_deviation_tolerance_deg: int = -1):
        self.__Arm_Goal_msg = Arm_Goal()
        self.__Arm_Goal_msg.goal = arm_goal
        self.__Arm_Goal_msg.goal_side = arm_goal_side
        self.__base_deviation_tolerance_deg = base_deviation_tolerance_deg
        self.__upper_deviation_tolerance_deg = upper_deviation_tolerance_deg
        self.__wrist_deviation_tolerance_deg = wrist_deviation_tolerance_deg

    # Do not call these methods directly
    def start(self):
        self.arm_subsystem.publish(self.__Arm_Goal_msg)

    # Do not call these methods directly
    def update(self):
        pass

    # Do not call these methods directly
    def done(self):
        pass

    def isFinished(self) -> bool:
        """
        Reports if the state is finished, to allow for transition.
        Do not call this method directly.
        """
        arm_status: Arm_Status = self.arm_subsystem.get()

        if arm_status is None:
            rospy.logerr("No status update present from arm.")
            return False

        # If not at the correct arm goal, then the movement is not finished.
        if arm_status.goal.goal != self.__Arm_Goal_msg.goal:
            return False

        # If all the tolerances are negative, use the "at setpoint" status.
        if self.__base_deviation_tolerance_deg <= 0 and self.__upper_deviation_tolerance_deg <= 0 and self.__wrist_deviation_tolerance_deg <= 0:
            return arm_status.arm_at_setpoint

        # Determine whether the arm is within the tolerances provided.
        base_arm_at_setpoint = True
        if self.__base_deviation_tolerance_deg > 0:
            base_arm_at_setpoint = abs(arm_status.arm_base_deviation_deg) < self.__base_deviation_tolerance_deg

        upper_arm_at_setpoint = True
        if self.__upper_deviation_tolerance_deg > 0:
            upper_arm_at_setpoint = abs(arm_status.arm_upper_deviation_deg) < self.__upper_deviation_tolerance_deg

        wrist_arm_at_setpoint = True
        if self.__wrist_deviation_tolerance_deg > 0:
            wrist_arm_at_setpoint = abs(arm_status.arm_wrist_deviation_deg) < self.__wrist_deviation_tolerance_deg

        return base_arm_at_setpoint and upper_arm_at_setpoint and wrist_arm_at_setpoint

    # Do not call these methods directly
    def affectedSystems(self) -> List[Subsystem]:
        return [Subsystem.ARM]
