from actions_node.default_actions.Action import Action
from ck_ros_msgs_node.msg import Arm_Goal, Arm_Status
import rospy
from frc_robot_utilities_py_node.SubsystemController import SubsystemController
from ck_utilities_py_node.ckmath import *
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem
from actions_node.game_specific_actions import constant

class MoveArmAction(Action):
    arm_subsystem = SubsystemController[Arm_Goal, Arm_Status]('ArmGoal', Arm_Goal, 'ArmStatus', Arm_Status)

    def __init__(self, arm_goal : int, arm_goal_side : int, wrist_goal : int = 0, base_deviation_tolerance_deg : int = -1, upper_deviation_tolerance_deg : int = -1):
        self.__Arm_Goal_msg = Arm_Goal()
        self.__Arm_Goal_msg.goal = arm_goal
        self.__Arm_Goal_msg.goal_side = arm_goal_side
        self.__Arm_Goal_msg.wrist_goal = wrist_goal
        self.__base_deviation_tolerance_deg = abs(base_deviation_tolerance_deg)
        self.__upper_deviation_tolerance_deg = abs(upper_deviation_tolerance_deg)

    #Do not call these methods directly
    def start(self):
        self.arm_subsystem.publish(self.__Arm_Goal_msg)

    #Do not call these methods directly
    def update(self):
        pass

    #Do not call these methods directly
    def done(self):
        pass

    #Do not call these methods directly
    def isFinished(self) -> bool:
        arm_status : Arm_Status = self.arm_subsystem.get()
        if arm_status is None:
            rospy.logerr("No status update present from arm")
            return False
        
        is_correct_arm_goal = arm_status.goal.goal == self.__Arm_Goal_msg.goal

        if (self.__base_deviation_tolerance_deg <= 0 and self.__upper_deviation_tolerance_deg <= 0):
            return is_correct_arm_goal and arm_status.arm_at_setpoint
        elif (self.__base_deviation_tolerance_deg <= 0 and self.__upper_deviation_tolerance_deg > 0):
            return is_correct_arm_goal and abs(arm_status.arm_upper_deviation_deg) < self.__upper_deviation_tolerance_deg
        elif (self.__base_deviation_tolerance_deg > 0 and self.__upper_deviation_tolerance_deg <= 0):
            return is_correct_arm_goal and abs(arm_status.arm_base_deviation_deg) < self.__base_deviation_tolerance_deg
        elif (self.__base_deviation_tolerance_deg > 0 and self.__upper_deviation_tolerance_deg > 0):
            return is_correct_arm_goal and abs(arm_status.arm_base_deviation_deg) < self.__base_deviation_tolerance_deg \
                and abs(arm_status.arm_upper_deviation_deg) < self.__upper_deviation_tolerance_deg


    #Do not call these methods directly
    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.ARM ]
