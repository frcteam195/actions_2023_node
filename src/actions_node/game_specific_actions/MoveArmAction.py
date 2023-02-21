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

    def __init__(self, arm_goal : int, arm_goal_side : int, wrist_goal : int = 0):
        self.__Arm_Goal_msg = Arm_Goal()
        self.__Arm_Goal_msg.goal = arm_goal
        self.__Arm_Goal_msg.goal_side = arm_goal_side
        self.__Arm_Goal_msg.wrist_goal = wrist_goal

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
        if self.arm_subsystem.get() is None:
            rospy.logerr("No status update present from arm")
            return False

        return self.arm_subsystem.get().goal.goal == self.__Arm_Goal_msg.goal and self.arm_subsystem.get().arm_at_setpoint

    #Do not call these methods directly
    def affectedSystems(self) -> List[Subsystem]:
        return { Subsystem.ARM }
