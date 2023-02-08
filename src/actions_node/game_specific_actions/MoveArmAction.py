from actions_node.default_actions.Action import Action
from ck_ros_msgs_node.msg import Arm_Control, Arm_Status
import rospy
from frc_robot_utilities_py_node.SubsystemController import SubsystemController
from ck_utilities_py_node.ckmath import *
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem
from actions_node.game_specific_actions.constant import ArmPosition
from actions_node.game_specific_actions import constant
class MoveArmAction(Action):
    arm_subsystem = SubsystemController[Arm_Control, Arm_Status]('ArmControl', Arm_Control, 'ArmStatus', Arm_Status)

    def __init__(self, arm__position : ArmPosition, reversed : bool, position_delta_threshold : float = constant.ARM_POSITION_DELTA_THRESHOLD):
        self.__Arm_Control_msg = Arm_Control()
        self.__Arm_Control_msg.arm_base_requested_position = constant.ArmLowerJointReversePositions[arm__position] if reversed else constant.ArmLowerJointForwardPositions[arm__position]
        self.__Arm_Control_msg.arm_upper_requested_position = constant.ArmUpperJointReversePositions[arm__position] if reversed else constant.ArmUpperJointForwardPositions[arm__position]
        self.__Arm_Control_msg.arm_wrist_requested_position = constant.WristPosition.Unchanged
        self.__Arm_Control_msg.extend = constant.ArmExtensionPosition.Unchanged
        self.__arm_position = arm__position
        self.__position_delta_threshold = position_delta_threshold

    #Do not call these methods directly
    def start(self):
        self.arm_subsystem.publish(self.__Arm_Control_msg)

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

        if self.__arm_position == ArmPosition.Unchanged:
            return True

        return within(self.arm_subsystem.get().arm_base_actual_position, self.__Arm_Control_msg.arm_base_requested_position, self.__position_delta_threshold) and \
            within(self.arm_subsystem.get().arm_upper_actual_position, self.__Arm_Control_msg.arm_upper_requested_position, self.__position_delta_threshold)

    #Do not call these methods directly
    def affectedSystems(self) -> List[Subsystem]:
        return { Subsystem.ARM }