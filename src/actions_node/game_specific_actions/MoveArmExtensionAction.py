from actions_node.default_actions.Action import Action
from ck_ros_msgs_node.msg import Arm_Control, Arm_Status
import rospy
from datetime import datetime
from frc_robot_utilities_py_node.SubsystemController import SubsystemController
from ck_utilities_py_node.ckmath import *
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem
from actions_node.game_specific_actions import constant
from actions_node.game_specific_actions.constant import ArmExtensionPosition

class MoveArmExtensionAction(Action):
    """
    An action to control the arm extension stage.
    """

    arm_subsystem = SubsystemController[Arm_Control, Arm_Status]('ArmControl', Arm_Control, 'ArmStatus', Arm_Status)

    def __init__(self, extend : ArmExtensionPosition):
        """
        Parameters
        ----------
        extend : bool
            If True, the arm is extended. If False, the arm is retracted
        """

        self.__Arm_Control_msg = Arm_Control()
        self.__Arm_Control_msg.extend = extend
        self.__Arm_Control_msg.arm_base_requested_position = constant.ArmPosition.Unchanged
        self.__Arm_Control_msg.arm_upper_requested_position = constant.ArmPosition.Unchanged
        self.__Arm_Control_msg.arm_wrist_requested_position = constant.WristPosition.Unchanged
        self.__extend_position = extend
        self.__start_time = datetime.now()

    #Do not call these methods directly
    def start(self):
        self.arm_subsystem.publish(self.__Arm_Control_msg)
        self.__start_time = datetime.now()

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
        
        if self.__extend_position == ArmExtensionPosition.Unchanged:
            return True

        duration = datetime.now() - self.__start_time
        return duration.total_seconds() > constant.ARM_EXTENSION_ACTUATION_TIME

    #Do not call these methods directly
    def affectedSystems(self) -> List[Subsystem]:
        return { Subsystem.ARM }