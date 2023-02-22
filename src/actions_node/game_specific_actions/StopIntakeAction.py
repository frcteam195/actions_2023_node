from actions_node.default_actions.Action import Action
from ck_ros_msgs_node.msg import Intake_Control, Intake_Status
import rospy
from datetime import datetime
from frc_robot_utilities_py_node.SubsystemController import SubsystemController
from ck_utilities_py_node.ckmath import *
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem
from actions_node.game_specific_actions import constant

class StopIntakeAction(Action):
    """
    An action to run the intake in the outward direction.
    """

    intake_subsystem = SubsystemController[Intake_Control, Intake_Status]('IntakeControl', Intake_Control, 'IntakeStatus', Intake_Status)

    def __init__(self, pinched : bool):
        """
        Parameters
        ----------
        pinched : bool
            If True, the intake is pinched closed. If False, the intake is opened
        """

        self.__Intake_Control_msg = Intake_Control()
        self.__Intake_Control_msg.rollers_intake = False
        self.__Intake_Control_msg.rollers_outtake = False
        self.__Intake_Control_msg.pinched = pinched
        self.__start_time = datetime.now()

    #Do not call these methods directly
    def start(self):
        self.intake_subsystem.publish(self.__Intake_Control_msg)
        self.__start_time = datetime.now()

    #Do not call these methods directly
    def update(self):
        pass

    #Do not call these methods directly
    def done(self):
        pass

    #Do not call these methods directly
    def isFinished(self) -> bool:
        if self.intake_subsystem.get() is None:
            rospy.logerr("No status update present from intake")
            return False
        
        duration = datetime.now() - self.__start_time
        return duration.total_seconds() > constant.INTAKE_ACTUATION_TIME

    #Do not call these methods directly
    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.INTAKE ]