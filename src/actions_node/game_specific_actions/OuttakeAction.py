from actions_node.default_actions.Action import Action
from ck_ros_msgs_node.msg import Intake_Control, Intake_Status
import rospy
from datetime import datetime
from frc_robot_utilities_py_node.SubsystemController import SubsystemController
from ck_utilities_py_node.ckmath import *
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem
from actions_node.game_specific_actions import constant

class OuttakeAction(Action):
    """
    An action to run the intake in the outward direction.
    """

    intake_subsystem = SubsystemController[Intake_Control, Intake_Status]('IntakeControl', Intake_Control, 'IntakeStatus', Intake_Status)

    def __init__(self, pinched : bool, time_to_outtake_s : float = -1):
        """
        Parameters
        ----------
        pinched : bool
            If True, the intake is pinched closed. If False, the intake is opened
        time_to_outtake_s : float
            An optional parameter of how long to run the intake for.
            Not specifying this parameter will just turn the intake on without stopping it at the end of the action.
            (You would need to run a StopIntakeAction)
        """
        
        self.__Intake_Control_msg = Intake_Control()
        self.__Intake_Control_msg.rollers_intake = False
        self.__Intake_Control_msg.rollers_outtake = True
        self.__Intake_Control_msg.pinched = pinched
        self.__time_to_outtake_s = time_to_outtake_s
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
        if self.__time_to_outtake_s >= 0:
            self.__Intake_Control_msg.rollers_intake = False
            self.__Intake_Control_msg.rollers_outtake = False
            self.intake_subsystem.publish(self.__Intake_Control_msg)

    #Do not call these methods directly
    def isFinished(self) -> bool:
        if self.intake_subsystem.get() is None:
            rospy.logerr("No status update present from intake")
            return False
        
        duration = datetime.now() - self.__start_time
        if self.__time_to_outtake_s >= 0:
            return duration.total_seconds() > constant.INTAKE_ACTUATION_TIME + self.__time_to_outtake_s
        else:
            return duration.total_seconds() > constant.INTAKE_ACTUATION_TIME

    #Do not call these methods directly
    def affectedSystems(self) -> List[Subsystem]:
        return { Subsystem.INTAKE }