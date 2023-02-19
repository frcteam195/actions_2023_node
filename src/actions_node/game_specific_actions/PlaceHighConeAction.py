from actions_node.default_actions.Action import Action
from ck_ros_msgs_node.msg import Intake_Control, Intake_Status
import rospy
from frc_robot_utilities_py_node.SubsystemController import SubsystemController
from ck_utilities_py_node.ckmath import *
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem
from actions_node.game_specific_actions import constant

class PlaceHighConeAction(Action):
    """
    An action to place the High Cone
    """

    intake_subsystem = SubsystemController[Intake_Control, Intake_Status]('IntakeControl', Intake_Control, 'IntakeStatus', Intake_Status)

    def __init__(self, intake_delay_s : float = 0.05, intake_run_time_s : float = 0.030):
        self.__Intake_Control_msg = Intake_Control()
        self.__Intake_Control_msg.rollers_intake = False
        self.__Intake_Control_msg.rollers_outtake = True
        self.__Intake_Control_msg.pincher_solenoid_on = False
        self.__intake_delay_s = intake_delay_s
        self.__intake_run_time_s = intake_run_time_s
        self.__start_time = rospy.Time().now()

    #Do not call these methods directly
    def start(self):
        self.intake_subsystem.publish(self.__Intake_Control_msg)
        self.__start_time = rospy.Time().now()

    #Do not call these methods directly
    def update(self):
        duration = rospy.Time().now() - self.__start_time
        if duration.to_sec() > self.__intake_delay_s:
            self.__Intake_Control_msg.pincher_solenoid_on = True

        if duration.to_sec() > self.__intake_run_time_s:
            self.__Intake_Control_msg.rollers_outtake = False

        self.intake_subsystem.publish(self.__Intake_Control_msg)

    #Do not call these methods directly
    def done(self):
        self.__Intake_Control_msg.rollers_intake = False
        self.__Intake_Control_msg.rollers_outtake = False
        self.intake_subsystem.publish(self.__Intake_Control_msg)

    #Do not call these methods directly
    def isFinished(self) -> bool:
        if self.intake_subsystem.get() is None:
            rospy.logerr("No status update present from intake")
            return False

        duration = rospy.Time().now() - self.__start_time
        if self.__intake_run_time_s >= 0:
            return duration.to_sec() > constant.INTAKE_ACTUATION_TIME + self.__intake_run_time_s
        else:
            return duration.to_sec() > constant.INTAKE_ACTUATION_TIME

    #Do not call these methods directly
    def affectedSystems(self) -> List[Subsystem]:
        return { Subsystem.INTAKE }