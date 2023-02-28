from actions_node.default_actions.Action import Action
import rospy
from frc_robot_utilities_py_node.SubsystemController import SubsystemController
from ck_utilities_py_node.ckmath import *
from typing import List, Callable
from actions_node.game_specific_actions.Subsystem import Subsystem
from limelight_vision_node.msg import Limelight_Control, Limelight_Status, Limelight, Limelight_Info

class ConeNodeAutoAlignmentAction(Action):
    limelight_subsystem = SubsystemController[Limelight_Control, Limelight_Status]('LimelightControl', Limelight_Control, 'LimelightStatus', Limelight_Status)

    def __init__(self, is_finished_condition : Callable[[], bool]):
        self.__is_finished_condition = is_finished_condition
        pass

    def start(self):
        #Enable Limelight Vision Pipeline
        ll : Limelight = Limelight_Control()
        ll.name = ""
        ll.pipeline = 1
        
        ll_ctrl : Limelight_Control = Limelight_Control()
        ll_ctrl.limelights.append()
        #Switch drivetrain to listen to input from limelight node alignment twist
        pass

    def update(self):
        #Send alignment info to drivetrain (Alternatively, have it listen to limelight node)
        pass

    def done(self):
        #Turn off Limelight Vision Pipeline
        pass

    def isFinished(self) -> bool:
        return self.__is_finished_condition()

    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.DRIVEBASE ]
