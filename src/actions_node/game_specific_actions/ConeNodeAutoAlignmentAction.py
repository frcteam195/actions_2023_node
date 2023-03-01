from actions_node.default_actions.Action import Action
import rospy
from frc_robot_utilities_py_node.SubsystemController import SubsystemController
from ck_utilities_py_node.ckmath import *
from typing import List, Callable
from actions_node.game_specific_actions.Subsystem import Subsystem
from limelight_vision_node.msg import Limelight_Control, Limelight_Status, Limelight, Limelight_Info
from enum import Enum
from ck_ros_msgs_node.msg import Arm_Goal
from ck_utilities_py_node.pid_controller import PIDController
from ck_ros_msgs_node.msg import Swerve_Drivetrain_Auto_Control


class LimelightNames(str, Enum):
    Front = "limelight-front"
    Back = "limelight-back"
    Arm = "limelight-arm"

    def __str__(self) -> str:
        return str.__str__(self)

limelight_vision_pipelines = {
    {LimelightNames.Front, 1},
    {LimelightNames.Back, 1},
    {LimelightNames.Arm, 2},
}

class ConeNodeAutoAlignmentAction(Action):
    limelight_subsystem = SubsystemController[Limelight_Control, Limelight_Status]('LimelightControl', Limelight_Control, 'LimelightStatus', Limelight_Status)

    def __init__(self, side : int, is_finished_condition : Callable[[], bool]):
        self.__is_finished_condition = is_finished_condition
        self.__side = side
        self.__active_limelight_name = LimelightNames.Front if side == Arm_Goal.SIDE_FRONT else LimelightNames.Back
        self.__pid_controller = PIDController(kP=0.003, kD=0.0025, filter_r=0.4)
        self.__twist_pub = rospy.Publisher(name="SwerveAutoControl", data_class=Swerve_Drivetrain_Auto_Control,queue_size=10,tcp_nodelay=True)
        pass

    def start(self):
        #Enable Limelight Vision Pipeline
        ll : Limelight = Limelight_Control()
        ll.name = self.__active_limelight_name.value
        ll.pipeline = limelight_vision_pipelines[self.__active_limelight_name]
        ll_ctrl : Limelight_Control = Limelight_Control()
        ll_ctrl.limelights.append()
        self.limelight_subsystem.publish(ll_ctrl)
        #Switch drivetrain to listen to input from limelight node alignment twist
        pass

    def update(self):
        #Send alignment info to drivetrain (Alternatively, have it listen to limelight node)
        ll_status : Limelight_Status = self.limelight_subsystem.get()
        active_limelight_info : Limelight_Info = None
        for l in ll_status.limelights:
            l : Limelight_Info = l
            if l.name == self.__active_limelight_name.value:
                active_limelight_info = l
                break
        tx = active_limelight_info.target_dx_deg
        ta = active_limelight_info.target_area
        tv = active_limelight_info.target_valid

        if tv:
            output = self.__pid_controller.update(0, tx)
            sdac = Swerve_Drivetrain_Auto_Control()
            sdac.twist.linear.y = output
            self.__twist_pub.publish(sdac)
            #adjust output by ta?


    def done(self):
        #Turn off Limelight Vision Pipeline
        ll : Limelight = Limelight_Control()
        ll.name = self.__active_limelight_name.value
        ll.pipeline = limelight_vision_pipelines[self.__active_limelight_name] - 1
        ll_ctrl : Limelight_Control = Limelight_Control()
        ll_ctrl.limelights.append()
        self.limelight_subsystem.publish(ll_ctrl)
        pass

    def isFinished(self) -> bool:
        return self.__is_finished_condition()

    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.DRIVEBASE ]
