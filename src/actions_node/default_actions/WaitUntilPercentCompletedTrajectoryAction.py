import rospy
from actions_node.default_actions.Action import Action
from typing import List
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from actions_node.game_specific_actions.Subsystem import Subsystem
from ck_ros_msgs_node.msg import Trajectory_Status
from ck_utilities_py_node.geometry import *


class WaitUntilPercentCompletedTrajectoryAction(Action):
    """An action that waits until a trajectory has reached a certain completion percentage"""

    def __init__(self, trajectory_index : int, percent_to_wait_until : float):
        register_for_robot_updates()
        self.__traj_status_subscriber = BufferedROSMsgHandlerPy(Trajectory_Status)
        self.__traj_status_subscriber.register_for_updates("/TrajectoryStatus")
        self.__trajectory_index = trajectory_index
        self.__trajectory_wait_until_percent = percent_to_wait_until

    def start(self):
        pass

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        traj_status : Trajectory_Status = self.__traj_status_subscriber.get()
        if traj_status is not None:
            return traj_status.progress >= self.__trajectory_wait_until_percent and traj_status.trajectory_index == self.__trajectory_index
        return False

    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.NONE ]