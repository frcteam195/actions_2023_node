import rospy
from actions_node.default_actions.Action import Action
from typing import List
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from actions_node.game_specific_actions.Subsystem import Subsystem
from swerve_trajectory_node.srv import StartTrajectory, StartTrajectoryResponse
from ck_ros_msgs_node.msg import Trajectory_Status
from ck_utilities_py_node.geometry import *


class DriveTrajectoryAction(Action):
    """An action that drives a trajectory and waits for completion before ending"""

    def __init__(self, autonomous_name : str, trajectory_index : int):
        register_for_robot_updates()
        self.__traj_status_subscriber = BufferedROSMsgHandlerPy(Trajectory_Status)
        self.__traj_status_subscriber.register_for_updates("/TrajectoryStatus")
        self.__autonomous_name = autonomous_name
        self.__trajectory_index = trajectory_index

    def start(self):
        rospy.wait_for_service('/start_trajectory')
        auto_runner = rospy.ServiceProxy('/start_trajectory', StartTrajectory)
        auto_run_response : StartTrajectoryResponse = auto_runner(self.__autonomous_name, self.__trajectory_index)
        if not auto_run_response.accepted:
            rospy.logerr(f"Failed to start trajectory {self.__autonomous_name}: {self.__trajectory_index}")

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        traj_status : Trajectory_Status = self.__traj_status_subscriber.get()
        if traj_status is not None:
            return traj_status.is_completed and traj_status.trajectory_index == self.__trajectory_index
        return False

    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.DRIVEBASE ]