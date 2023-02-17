import rospy
from actions_node.default_actions.Action import Action
from typing import List
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from actions_node.game_specific_actions.Subsystem import Subsystem
from swerve_trajectory_node.srv import StartTrajectory, StartTrajectoryResponse, GetStartPose, GetStartPoseResponse
from ck_ros_msgs_node.msg import Trajectory_Status
from ck_utilities_py_node.geometry import *

class DriveTrajectoryAction(Action):
    """An action that drives a trajectory and waits for completion before ending"""

    def __init__(self, autonomous_name : str, trajectory_index : int, start_pose : Pose = None):
        self.__traj_status_subscriber = BufferedROSMsgHandlerPy(Trajectory_Status)
        self.__traj_status_subscriber.register_for_updates("/TrajectoryStatus")
        self.__autonomous_name = autonomous_name
        self.__trajectory_index = trajectory_index
        self.__start_pose : Pose = start_pose

    def start(self):
        if self.__start_pose is not None:
            reset_robot_pose(self.__start_pose.position.x, self.__start_pose.position.y, self.__start_pose.orientation.yaw)

        auto_runner = rospy.ServiceProxy('/start_trajectory', StartTrajectory)
        auto_run_response : StartTrajectoryResponse = auto_runner(self.__autonomous_name, self.__trajectory_index)
        if not auto_run_response.accepted:
            rospy.logerr("Failed to start trajectory %s: %d", self.__autonomous_name, self.__trajectory_index)

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        traj_status : Trajectory_Status = self.__traj_status_subscriber.get()
        if traj_status is not None:
            return traj_status.is_completed
        return False

    def affectedSystems(self) -> List[Subsystem]:
        return { Subsystem.DRIVEBASE }