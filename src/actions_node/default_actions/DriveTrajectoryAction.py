import rospy
from actions_node.default_actions.Action import Action
from typing import List
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from actions_node.game_specific_actions.Subsystem import Subsystem
from swerve_trajectory_node.srv import StartTrajectory, StartTrajectoryResponse, GetStartPose, GetStartPoseResponse
from ck_ros_msgs_node.msg import Trajectory_Status

class DriveTrajectoryAction(Action):
    """An action that drives a trajectory and waits for completion before ending"""

    def __init__(self, trajectory : str, reset_pose : bool = False):
        self.__traj_status_subscriber = BufferedROSMsgHandlerPy(Trajectory_Status)
        self.__traj_status_subscriber.register_for_updates("/TrajectoryStatus")
        self.__trajectory_name = trajectory
        self.__reset_pose = reset_pose
        pass

    def start(self):
        if self.__reset_pose:
            get_pose = rospy.ServiceProxy('/get_start_pose', GetStartPose)
            start_pose: GetStartPoseResponse = get_pose(self.__trajectory_name)
            reset_robot_pose(start_pose.x_inches, start_pose.y_inches, start_pose.heading_degrees)

        auto_runner = rospy.ServiceProxy('/start_trajectory', StartTrajectory)
        auto_run_response : StartTrajectoryResponse = auto_runner(self.__trajectory_name)
        if not auto_run_response.accepted:
            rospy.logerr("Failed to start trajectory %s", self.__trajectory_name)

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