import rospy
from actions_node.default_actions.Action import Action
from swerve_trajectory_node.srv import GetAutonomousInfo, GetAutonomousInfoResponse, ResetPoseWithConfirmation, ResetPoseWithConfirmationRequest, ResetPoseWithConfirmationResponse
from ck_utilities_py_node.geometry import *
from actions_node.game_specific_actions.Subsystem import Subsystem
from typing import List

class ResetPoseAction(Action):
    def __init__(self, autonomous_name : str) -> None:
        self.__auto_start_pose : Pose = None
        self.__autonomous_name = autonomous_name
        self.__reset_pose_successful = False
    
        rospy.wait_for_service('/get_autonomous_info')
        get_auto_info = rospy.ServiceProxy('/get_autonomous_info', GetAutonomousInfo)
        try:
            auto_info_response : GetAutonomousInfoResponse = get_auto_info(self.__autonomous_name)

            if auto_info_response is not None:
                self.__auto_start_pose = Pose()
                self.__auto_start_pose.position.x = auto_info_response.x_inches
                self.__auto_start_pose.position.y = auto_info_response.y_inches
                self.__auto_start_pose.orientation.yaw = auto_info_response.heading_degrees
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed {e}")

    def start(self):
        rospy.wait_for_service('/reset_pose_with_confirmation')
        reset_pose_service = rospy.ServiceProxy('/reset_pose_with_confirmation', ResetPoseWithConfirmation)
        reset_pose_req : ResetPoseWithConfirmationRequest = ResetPoseWithConfirmationRequest()
        reset_pose_req.x_inches = self.__auto_start_pose.position.x
        reset_pose_req.y_inches = self.__auto_start_pose.position.y
        reset_pose_req.heading_degrees = self.__auto_start_pose.orientation.yaw
        reset_pose_response : ResetPoseWithConfirmationResponse = reset_pose_service(reset_pose_req)
        if reset_pose_response is None:
            rospy.logerr(f"Failed to reset pose for {self.__autonomous_name}!")
        else:
            if not reset_pose_response.completed:
                rospy.logerr(f"Failed to reset pose for {self.__autonomous_name}!")
            self.__reset_pose_successful = reset_pose_response.completed
            

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__reset_pose_successful

    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.NONE ]