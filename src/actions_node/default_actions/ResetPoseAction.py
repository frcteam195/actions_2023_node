import rospy
from actions_node.default_actions.Action import Action
from swerve_trajectory_node.srv import GetAutonomousInfo, GetAutonomousInfoResponse, ResetPoseWithConfirmation, ResetPoseWithConfirmationRequest, ResetPoseWithConfirmationResponse
from ck_utilities_py_node.geometry import *
from actions_node.game_specific_actions.Subsystem import Subsystem
from typing import List

class ResetPoseAction(Action):
    def __init__(self, autonomous_name : str) -> None:
        self.__autonomous_name = autonomous_name
        self.__reset_pose_successful = False

    def start(self):
        rospy.wait_for_service('/reset_pose_with_confirmation')
        reset_pose_service = rospy.ServiceProxy('/reset_pose_with_confirmation', ResetPoseWithConfirmation)
        reset_pose_req : ResetPoseWithConfirmationRequest = ResetPoseWithConfirmationRequest()
        reset_pose_req.auto_name = self.__autonomous_name
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
