import rospy
from actions_node.default_actions.DriveTrajectoryAction import DriveTrajectoryAction
from swerve_trajectory_node.srv import GetAutonomousInfo, GetAutonomousInfoResponse
from ck_utilities_py_node.geometry import *

class DriveTrajectoryActionIterator():
    def __init__(self, autonomous_name : str, expected_trajectory_count : int) -> None:
        self.__trajectory_count = 0
        self.__autonomous_name = autonomous_name
        self.__trajectory_index_iterator = 0
    
        self.__expected_trajectory_count = expected_trajectory_count
        rospy.wait_for_service('/get_autonomous_info')
        get_auto_info = rospy.ServiceProxy('/get_autonomous_info', GetAutonomousInfo)
        try:
            auto_info_response : GetAutonomousInfoResponse = get_auto_info(self.__autonomous_name)

            if auto_info_response is not None:
                self.__trajectory_count = auto_info_response.number_of_trajectories

            if self.__expected_trajectory_count != self.__trajectory_count:
                rospy.logerr(f"Expected trajectory count for {self.__autonomous_name} is not correct. Expecting {self.__expected_trajectory_count} but got {self.__trajectory_count}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed {e}")
        
    def get_next_trajectory_action(self) -> DriveTrajectoryAction:
        curr_iterator = self.__trajectory_index_iterator
        self.__trajectory_index_iterator = self.__trajectory_index_iterator + 1

        if curr_iterator >= self.__trajectory_count:
            rospy.logerr(f"Index out of range for trajectory {curr_iterator} in auto {self.__autonomous_name}")
            return None

        return DriveTrajectoryAction(self.__autonomous_name, curr_iterator)
        
    def reset_iterator(self):
        self.__trajectory_index_iterator = 0