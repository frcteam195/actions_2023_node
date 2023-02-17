import rospy
from actions_node.default_actions.DriveTrajectoryAction import DriveTrajectoryAction
from swerve_trajectory_node.srv import GetAutonomousInfo, GetAutonomousInfoResponse
from ck_utilities_py_node.geometry import *

class DriveTrajectoryActionIterator():
    def __init__(self, autonomous_name : str, expected_trajectory_count : int) -> None:
        self.__trajectory_count = 0
        self.__auto_start_pose : Pose = None
        self.__autonomous_name = autonomous_name
        self.__trajectory_index_iterator = 0
    
        self.__expected_trajectory_count = expected_trajectory_count
        rospy.wait_for_service('/get_autonomous_info')
        get_auto_info = rospy.ServiceProxy('/get_autonomous_info', GetAutonomousInfo)
        try:
            auto_info_response : GetAutonomousInfoResponse = get_auto_info(self.__autonomous_name)

            if auto_info_response is not None:
                self.__trajectory_count = auto_info_response.number_of_trajectories
                self.__auto_start_pose = Pose()
                self.__auto_start_pose.position.x = auto_info_response.x_inches
                self.__auto_start_pose.position.y = auto_info_response.y_inches
                self.__auto_start_pose.orientation.yaw = auto_info_response.heading_degrees

            if self.__expected_trajectory_count != self.__trajectory_count:
                rospy.logerr("Expected trajectory count for %s is not correct. Expecting %d but got %d" % (self.__autonomous_name, self.__expected_trajectory_count, self.__trajectory_count))

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed {e}" % e)
        
    def get_next_trajectory_action(self) -> DriveTrajectoryAction:
        curr_iterator = self.__trajectory_index_iterator
        self.__trajectory_index_iterator = self.__trajectory_index_iterator + 1

        if curr_iterator >= self.__trajectory_count:
            rospy.logerr("Index out of range for trajectory %d in auto %s" % (curr_iterator, self.__autonomous_name))
            return None

        if curr_iterator == 0:
            return DriveTrajectoryAction(self.__autonomous_name, curr_iterator, self.__auto_start_pose)
        else:
            return DriveTrajectoryAction(self.__autonomous_name, curr_iterator)