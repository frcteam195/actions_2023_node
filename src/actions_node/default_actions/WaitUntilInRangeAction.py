import rospy
from actions_node.default_actions.Action import Action
from typing import List
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from actions_node.game_specific_actions.Subsystem import Subsystem
from ck_utilities_py_node.geometry import *
from ck_utilities_py_node.geometry_helper import *

from nav_msgs.msg import Odometry

class WaitUntilInRangeAction(Action):
    """An action that waits until the robot is in a specified range drawn by the rectangle points provided"""

    def __init__(self, range : Rectangle):
        self.__odometry_subscriber = BufferedROSMsgHandlerPy(Odometry)
        self.__odometry_subscriber.register_for_updates("odometry/filtered")
        self.__rectangular_range = RectangularRange2D(range)

    def start(self):
        pass

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        odometry_msg : Odometry = self.__odometry_subscriber.get()
        if odometry_msg is not None:
            return self.__rectangular_range.is_within_range_2d(Pose(odometry_msg.pose.pose))
        rospy.logerr("No status update present from odometry")
        return False

    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.NONE ]