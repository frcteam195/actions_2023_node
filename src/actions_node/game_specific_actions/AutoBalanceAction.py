from enum import Enum
from math import degrees, radians
from typing import List

import rospy

from actions_node.default_actions.Action import Action
from actions_node.game_specific_actions.Subsystem import Subsystem

from ck_ros_msgs_node.msg import Led_Control, Swerve_Drivetrain_Auto_Control
from nav_msgs.msg import Odometry
from swerve_trajectory_node.srv import StopTrajectory, StopTrajectoryResponse

from ck_utilities_py_node.ckmath import limit
from ck_utilities_py_node.geometry import *

from frc_robot_utilities_py_node.frc_robot_utilities_py import *

NUM_LEDS = 50
fire_animation = Led_Control(Led_Control.ANIMATE, Led_Control.FIRE, 0, 0, 0, 0, 0.5, 0.5, NUM_LEDS, "")
twinkle_purple = Led_Control(Led_Control.ANIMATE, Led_Control.TWINKLE_OFF, 57, 3, 87, 0, 0, 0.25, NUM_LEDS, "")

class BalanceDirection(Enum):
    """
    The lateral direction for balancing.
    """
    PITCH = 0
    ROLL = 1

class AutoBalanceAction(Action):
    """
    An action that commands a twist to the swerve drivetrain to balance on the charge station.
    """

    def __init__(self, balance_direction: BalanceDirection, desired_heading: int = 0):
        rospy.logdebug("Init Auto-Balance Action")

        self.__imu_subscriber = BufferedROSMsgHandlerPy(Odometry)
        self.__imu_subscriber.register_for_updates("/RobotIMU")

        self.__balance_direction = balance_direction
        self.__desired_heading = radians(desired_heading)

        self.__desired_rotation = Rotation()
        self.__desired_rotation.yaw = self.__desired_heading
        self.__desired_quat = self.__desired_rotation.to_msg_quat()

        self.__drive_twist_publisher = rospy.Publisher(name="/SwerveAutoControl", data_class=Swerve_Drivetrain_Auto_Control, queue_size=10, tcp_nodelay=True)

        self.__current_leds = twinkle_purple
        self.__led_control_publisher = rospy.Publisher(name="/LedControl", data_class=Led_Control, queue_size=10, tcp_nodelay=True)

        self.__imu_pose = Pose()
        self.__imu_twist = Twist()

        self.__tipped = 0

        self.__start_time = 0

    def start(self):
        rospy.logdebug("Starting Auto-Balance Action")
        rospy.logerr("Starting Auto-Balance Action")
        rospy.wait_for_service('/stop_trajectory')
        stop_traj = rospy.ServiceProxy('/stop_trajectory', StopTrajectory)
        stop_traj_response: StopTrajectoryResponse = stop_traj()

        self.__tipped = 0
        self.__start_time = rospy.get_time()
        if not stop_traj_response.accepted:
            rospy.logerr("Auto-balance failed to stop trajectory!")

    def update(self):

        imu_data: Odometry = self.__imu_subscriber.get()

        if imu_data is not None:
            self.__imu_pose: Pose = Pose(imu_data.pose.pose)
            self.__imu_twist: Twist = Twist(imu_data.twist.twist)

            control_msg: Swerve_Drivetrain_Auto_Control = Swerve_Drivetrain_Auto_Control()
            control_msg.pose.orientation = self.__desired_quat

            control_msg.twist = self.__determine_twist_direction().to_msg()

            # Determine the angle/rate to use for calculation.
            attitude = 0.0
            attitude_rate = 0.0

            if self.__balance_direction == BalanceDirection.PITCH:
                attitude = self.__imu_pose.orientation.pitch
                attitude_rate = self.__imu_twist.angular.pitch
            elif self.__balance_direction == BalanceDirection.ROLL:
                attitude = self.__imu_pose.orientation.roll
                attitude_rate = self.__imu_twist.angular.roll

            # rospy.loginfo(f"Pitch: {degrees(self.__imu_pose.orientation.pitch)} Roll: {degrees(self.__imu_pose.orientation.roll)}")
            # rospy.loginfo(f"PitchRate: {degrees(self.__imu_twist.angular.pitch)},  RollRate: {degrees(self.__imu_twist.angular.roll)}")

            KP = 0.005
            BASE_MAX_SPEED = 0.48

            output = KP * np.sign(attitude) * pow(degrees(attitude), 2)
            output = limit(output, -BASE_MAX_SPEED, BASE_MAX_SPEED)

            control_msg.twist.linear.x = control_msg.twist.linear.x * output
            control_msg.twist.linear.y = control_msg.twist.linear.y * output

            time_passed = rospy.get_time() - self.__start_time
            if time_passed > 0.075 and abs(attitude_rate) > 0.28:
                control_msg.x_mode = True
                self.__tipped += 1
            else:
                self.__tipped = 0

            self.__drive_twist_publisher.publish(control_msg)

            self.__led_control_publisher.publish(self.__current_leds)

    def done(self):
        control_msg: Swerve_Drivetrain_Auto_Control = Swerve_Drivetrain_Auto_Control()
        control_msg.pose.orientation = self.__desired_quat
        control_msg.x_mode = True
        self.__drive_twist_publisher.publish(control_msg)

        self.__current_leds = fire_animation
        self.__led_control_publisher.publish(self.__current_leds)

    def isFinished(self) -> bool:
        return self.__tipped > 1

    def affectedSystems(self) -> List[Subsystem]:
        return [Subsystem.DRIVEBASE]

    def __determine_twist_direction(self) -> Twist:
        """
        Returns a twist with the correct linear axis as 1.0.
        """
        twist = Twist()

        if self.__balance_direction == BalanceDirection.PITCH:
            twist.linear.x = 1.0
        elif self.__balance_direction == BalanceDirection.ROLL:
            twist.linear.y = 1.0

        return twist
