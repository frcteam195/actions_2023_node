from enum import Enum
from typing import List

import rospy

from actions_node.default_actions.Action import Action
from actions_node.game_specific_actions.Subsystem import Subsystem

from ck_ros_msgs_node.msg import Swerve_Drivetrain_Auto_Control
from nav_msgs.msg import Odometry
from swerve_trajectory_node.srv import StopTrajectory, StopTrajectoryResponse

from ck_utilities_py_node.geometry import *
from ck_utilities_py_node.pid_controller import PIDController
from ck_utilities_py_node.moving_average import MovingAverage
from ck_utilities_py_node.ckmath import limit
from math import degrees, radians

from frc_robot_utilities_py_node.frc_robot_utilities_py import *


class BalanceDirection(Enum):
    """
    The lateral direction for balancing.
    """
    PITCH = 0
    ROLL = 1


class RobotDirection(Enum):
    """
    Direction the robot is driving towards the charge station.
    """
    FRONT = 0
    BACK = 1


class AutoBalanceState(Enum):
    InitialDrive = 0,
    Backup = 1,
    Done = 2


class AutoBalanceAction(Action):
    """
    An action that commands a twist to the swerve drivetrain to balance on the charge station.
    """

    def __init__(self, balance_direction: BalanceDirection, balance_threshold_deg: float, desired_robot_direction: RobotDirection):
        rospy.logdebug("Init Auto-Balance Action")

        self.__imu_subscriber = BufferedROSMsgHandlerPy(Odometry)
        self.__imu_subscriber.register_for_updates("/RobotIMU")

        self.__odometry_subscriber = BufferedROSMsgHandlerPy(Odometry)
        self.__odometry_subscriber.register_for_updates("/odometry/filtered")

        self.__balance_direction = balance_direction
        self.__desired_heading = 0
        self.__pitch_rate_average = MovingAverage(10)
        alliance = robot_status.get_alliance()
        self.__alliance = alliance
        self.__desired_robot_direction = desired_robot_direction
        self.__current_state = AutoBalanceState.InitialDrive
        self.__initial_backup_pos = 0
        if alliance == Alliance.RED:
            if desired_robot_direction == RobotDirection.FRONT:
                self.__desired_heading = 0
            else:
                self.__desired_heading = math.pi
        elif alliance == Alliance.BLUE:
            if desired_robot_direction == RobotDirection.FRONT:
                self.__desired_heading = math.pi
            else:
                self.__desired_heading = 0
        else:
            rospy.logerr("No valid alliance color selected")
            self.__desired_heading = 0
        rospy.logerr(self.__desired_heading)
        self.__desired_rotation = Rotation()
        self.__desired_rotation.yaw = self.__desired_heading
        self.__desired_quat = self.__desired_rotation.to_msg_quat()
        self.__balance_threshold = math.radians(balance_threshold_deg)
        self.__balance_pid = PIDController(kP=1.4, kD=0.6, filter_r=0.6)
        self.__drive_twist_publisher = rospy.Publisher(name="/SwerveAutoControl", data_class=Swerve_Drivetrain_Auto_Control, queue_size=10, tcp_nodelay=True)
        self.__current_pose = Pose()
        self.__start_time = 0
        self.__level_time = 0
        self.__imu_pose = Pose()

    def start(self):
        rospy.logdebug("Starting Auto-Balance Action")
        rospy.logerr("Starting Auto-Balance Action")
        rospy.wait_for_service('/stop_trajectory')
        stop_traj = rospy.ServiceProxy('/stop_trajectory', StopTrajectory)
        stop_traj_response: StopTrajectoryResponse = stop_traj()

        self.__start_time = rospy.Time.now().to_sec()

        if not stop_traj_response.accepted:
            rospy.logerr("Auto-balance failed to stop trajectory!")

    def update(self):

        filtered_odometry_data: Odometry = self.__odometry_subscriber.get()

        if filtered_odometry_data is not None:
            self.__current_pose = Pose(filtered_odometry_data.pose.pose)

        imu_data: Odometry = self.__imu_subscriber.get()

        if imu_data is not None:
            current_twist: Twist = Twist(imu_data.twist.twist)
            self.__imu_pose: Pose = Pose(imu_data.pose.pose)

            control_msg: Swerve_Drivetrain_Auto_Control = Swerve_Drivetrain_Auto_Control()
            control_msg.pose.orientation = self.__desired_quat
            control_msg.pose.position.x = 11

            self.__pitch_rate_average.add_sample(current_twist.angular.pitch)
            rospy.logerr(f"Pitch: {degrees(self.__imu_pose.orientation.pitch)} Roll: {degrees(self.__imu_pose.orientation.roll)} PitchRate: {current_twist.angular.pitch}")

            control_msg.twist = self.__calculate_twist(False).to_msg()

            max_pitch_error = 0.35


            sign = np.sign(self.__imu_pose.orientation.pitch)
            pitch_error = degrees(self.__imu_pose.orientation.pitch)
            pitch_error = pitch_error * pitch_error
            pitch_error = pitch_error * 0.0035 * sign

            pitch_error = limit(pitch_error, -max_pitch_error, max_pitch_error)

            control_msg.twist.linear.x = control_msg.twist.linear.x * pitch_error
            control_msg.twist.linear.y = control_msg.twist.linear.y * pitch_error

            if abs(current_twist.angular.pitch) > 0.23:
                control_msg.twist.linear.x = 0
                control_msg.twist.linear.y = 0

            self.__drive_twist_publisher.publish(control_msg)

    def done(self):
        control_msg: Swerve_Drivetrain_Auto_Control = Swerve_Drivetrain_Auto_Control()
        control_msg.pose.orientation = self.__desired_quat
        self.__drive_twist_publisher.publish(control_msg)

    def isFinished(self) -> bool:
        offset = abs(self.__imu_pose.orientation.pitch) + abs(self.__imu_pose.orientation.roll)
        offset = degrees(offset)
        if offset < 5:
            self.__level_time = rospy.Time.now().to_sec()
            rospy.logerr(f"Level atm: {offset}")
        else:
            self.__start_time = rospy.Time.now().to_sec()
            rospy.logerr(f"Resetting the counter: {offset}")
        return self.__level_time > self.__start_time + 1

    def affectedSystems(self) -> List[Subsystem]:
        return [Subsystem.DRIVEBASE]

    def __calculate_twist(self, inverted: bool = False) -> Twist:
        """
        Returns a twist from the current pose.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0

        # Return nothing if the alliance is incorrect.
        if self.__alliance not in (Alliance.RED, Alliance.BLUE):
            rospy.logerr("Auto-balance has no valid alliance color selected.")
            return twist

        # Determine if the twist needs to be inverted.
        invert: float = 1.0
        if self.__alliance == Alliance.BLUE:
            invert *= -1.0
        if self.__desired_robot_direction == RobotDirection.BACK:
            invert *= -1.0
        if inverted:
            invert *= -1.0

        # Calculate the twist.
        if self.__balance_direction == BalanceDirection.PITCH:
            twist.linear.x = -invert * 1.0
        elif self.__balance_direction == BalanceDirection.ROLL:
            twist.linear.y = -invert * 1.0

        return twist

    def __calaculate_distance_travelled(self) -> float:
        distance_travelled = 0.0
        if self.__balance_direction == BalanceDirection.PITCH:
            distance_travelled = abs(self.__current_pose.position.x - self.__initial_backup_pos)
        elif self.__balance_direction == BalanceDirection.ROLL:
            distance_travelled = abs(self.__current_pose.position.y - self.__initial_backup_pos)
        return distance_travelled
