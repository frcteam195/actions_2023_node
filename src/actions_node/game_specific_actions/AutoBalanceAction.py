import rospy
import math
from actions_node.default_actions.Action import Action
from typing import List
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from actions_node.game_specific_actions.Subsystem import Subsystem
from swerve_trajectory_node.srv import StopTrajectory, StopTrajectoryResponse
from ck_ros_msgs_node.msg import Trajectory_Status, Swerve_Drivetrain_Auto_Control
from nav_msgs.msg import Odometry
from ck_utilities_py_node.geometry import *
from enum import Enum
from ck_utilities_py_node.pid_controller import PIDController
from ck_utilities_py_node.moving_average import MovingAverage


class BalanceDirection(Enum):
    PITCH = 0
    ROLL = 1


class RobotDirection(Enum):
    FRONT = 0
    BACK = 1


class AutoBalanceAction(Action):
    """An action that commands a twist to the swerve drivetrain in order to balance on the charge station"""

    def __init__(self, balance_direction: BalanceDirection, balance_threshold_deg: float, desired_robot_direction: RobotDirection):
        rospy.logerr("Init auto balance action")
        self.__imu_subscriber = BufferedROSMsgHandlerPy(Odometry)
        self.__imu_subscriber.register_for_updates("/RobotIMU")
        self.__balance_direction = balance_direction
        self.__desired_heading = 0
        self.__pitch_rate_average = MovingAverage(10)
        alliance = robot_status.get_alliance()
        self.__alliance = alliance
        self.__desired_robot_direction = desired_robot_direction
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

    def start(self):
        rospy.logerr("Starting auto balance")
        rospy.wait_for_service('/stop_trajectory')
        stop_traj = rospy.ServiceProxy('/stop_trajectory', StopTrajectory)
        stop_traj_response: StopTrajectoryResponse = stop_traj()

        if not stop_traj_response.accepted:
            rospy.logerr("Failed to stop trajectory!")

    def update(self):
        imu_data: Odometry = self.__imu_subscriber.get()
        if imu_data is not None:
            imu_sensor_data: Pose = Pose(imu_data.pose.pose)
            process_var: float = self.__determine_process_var(imu_sensor_data.orientation)

            control_msg: Swerve_Drivetrain_Auto_Control = Swerve_Drivetrain_Auto_Control()
            control_msg.pose.orientation = self.__desired_quat
            control_msg.pose.position.x = 11
            yaw = normalize_to_2_pi(imu_sensor_data.orientation.yaw)
            yaw = np.degrees(yaw)
            if self.__balance_direction == BalanceDirection.PITCH:
                if self.__alliance == Alliance.RED:
                    if self.__desired_robot_direction == RobotDirection.FRONT:
                        control_msg.twist.linear.x = self.__balance_pid.update(0, process_var)
                    else:
                        control_msg.twist.linear.x = -self.__balance_pid.update(0, process_var)
                elif self.__alliance == Alliance.BLUE:
                    if self.__desired_robot_direction == RobotDirection.FRONT:
                        control_msg.twist.linear.x = -self.__balance_pid.update(0, process_var)
                    else:
                        control_msg.twist.linear.x = self.__balance_pid.update(0, process_var)
                else:
                    rospy.logerr("No valid alliance color selected")
                    control_msg.twist.linear.x = 0
                control_msg.twist.linear.y = 0
            elif self.__balance_direction == BalanceDirection.ROLL:
                if self.__alliance == Alliance.RED:
                    if self.__desired_robot_direction == RobotDirection.FRONT:
                        control_msg.twist.linear.y = self.__balance_pid.update(0, process_var)
                    else:
                        control_msg.twist.linear.y = -self.__balance_pid.update(0, process_var)
                elif self.__alliance == Alliance.BLUE:
                    if self.__desired_robot_direction == RobotDirection.FRONT:
                        control_msg.twist.linear.y = -self.__balance_pid.update(0, process_var)
                    else:
                        control_msg.twist.linear.y = self.__balance_pid.update(0, process_var)
                else:
                    rospy.logerr("No valid alliance color selected")
                    control_msg.twist.linear.y = 0
                control_msg.twist.linear.x = 0
            self.__drive_twist_publisher.publish(control_msg)


    def done(self):
        control_msg: Swerve_Drivetrain_Auto_Control = Swerve_Drivetrain_Auto_Control()
        control_msg.pose.orientation = self.__desired_quat
        self.__drive_twist_publisher.publish(control_msg)

    def isFinished(self) -> bool:
        imu_data: Odometry = self.__imu_subscriber.get()
        if imu_data is not None:
            angular_rates = Twist(imu_data.twist.twist).angular
            self.__pitch_rate_average.add_sample(angular_rates.pitch)
            return abs(self.__pitch_rate_average.get_average()) > math.radians(12.0)
        return False

    def affectedSystems(self) -> List[Subsystem]:
        return [Subsystem.DRIVEBASE]

    def __determine_process_var(self, imu_sensor_data: Rotation) -> float:
        process_var: float = 0
        if self.__balance_direction == BalanceDirection.PITCH:
            process_var = imu_sensor_data.pitch
        elif self.__balance_direction == BalanceDirection.ROLL:
            process_var = imu_sensor_data.roll
        return process_var
