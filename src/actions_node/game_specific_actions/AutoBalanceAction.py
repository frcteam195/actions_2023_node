import rospy
from actions_node.default_actions.Action import Action
from typing import List
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from actions_node.game_specific_actions.Subsystem import Subsystem
from swerve_trajectory_node.srv import StopTrajectory, StopTrajectoryResponse
from ck_ros_msgs_node.msg import Trajectory_Status, Swerve_Drivetrain_Auto_Control
from ck_ros_base_msgs_node.msg import IMU_Data, IMU_Sensor_Data
from ck_utilities_py_node.geometry import *
from enum import Enum
from ck_utilities_py_node.pid_controller import PIDController

class BalanceDirection(Enum):
    PITCH = 0
    ROLL = 1


class AutoBalanceAction(Action):
    """An action that commands a twist to the swerve drivetrain in order to balance on the charge station"""

    def __init__(self, balance_direction : BalanceDirection, balance_threshold_deg : float):
        self.__traj_status_subscriber = BufferedROSMsgHandlerPy(Trajectory_Status)
        self.__traj_status_subscriber.register_for_updates("/TrajectoryStatus")
        self.__imu_subscriber = BufferedROSMsgHandlerPy(IMU_Data)
        self.__imu_subscriber.register_for_updates("/IMUData")
        self.__balance_direction = balance_direction
        self.__balance_threshold = math.radians(balance_threshold_deg)
        self.__balance_pid = PIDController(kP = 0.005, kD=0.0008)
        self.__drive_twist_publisher = rospy.Publisher(name="/SwerveAutoControl", data_class=Swerve_Drivetrain_Auto_Control, queue_size=10, tcp_nodelay=True)


    def start(self):
        rospy.wait_for_service('/stop_trajectory')
        stop_traj = rospy.ServiceProxy('/stop_trajectory', StopTrajectory)
        stop_traj_response : StopTrajectoryResponse = stop_traj()
        if not stop_traj_response.accepted:
 
            rospy.logerr(f"Failed to stop trajectory")


    def update(self):
        traj_status : Trajectory_Status = self.__traj_status_subscriber.get()
        if traj_status is not None:
            #Maybe check percentage complete of current traj before stopping it
            pass

        imu_data : IMU_Data = self.__imu_subscriber.get()
        if imu_data is not None:
            if len(imu_data.imuData) > 0:
                imu_sensor_data : IMU_Sensor_Data = imu_data.imuData[0]
                process_var : float = self.__determine_process_var(imu_sensor_data)
            
                control_msg : Swerve_Drivetrain_Auto_Control = Swerve_Drivetrain_Auto_Control()

                if self.__balance_direction == BalanceDirection.PITCH:
                    control_msg.twist.linear.x = self.__balance_pid.update(0, process_var)
                    control_msg.twist.linear.y = 0
                elif self.__balance_direction == BalanceDirection.ROLL:
                    control_msg.twist.linear.x = 0
                    control_msg.twist.linear.y = self.__balance_pid.update(0, process_var)

                self.__drive_twist_publisher.publish(control_msg)



    def done(self):
        self.__drive_twist_publisher.publish(Swerve_Drivetrain_Auto_Control())


    def isFinished(self) -> bool:
        imu_data : IMU_Data = self.__imu_subscriber.get()
        if imu_data is not None:
            if len(imu_data.imuData) > 0:
                imu_sensor_data : IMU_Sensor_Data = imu_data.imuData[0]
                process_var : float = self.__determine_process_var(imu_sensor_data)
                return within(process_var, 0, self.__balance_threshold)
        return False


    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.DRIVEBASE ]
    

    def __determine_process_var(self, imu_sensor_data : IMU_Sensor_Data) -> float:
        process_var : float = 0
        if self.__balance_direction == BalanceDirection.PITCH:
            process_var = imu_sensor_data.pitch
        elif self.__balance_direction == BalanceDirection.ROLL:
            process_var = imu_sensor_data.roll
        return process_var
