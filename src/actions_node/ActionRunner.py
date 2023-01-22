#!/usr/bin/env python3

import rospy
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotMode
from actions_node.default_actions.SeriesAction import SeriesAction

class ActionRunner:
    def __init__(self) -> None:
        self.__active_action:SeriesAction = None

    def start_action(self, action:SeriesAction):
        self.__active_action = action
        if self.__active_action is not None:
            try:
                self.__active_action.start()
            except:
                rospy.logerr("Exception encountered starting action")
                self.__active_action = None

    def loop(self, robot_mode : RobotMode):
        if robot_mode != RobotMode.DISABLED:
            if self.__active_action is not None:
                if self.__active_action.isFinished():
                    self.__active_action.done()
                    self.__active_action = None
                else:
                    self.__active_action.update()
        else:
            self.__active_action = None