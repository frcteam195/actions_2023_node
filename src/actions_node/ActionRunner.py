#!/usr/bin/env python3

import rospy
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotMode
from actions_node.default_actions.SeriesAction import SeriesAction
from actions_node.default_actions.Action import Action

class ActionRunner:
    def __init__(self) -> None:
        self.__active_action_list:list[Action] = []

    def start_action(self, new_action:Action):
        try:
            #TODO test timing and maybe come up with a more efficient solution
            self.__active_action_list[:] = list(filter(lambda a: not any(s in a.affectedSystems() for s in new_action.affectedSystems()), self.__active_action_list))
            self.__active_action_list.append(new_action)
            new_action.start() 
        except:
            rospy.logerr("Exception encountered starting action")

    def loop(self, robot_mode : RobotMode):
        num_actions = len(self.__active_action_list)
        if robot_mode != RobotMode.DISABLED:
            if num_actions > 0:
                if any(a.isFinished() for a in self.__active_action_list):
                    for a in self.__active_action_list:
                        if a.isFinished():
                            a.done()
                    
                    self.__active_action_list[:] = list(filter(lambda a: not a.isFinished(), self.__active_action_list))

                for a in self.__active_action_list:
                    a.update()
        else:
            if num_actions > 0:
                self.__active_action_list = []