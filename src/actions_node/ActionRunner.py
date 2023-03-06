#!/usr/bin/env python3

import rospy
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotMode
from actions_node.default_actions.Action import Action
from actions_node.game_specific_actions.Subsystem import Subsystem
from typing import List

class ActionRunner:
    def __init__(self) -> None:
        self.__active_action_list:List[Action] = []

    def get_operated_systems(self) -> List[Subsystem]:
        b = []
        for a in self.__active_action_list:
            for s in a.affectedSystems():
                b.append(s)
        return b
    
    def reset_action_list(self):
        self.__active_action_list:List[Action] = []


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
                ############DEBUG
                for a in self.__active_action_list:
                    rospy.loginfo_throttle_identical(1, f"Active Action: {str(a)}")
                #################

                if any(a.isFinished() for a in self.__active_action_list):
                    for a in self.__active_action_list:
                        if a.isFinished():
                            ############DEBUG
                            rospy.loginfo_throttle_identical(1, f"Finished Action: {str(a)}")
                            #################
                            a.done()

                    self.__active_action_list[:] = list(filter(lambda a: not a.isFinished(), self.__active_action_list))

                for a in self.__active_action_list:
                    a.update()
        else:
            if num_actions > 0:
                self.__active_action_list = []
