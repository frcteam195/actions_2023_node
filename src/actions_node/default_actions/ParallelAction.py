import rospy
from actions_node.default_actions.Action import Action
from datetime import datetime
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem

class ParallelAction(Action):
    def __init__(self, action_list:List[Action]):
        self.__action_list:List[Action] = action_list

        for a in self.__action_list[:]:
            if a is None:
                rospy.logerr("Invalid action added to list")
                self.__action_list.remove(a)
        
    def start(self):
        for a in self.__action_list:
            a.start()

    def update(self):
        for a in self.__action_list:
            a.update()

    def done(self):
        for a in self.__action_list:
            a.done()

    def isFinished(self) -> bool:
        for a in self.__action_list:
            if not a.isFinished():
                return False

        return True

    def affectedSystems(self) -> List[Subsystem]:
        retlist = []
        for a in self.__action_list:
            retlist.extend(a.affectedSystems())
        return retlist