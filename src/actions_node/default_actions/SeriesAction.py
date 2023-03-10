from actions_node.default_actions.Action import Action
from datetime import datetime
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem
import rospy

class SeriesAction(Action):
    def __init__(self, action_list:List[Action]):
        self.__current_action_index:int = -1
        self.__action_list:List[Action] = action_list

        for a in self.__action_list[:]:
            if a is None:
                rospy.logerr("Invalid action added to list")
                self.__action_list.remove(a)

        self.__current_action:Action = None
        pass

    def start(self):
        self.__current_action = None
        self.__current_action_index = 0

    def update(self):
        if self.__current_action == None:
            if self.__current_action_index >= len(self.__action_list):
                return

            self.__current_action = self.__action_list[self.__current_action_index]
            self.__current_action_index += 1
            self.__current_action.start()

        self.__current_action.update()

        if self.__current_action.isFinished():
            self.__current_action.done()
            self.__current_action = None

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__current_action == None and self.__current_action_index == len(self.__action_list)

    def affectedSystems(self) -> List[Subsystem]:
        retlist = []
        for a in self.__action_list:
            retlist.extend(a.affectedSystems())
        return retlist
    
    def __str__(self) -> str:
        return f"Series: {str(self.__current_action)}"