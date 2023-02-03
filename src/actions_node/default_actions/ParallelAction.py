from actions_node.default_actions.Action import Action
from datetime import datetime
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem
from actions_node.json_dispatcher import get_action

class ParallelAction(Action):
    def __init__(self, action_list:List[Action]):
        self.__current_action_index:int = -1
        self.__action_list:List[Action] = action_list

        for a in self.__action_list[:]:
            if a is None:
                print("Invalid action added to list")
                self.__action_list.remove(a)
        
        self.__current_action:Action = None
        pass

    def start(self):
        self.__current_action = None
        self.__current_action_index = 0

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
    
    @staticmethod
    def from_json(json_dct : dict):
        ret_list = []
        for k,v in json_dct:
            ret_list.append(get_action(k, v))
        return ParallelAction(ret_list)
