from abc import ABC, abstractmethod
from typing import List
from actions_node.game_specific_actions.Subsystem import Subsystem
import json

class Action(ABC):
    @abstractmethod
    def isFinished(self) -> bool:
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def done(self):
        pass

    @abstractmethod
    def affectedSystems(self) -> List[Subsystem]:
        pass 

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)

    def __repr__(self):
        return self.__str__()

    def to_json(self):
        return self.__str__()
    
    #TODO: not sure this works
    @staticmethod
    @abstractmethod
    def from_json(json_dct):
        pass