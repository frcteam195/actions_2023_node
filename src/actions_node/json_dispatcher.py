from actions_node.default_actions.Action import Action
from actions_node.game_specific_actions.HighConeAction import HighConeAction
from actions_node.game_specific_actions.MoveArmAction import MoveArmAction
from actions_node.default_actions.ParallelAction import ParallelAction
from actions_node.default_actions.SeriesAction import SeriesAction

#TODO Generate this file automatically

def get_action(k : str, v) -> Action:
    if (k == 'HighConeAction'):
        return HighConeAction.from_json(v)
    elif (k == 'MoveArmAction'):
        return MoveArmAction.from_json(v)
    elif (k == 'ParallelAction'):
        return ParallelAction.from_json(v)
    elif (k == 'SeriesAction'):
        return SeriesAction.from_json(v)
