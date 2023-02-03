from actions_node.default_actions.Action import Action
from actions_node.game_specific_actions.HighConeAction import HighConeAction
from actions_node.game_specific_actions.MoveArmAction import MoveArmAction

#TODO Generate this file automatically

def get_action(json_dct : dict) -> Action:
    if ('HighConeAction' in json_dct.keys()):
        return HighConeAction.from_json(json_dct)
    elif ('MoveArmAction' in json_dct.keys()):
        return MoveArmAction.from_json(json_dct)