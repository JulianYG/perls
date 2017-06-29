
class Adapter(object):
    """
    In this project only view2model adapter
    is needed, since model directly 
    writes into view
    """

    def __init__(self, model):

        self._world = model

    def update_world(self, update_info):
        pass

    def react(self, signal):
        """
        React only take care of motion related 
        control, that is manipulating a tool.
        :param signal: dictionary signal
        :return: None
        """
        commands, instructions = signal['cmd'], signal['instruction']
        if commands or instructions:
            tool = self._world.get_tool(signal['tid'], signal['key'])

            # First check if there's low level cmds
            # TODO: Think if cmd/ins needs to be a class
            # A sequential list of commands to execute in order
            for cmd in commands:
                method, value = cmd
                if method == 'pos':
                    tool.tool_pos = value
                elif method == 'orn':
                    tool.tool_orn = value
                elif method == 'joint_states':
                    tool.joint_states = value
                else:
                    print('Unrecognized command type. Skipped')

            # Next perform high level instructions
            for ins in instructions:
                # Use None for no value
                method, value = ins
                if method == 'reset':
                    tool.reset()
                elif method == 'reach':
                    pos, orn = value
                    # Use relative position and absolute orientation
                    tool.reach(tool.tool_pos + pos, orn)
                elif method == 'grasp':
                    tool.grasp(value)
                elif method == 'pick_and_place':
                    tool.pick_and_place(*value)

