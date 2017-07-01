from .utils import math_util


class Adapter(object):
    """
    In this project only view2model adapter
    is needed, since model directly 
    writes into view
    """

    _INQUIRY_DIC = dict(
        tool=['pose', 'v', 'omega',
              # TODO
              # 'force', 'wrench', 'shape',
              'name', 'contact'],
        body=['pose', 'v', 'omega',
              # 'force', 'wrench', 'shape',
              'name', 'tid',
              'contact'],
        env=['gravity', 'traction', ]
    )

    def __init__(self, model):

        self._world = model

        # space to store useful states
        self._states = \
            dict(tool=dict(),
                 # The log of commands
                 # or user operations/modifications
                 # on the world
                 log=list())

    @property
    def states(self):
        """
        Get internal state of this adapter
        :return: dictionary of states
        """
        return self._states

    @states.setter
    def states(self, states):
        """
        Store necessary input states for interruption 
        handling. Provided as an adapter socket to 
        communicate with this handler.
        :return: None
        """
        for tid, init_pose in states.items():
            self._states['tool'][tid] = init_pose

    def reset(self):
        """
        Reset the adapter states
        :return: None
        """
        # Special case for keyboard control again on Model side
        # setup initial states for relative comp
        self.states = self.get_world_states(
            ('tool', 'pose'))[0]

    def update_world(self, update_info):
        pass

    def set_world_states(self, name=('', '')):
        pass

    def get_world_states(self, *args):
        """
        Get world states by attribute name
        :param args: list of string tuples indicating what
        kind of inquiry (key), and what attributes 
        to get (value).
        :return: states dictionary of given 
        inquiry. Keys are ids, values are states.
        For example, [('body', 'pose')]
        It will return a list of dictionaries in the same 
        order as inquiry list.
        """
        state_list = list()
        for key, value in args:
            assert key in self._INQUIRY_DIC, \
                'Invalid inquiry key \'%s\'' % key
            assert value in self._INQUIRY_DIC[key], \
                'Invalid inquiry value \'%s\'' % value
            if key == 'env':
                # Hope this is not evil as setattr!
                info = getattr(self._world, value)
            else:
                prop = getattr(self._world, key)
                info = dict((x, getattr(prop[x], value)) for x in prop)
            state_list.append(info)
        return state_list

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

            # First check if there's low level commands
            # TODO: Think if cmd/ins needs to be a class
            # A sequential list of commands to execute in order
            # These low level commands are set to absolute
            # values in all cases
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

                # Note this reset does not reset the elapsed
                # run time. The user is forced to finish the
                # task in limited amount of time.
                if method == 'rst':
                    self._world.reset()
                    # Update the states again
                    self.reset()
                    # TODO: add reset
                elif method == 'reach':
                    r_pos, a_orn = value
                    i_pos, _ = self._states['tool'][tool.tid]

                    # Orientation is always relative to the
                    # world frame, that is, absolute
                    # if a_orn is None:
                    #     a_orn = tool.orn

                    # TODO: orn may need clipping
                    r = math_util.quat2mat(tool.orn)
                    # print(math_util.quat2euler(tool.tool_orn),
                    #       math_util.quat2euler(tool.orn),
                    #       math_util.quat2euler((0,0,0.707,0.707)))
                    # TODO: do we need to use orn? Yes for robot
                    d_pos = r.dot(r_pos)
                    # d_pos = r_pos

                    # Increment to get absolute pos
                    i_pos += d_pos

                    # TODO: consider losing control, too far away
                    # TODO: do we need thresholding?
                    tool.reach(i_pos, a_orn)
                elif method == 'grasp':
                    tool.grasp(value)
                elif method == 'pick_and_place':
                    tool.pick_and_place(*value)

