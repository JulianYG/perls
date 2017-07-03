from .utils import math_util
from .utils.io_util import (FONT,
                            logerr,
                            loginfo)

class Adapter(object):
    """
    In this project only view2model adapter
    is needed, since model directly 
    writes into view
    """

    # Storing inquiry legend
    _INQUIRY_DIC = dict(
        tool=['pose', 'v', 'omega',
              # TODO
              # 'force', 'wrench', 'shape',
              'name', 'contact'],
        body=['pose', 'v', 'omega',
              # 'force', 'wrench', 'shape',
              'name', 'tid',
              'contact'],
        env=['gravity', 'traction', 'target']
    )

    def __init__(self, model):
        """
        Initialize the adapter with given model
        :param model: the world
        """
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

    def update_states(self):
        """
        Update the adapter states
        :return: None
        """
        # Special case for keyboard control again on Model side
        # setup initial states for relative comp
        self.states = self.get_world_states(
            ('tool', 'pose'))[0]

    def check_world_states(self):
        """
        Check the world states to see if the task
        is completed.
        :return: done, success
        """
        return False, 0

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
                    loginfo('Unrecognized command type. Skipped',
                            FONT.ignore)

            # Next perform high level instructions
            for ins in instructions:
                # Use None for no value
                method, value = ins

                # Note this reset does not reset the elapsed
                # run time. The user is forced to finish the
                # task in limited amount of time.
                if method == 'rst':
                    loginfo('Resetting...', FONT.model)
                    self._world.reset()

                    # Update the states again
                    self.update_states()
                    loginfo('World is reset.', FONT.model)
                elif method == 'reach':
                    # Cartesian, quaternion
                    r_pos, a_orn = value
                    i_pos, _ = self._states['tool'][tool.tid]

                    # Orientation is always relative to the
                    # world frame, that is, absolute
                    r = math_util.quat2mat(tool.orn)

                    if r_pos is not None:
                        # Increment to get absolute pos
                        # Take account of rotation
                        i_pos += r.dot(r_pos)
                        pos_diff, orn_diff = tool.reach(i_pos, None)
                    else:
                        ###
                        # Note: clipping does not happen here because
                        # arm and gripper are treated in the same way.
                        # Clipping would result in gripper not able to
                        # change orn. However, clipping here would give
                        # arm perfect response. Currently the arm end
                        # effector will switch position when reaching its
                        # limit. This is trade-off, sadly.
                        pos_diff, orn_diff = tool.reach(None, a_orn)

                    # Update state orientation all the time
                    # Note by intuition, position should be updated too.
                    # However, due to the limitation of IK, this will
                    # cause robot arm act weirdly.
                    state_pose = self._states['tool'][tool.tid]
                    self._states['tool'][tool.tid] = \
                        (state_pose[0], tool.tool_orn)

                    # If the tool is out of reach, update the adapter states
                    # TODO: make the threshold configs
                    # if math_util.rms(pos_diff) > 1.5 or \
                    #    math_util.rms(orn_diff) > 5.0:
                    #     self.update_states()

                elif method == 'grasp':
                    tool.grasp(value)
                elif method == 'pick_and_place':
                    tool.pick_and_place(*value)
