#!/usr/bin/env python

from ..utils import math_util

__author__ = 'Julian Gao'
__email__ = 'julianyg@stanford.edu'
__license__ = 'private'
__version__ = '0.1'


class Body(object):
    """
    The lowest (base) level of things in the environment.
    Can be component of joint, link, or even robot.
    """
    def __init__(self,
                 engine, path,
                 pos=None,
                 orn=None,
                 fixed=False):
        """
        :param engine: The physics engine used to simulate
         this tool.
        :param path: body model path (urdf, sdf, xml, mjcf, etc.)
        :param pos: initial position of object
        :param orn: initial orientation of object
        :param fixed: if using a fixed base for the object. Not 
        enabled for SDF files 
        """
        pos = pos or (0., 0., 0.)
        orn = orn or (0., 0., 0., 1.)

        # Store original info for reset
        self._init_state = (pos, orn, fixed)
        self._engine = engine
        self._model_path = path
        self._text_markers = dict()
        self._fixed = fixed
        self._children = dict()      # No children constraints initially
        self._uid, self._links, self._joints = \
            self._engine.load_asset(path, pos, orn, fixed)

    ###
    # Basic info
    @property
    def pos(self):
        """
        Get base position of the body
        :return: numpy array [x, y, z] position of object
        """
        return self._engine.get_body_scene_position(self._uid)

    @property
    def orn(self):
        """
        Get base orientation of the body in quaternion form
        :return: vec4 numpy float array of orientation of body
        """
        return self._engine.get_body_scene_orientation(self._uid, type='quaternion')
    
    @property
    def pose(self):
        """
        An extra level of abstraction to get 
        position and orientation.
        :return: (pos, orn) tuple
        """
        return self.pos, self.orn
    
    @property
    def uid(self):
        """
        Get id of body
        :return: unique object ID of this instance
        """
        return self._uid

    @property
    def cid(self):
        """
        Get constraints of body
        :return: list of current constraint IDs associated with this body
        """
        return [self._children[k]['cid'] for k in self._children.keys()]

    @property
    def fix(self):
        """
        Check if this instance use fixed base
        :return: fixed base or not
        """
        return self._fixed

    @property
    def name(self):
        """
        Get the name of the body
        :return: name string 
        """
        return self._engine.get_body_name(self._uid)

    @property
    def v(self):
        """
        Get linear velocity of body base
        :return: Numpy array of 3 Cartesian
        """
        return self._engine.get_body_linear_velocity(self._uid)

    @property
    def omega(self):
        """
        Get angular velocity of body base
        :return: Numpy array of 3 Cartesian
        """
        return self._engine.get_body_angular_velocity(self._uid)

    ###
    # Link related information
    @property
    def kinematics(self):
        """
        Get kinematics info of body instance.
        Note only kinematics does not allow -1 for base link,
        use 0 instead.
        :return: a dictionary of lists of
        {name, pos, orn, rel_pos, rel_orn, abs_frame_pos, 
        abs_frame_orn, abs_v, abs_omega}, 
        where the keys are info names, and 
        values are arranged in order of link indices.
        """
        info_dic = dict(
            name=[],
            pos=[],
            orn=[],
            rel_pos=[],
            rel_orn=[],
            abs_frame_pos=[],
            abs_frame_orn=[],
            abs_v=[],
            abs_omega=[]
        )
        if len(self._links) == 1:
            info_dic['name'].append(self._engine.get_link_name(self._uid, -1))
            info_dic['pos'].append(self.pos)
            info_dic['orn'].append(self.orn)
            info_dic['rel_pos'].append([0., 0., 0.])
            info_dic['rel_orn'].append([0., 0., 0., 1.])
            info_dic['abs_frame_pos'].append(self.pos)
            info_dic['abs_frame_orn'].append(self.orn)
            info_dic['abs_v'].append(self.v)
            info_dic['abs_omega'].append(self.omega)
        else:
            for lid in self._links[1:]:
                info = self._engine.get_body_link_state(self._uid, lid)
                info_dic['name'].append(self._engine.get_link_name(self._uid, lid))
                info_dic['pos'].append(info[0])
                info_dic['orn'].append(info[1])
                info_dic['rel_pos'].append(info[2])
                info_dic['rel_orn'].append(info[3])
                info_dic['abs_frame_pos'].append(info[4])
                info_dic['abs_frame_orn'].append(info[5])
                info_dic['abs_v'].append(info[6])
                info_dic['abs_omega'].append(info[7])
        return info_dic

    @property
    def dynamics(self):
        """
        Get dynamics info of entire body
        :return: a dictionary of dictionaries of mass (kg), 
        friction coefficient, etc., where the keys 
        are the link indices.
        """
        info_dic = {}
        for lid in self._links:
            info = self._engine.get_body_dynamics(self._uid, lid)
            info_dic[lid] = dict(mass=info[0],
                                 lateral_friction=info[1])
        return info_dic

    ###
    # Joint related information
    @property
    def joint_specs(self):
        """
        Get joints info of the body.
        :return: a dictionary of lists of
        {name, joint type, pos_idx, vel_idx, damping,
        friction, lower, upper, max_force, max_vel},
        where the keys are info names, and
        values are arranged in link indices order.
        """
        info_dict = dict(
            name=[],
            jtype=[],
            pos_idx=[],
            vel_idx=[],
            damping=[],
            friction=[],
            lower=[],
            upper=[],
            max_force=[],
            max_vel=[]
        )
        for jid in self._joints:
            info = self._engine.get_body_joint_info(self._uid, jid)
            info_dict['name'].append(info[1])
            info_dict['jtype'].append(info[2])
            info_dict['pos_idx'].append(info[3])
            info_dict['vel_idx'].append(info[4])
            info_dict['damping'].append(info[6])
            info_dict['friction'].append(info[7])
            info_dict['lower'].append(info[8])
            info_dict['upper'].append(info[9])
            info_dict['max_force'].append(info[10])
            info_dict['max_vel'].append(info[11])
        return info_dict

    @property
    def joint_states(self):
        """
        Get joint states of the body
        :return: a dictionary of dictionaries of
        {joint_index: {pos, v, wrench (vec6), motor_torque}}
        where motor_torque is the applied motor torque
        during last simulation step.
        Note that force and motor_torque are only
        available for non real time simulation.
        """
        info_dict = {}
        for jid in self._joints:
            info = self._engine.get_body_joint_state(self._uid, jid)
            info_dict[jid] = dict(
                pos=info[0],
                v=info[1],
                wrench=list(info[2]),
                motor_torque=info[3]
            )
        return info_dict

    ###
    # Relation info with other bodies in space
    @property
    def aabb(self):
        """
        Get the axis-aligned-bounding-box of the body base
        :return: a list of vec3, 
        [(x_min, y_min, z_min), (x_max, y_max, z_max)] in the world
        """
        return self._engine.get_body_bounding_box(self._uid, -1)

    @property
    def contact(self):
        """
        Get the contact points from bodies B on this body A
        :return: a list of lists of dictionaries
        [
            base: 
                [{uid, lid, posA, posB, 
                normalB->A, distance, normal force applied}, 
                {}...],
            link_0:
                [{}, {}...],
            ...
        ]
        """
        return [self._engine.get_body_contact_info(self._uid, lid)
                for lid in self._links]

    @property
    def attach_children(self):
        """
        Get info of attach_children children bodies B's on this parent body A
        :return: a list of attach_children bodies dict of dicts
        The key of the big dict is child body uid.
        [{uid_B0: cid_0, j_0 A, j_0 B, type, jAxis, jPivotA, jPivotB, jOrnA,
            jOrnB, maxForce}, 
         {uid_B1: cid_1, j_1 A, ...}, 
         ...]
        """
        return self._children

    @property
    def attach_parent(self):
        """
        Get the parent body B this child body A is attach_parent to
        :return: Parent B
        """
        return self._engine.get_body_attachment(self._uid)

    ###
    # Visual information
    @property
    def visual_shape(self):
        """
        Get the visual shape information. Can be used to
        bridge your own rendering method with simulation, 
        and synchronize the world transforms manually 
        after each simulation step
        :return: A list of visual shape data:
        [uid, linkIndex, visualGeometryType, dimensions (vec3), 
        meshAssetFileName (str), localVisualFramePos (vec3), 
        localVisualFrameOrn (vec4), rgbaColor (vec4)]
        """
        return self._engine.get_body_visual_shape(self._uid)

    @property
    def collision_shape(self):
        # TODO
        return

    @property
    def mark(self):
        """
        Text mark getter. Get all marked texts info on this body
        :return: dictionary of dictionaries of info:
        [{id: {text_string, font_size, color, life_time}}, {}, ...]
        """
        return self._text_markers

    @fix.setter
    def fix(self, (pos, orn)):
        """
        Fix the object to given pose
        :param pos: position vec3 float cartesian to fix at
        :return: None
        """
        pos = self.pos if pos is None else pos
        orn = self.orn if orn is None else orn
        self.attach_children = (
            -1, -1, -1, 'fixed', [0.,0.,0.],
            [0.,0.,0.], pos, None, orn)
        self._fixed = True

    @fix.deleter
    def fix(self):
        """
        Unfix the item. Note that this is special because 
        for normal fix, this body A should be child, and 
        would be attach to parent -- the world. However, 
        pybullet wants body A as parent and world as 
        child (presumably this is relative). Need to treat 
        those cases separately.
        :return: None
        """
        if -1 in self._children:
            self._engine.remove_body_attachment(
                self._children[-1]['cid'])

    @pos.setter
    def pos(self, pos):
        """
        Set base position of the body base
        :param pos: object position in (x, y, z)
        :return: None
        """
        self._engine.set_body_scene_pose(self._uid, pos, self.orn)

    @orn.setter
    def orn(self, orn):
        """
        Set orientation of the body base
        :param orn: orientation in either quaternion or 
        euler form
        :return: None
        """
        self._engine.set_body_scene_pose(self._uid, self.pos, orn)

    @v.setter
    def v(self, velocity):
        """
        Set linear velocity of body base
        :param velocity: vec3 float Cartesian
        :return: None
        """
        if not self.fix:
            self._engine.set_body_linear_velocity(self._uid, velocity)
        else:
            print('Cannot set linear velocity for fixed body')

    @omega.setter
    def omega(self, velocity):
        """
        Set linear velocity of body base
        :param velocity: vec3 Cartesian
        :return: None
        """
        if not self.fix:
            self._engine.set_body_angular_velocity(self._uid, velocity)
        else:
            print('Cannot set angular velocity for fixed body')

    # Base level control functionality
    @joint_states.setter
    def joint_states(self, (jid, value, ctype, kwargs)):
        """
        Taken joint index(es), value(s), as well as control type
        (ctype) among position, velocity, and torque,
        set the control. The jid and value can be lists
        of the same length. kwargs is a dictionary
        :return: None
        """
        max_forces = tuple(self.joint_specs['max_force'][j] for j in jid)
        if kwargs:
            kwargs['forces'] = kwargs.get('forces', None) or max_forces
        else:
            kwargs = dict(forces=max_forces)
        self._engine.set_body_joint_state(self._uid, jid, value, ctype, kwargs)

    @dynamics.setter
    def dynamics(self, info):
        """
        Set dynamics of body based on given info
        :param info: a dictionary of dictionary that may or 
        may not contain following keys:
        {link_index: 
            {mass, lateral_friction, spinning_friction, rolling_friction,
            restitution (0-1)},
            ...
        }
        :return: None
        """
        for link_id, d_info in info.items():
            self._engine.set_body_dynamics(self._uid, link_id, d_info)

    @attach_children.setter
    def attach_children(self,
                        (lid, child_uid, child_lid, jtype, jaxis,
                         parent_pos, child_pos, parent_orn, child_orn)):
        """
        Attach a child body B on this parent body A
        Required input:
        an info list:
        [j_0 A, j_0 B, type, jAxis, jPivotA, jPivotB, jOrnA, jOrnB]
        :return: None
        """
        # TODO: add if/else for changing constraint
        if parent_orn is None:
            parent_orn = [0., 0., 0., 1.]
        if child_orn is None:
            child_orn = [0., 0, 0., 1.]

        cid = self._engine.set_body_attachment(
            self._uid, lid, child_uid, child_lid,
            jtype, jaxis, parent_pos, child_pos,
            parentFrameOrientation=parent_orn,
            childFrameOrientation=child_orn)

        self._children[child_uid] = dict(
            cid=cid, parentJointIdx=lid,
            childLinkIndex=child_lid,
            type=jtype, jointAxis=jaxis,
            parentJointPvt=parent_pos,
            childJointPvt=child_pos,
            parentJointOrn=parent_orn,
            childJointOrn=child_orn)

    @attach_children.deleter
    def attach_children(self):
        """
        Remove all children body constraints 
        on this body.
        :return: None
        """
        for child in self._children.keys():
            # Exclude world frame
            if child != -1:
                self._engine.remove_body_attachment(
                    self._children[child]['cid'])
                del self._children[child]

    @visual_shape.setter
    def visual_shape(self, (qid, sid, tid, rgba, spec)):
        """
        Reset visual shape data to change the texture of a shape. 
        Currently only affects the software renderer (getCameraImage), 
        does not show up on OpenGL window
        :param qid: joint index
        :param sid: shape index ()
        :param tid: texture unique index
        :param rgba: vec4 in range [0, 1]. No transparent alpha yet
        :param spec: RGB 0-100 vec3
        :return: None
        """
        self._engine.set_body_visual_shape(self._uid, qid, sid, tid, rgba, spec)

    @collision_shape.setter
    def collision_shape(self, *args):
        # TODO
        pass
        
    @mark.setter
    def mark(self, (text, font_size, color, lid, time)):
        """
        Add marker text to this body.
        ### Note: 
        Use None for lid for base link!
        :param text: text string to display
        :param font_size: float scalar of text size.
        By default is 2.5
        :param color: RGB color, vec3 float in [0,1]. 
        By default is red
        :param time: float time to display, 0 for permanent.
        By default is 10 seconds
        :param lid: link id to display on this body
        :return: None
        """
        # Need a few tricks to hack link index for base

        mid = self._engine.add_body_text_marker(
            text, self.kinematics['pos'][lid or 0],
            font_size, color, self.uid, lid or -1, time
        )
        self._text_markers[mid] = dict(
            text=text, size=font_size, color=color, time=time)

    @mark.deleter
    def mark(self):
        """
        Delete all markers on this body
        :return: None
        """
        for mid in self._text_markers.keys():
            self._engine.remove_body_text_marker(mid)
        self._text_markers = dict()

    def reset(self):
        """
        Reset body to its original pose/fix condition.
        Note currently this cannot remove body attachments
        :return: None
        """
        del self.fix
        pos, orn, fixed = self._init_state
        if not fixed:
            self._engine.set_body_scene_pose(self._uid, pos, orn)
        else:
            self.fix = (pos, orn)

    def remove(self):
        """
        Remove current object instance from environment
        :return: None
        """
        self._engine.delete_body(self._uid)

    def get_neighbors(self, uidB, dist_thresh, lidB=-1):
        """
        Get the closest points of bodies B near this body A,
        within given distance threshold
        :return: a list of lists of dictionaries
        [
            base: [{uid, lid, posA, posB, distance}, {}...],
            link_0: [{}, {}...],
            ...
        ]
        """
        return [self._engine.get_body_surroundings(
            self._uid, lid,
            uidB, lidB, dist_thresh) for lid in self._links]
        
    def apply_force(self, force, pos, ref='abs', lid=-1):
        """
        Apply a force to body. Only available when explicitly 
         stepping the simulation
        :param lid: link index, -1 for base
        :param force: Vec3 
        :param pos: Vec3
        :param ref: string 'abs' for world frame,
        'rel' for link frame
        :return: -1 if failure, 0 if success
        """
        if lid not in self._links:
            print('Invalid link index to apply force on.')
            return -1
        else:
            self._engine.apply_force_to_body(
                self._uid, lid, force, pos, ref)
            return 0

    def apply_torque(self, torque, pos, ref='abs', lid=-1):
        """
        Apply a torque to body. Only available when explicitly 
        stepping the simulation
        :param lid: link index, -1 for base
        :param torque: Vec3 
        :param pos: Vec3
        :param ref: string 'abs' for world frame,
            'rel' for link frame
        :return: -1 if failure, 0 if success
        """
        if lid not in self._links:
            print('Invalid link index to apply force on.')
            return -1
        else:
            self._engine.apply_torque_to_body(
                self._uid, lid, torque, pos, ref)
            return 0

    def track(self, pos, orn, max_force):
        """
        Follow the given pos/orn with given max force
        with the base link
        :return: None
        """
        # Cannot move if fixed already
        if self.fix:
            print('Fix-based body cannot track.')
            return
        # Need to constrain to world frame first
        if -1 not in self.attach_children:
            self.fix = (pos, orn)
        else:
            cid = self.attach_children[-1]['cid']
            pos = self.pos if pos is None else pos
            orn = self.orn if orn is None else orn
            self._engine.move_body(cid, pos, orn, max_force)


class Tool(Body):
    """
    The abstracted level of tools in the world. Can be
    a gripper, a robot arm, a robot, or even hand.
    """
    def __init__(self, t_id, engine, path, pos, orn, fixed):
        """
        Initialize the tool.
        :param t_id: A different id for tool other than its
        inherited body uid, for control purpose. The id
        is in the form of '%s%d', where s is 'm' for arm,
        'b' for robot, 'g' for gripper, and 'h' for hand
        (some haptic device).
        :param engine: The physics engine used to simulate
         this tool.
        :param path: the asset model file path of this tool.
        :param pos: initial position vec3 float cartesian
        :param orn: initial orientation vec4 float quaternion
        :param fixed: indicating whether the tool is fixed,
        default False for gripper, True for robot arm.
        """
        self._tool_id = t_id
        super(Tool, self).__init__(engine, path, pos, orn, fixed)

        # The ultimate purpose for tool is to manipulate objects,
        # so close grip is universal for all tools
        self._close_grip = False
        self._dof = len(self._joints)
        self._tip_offset = math_util.zero_vec(3)

    @property
    def tid(self):
        """
        A tool id specifically assigned to this tool.
        Used for control.
        :return: integer tool id
        """
        return 't{}'.format(self._tool_id)

    @property
    def tool_pos(self):
        """
        Get the position of the tool. This is semantic
        :return: vec3 float in Cartesian
        """
        raise NotImplementedError('Method <tool_pos> not implemented for tool')

    @property
    def tool_orn(self):
        """
        Get the orientation of the tool. This is semantic
        :return: vec4 float quaternion in Cartesian
        """
        raise NotImplementedError('Method <tool_orn> not implemented for tool')

    ###
    #  Low level control functionality

    @tool_pos.setter
    def tool_pos(self, pos):
        """
        Set the tool to given position
        :param pos: vec3 float in cartesian space
        :return: None
        """
        raise NotImplementedError('Method <tool_pos> not implemented for tool')

    @tool_orn.setter
    def tool_orn(self, orn):
        """
        Set the tool to given orientation
        :param orn: vec4 float in quaternion form
        :return: None
        """
        raise NotImplementedError('Method <tool_orn> not implemented for tool')

    ###
    #  High level control functionality

    def reach(self, pos, orn):
        """
        Reach given position and orientation.
        Note that this method does not perform 
        position transform, and sets the position and 
        orientation individually.
        To achieve high accuracy finger-tip manipulation,
        use <pinpoint> method instead.
        :param pos: vec3 float in cartesian
        :param orn: vec4 float quaternion
        :return: Delta difference between target and actual (pos, orn)
        """
        raise NotImplementedError('Method <reach> not implemented for tool. '
                                  'Method is tool-specific')

    def pinpoint(self, pos, orn):
        """
        Accurately reach to given position and orientation.
        :param pos: vec3 float in cartesian
        :param orn: vec4 float quaternion
        :return: None
        """
        raise NotImplementedError('Method <pinpoint> not implemented for tool. '
                                  'Method is tool-specific')

    def grasp(self, slide):
        """
        Perform gripper close/release based on current state
        :param slide: if given, perform slider grasp
        :return: None
        """
        raise NotImplementedError('Method <grasp> not implemented for tool')

    def pick_and_place(self, pick_pos, place_pos,
                       pick_orn=None, place_orn=None,
                       **kwargs):
        """
        Pick on given positions and place it elsewhere
        :param pick_pos: vec3 float cartesian
        :param place_pos: vec3 float cartesian
        :param pick_orn: vec4 float quaternion
        :param place_orn: vec4 float quaternion
        :param kwargs: User can specify reaching/leaving
        behavior with five stages 'hover', 'attend', 
        'carry', 'release', and 'leave'. 
        The values are designated pos, orn
        This is especially
        useful on real robot case to achieve higher 
        success rate.
        :return: None
        """
        if kwargs['hover']:
            self.tool_pos, self.tool_orn = kwargs['hover']
        self.reach(pick_pos, pick_orn)
        if kwargs['attend']:
            self.tool_pos, self.tool_orn = kwargs['attend']
        self.grasp(None)
        if kwargs['carry']:
            self.tool_pos, self.tool_orn = kwargs['carry']
        self.reach(place_pos, place_orn)
        if kwargs['release']:
            self.tool_pos, self.tool_orn = kwargs['release']
        self.grasp(None)
        if kwargs['leave']:
            self.tool_pos, self.tool_orn = kwargs['leave']
