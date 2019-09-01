#! /usr/bin/python
import rospy
import numpy
import itertools
from planner_tests.srv import *
from geometry_msgs.msg import Pose2D


class ROSOracleBridge(object):
    """
        Python interface using ROS services to interact with the OracleTrainingServer.
    """

    def __init__(self, node_name='ros_oracle_client', server_name='oracle_training_server'):
        """
            Create a new ROSOracleBridge.
            ---------
            Arguments
            ---------
            node_name, string - name of this ROS node
            server_name, string - name of the server
        """
        rospy.init_node(node_name)
        rospy.loginfo("Waiting for services...")
        gasi_name = '/' + server_name + '/get_action_space_info'
        gop_name = '/' + server_name + '/get_object_properties'
        sop_name = '/' + server_name + '/set_object_properties'
        gs_name = '/' + server_name + '/get_state'
        prop_name = '/' + server_name + '/propagate'
        sao_name = '/' + server_name + '/set_active_objects'
        ss_name = '/' + server_name + '/set_state'
        ssp_name = '/' + server_name + '/set_static_pose'
        rospy.wait_for_service(gasi_name)
        rospy.wait_for_service(gop_name)
        rospy.wait_for_service(gs_name)
        rospy.wait_for_service(prop_name)
        rospy.wait_for_service(sao_name)
        rospy.wait_for_service(ss_name)
        rospy.wait_for_service(ssp_name)
        self._get_action_space_info_service = rospy.ServiceProxy(gasi_name, GetActionSpaceInfo)
        self._get_object_properties_service = rospy.ServiceProxy(gop_name, GetObjectProperties)
        self._set_object_properties_service = rospy.ServiceProxy(sop_name, SetObjectProperties)
        self._get_state_service = rospy.ServiceProxy(gs_name, GetState)
        self._propagate_service = rospy.ServiceProxy(prop_name, Propagate)
        self._set_active_objects_service = rospy.ServiceProxy(sao_name, SetActiveObjects)
        self._set_state_service = rospy.ServiceProxy(ss_name, SetState)
        self._set_static_pose_service = rospy.ServiceProxy(ssp_name, SetState)
        resp = self.get_object_properties()
        self._robot_name = 'robot'
        self._object_names = [name for name in resp.obj_names if name != self._robot_name]
        self._statics = {resp.obj_names[n] for n in range(len(resp.obj_names)) if resp.is_static[n]}
        movables = (set(self._object_names) - self._statics) | set([self._robot_name])
        self._ordered_movables = [m for m in resp.obj_names if m in movables]
        self._active_objects = set(self._ordered_movables)
        self._ordered_active_objects = [o for o in self._ordered_movables if o in self._active_objects]
        rospy.loginfo("Ready")

    def get_robot_name(self):
        """
            Return the name of robot.
        """
        return self._robot_name

    def get_action_space_info(self):
        """
            Return information on action space.
            ------
            Returns
            -------
            response, planner_tests.srv.GetActionSpaceInfoResponse

            The response has the following fields:
            dof_action_space, int - dimension of action space
            lower_bounds, array of floats
            upper_bounds, array of floats
            is_cyclic, array of bools - True if the action dimension is cyclic
        """
        return self._get_action_space_info_service()

    def get_statics(self):
        """
            Return a list containing all names of static obstacles.
        """
        return list(self._statics)

    def get_movables(self):
        """
            Return a list of all movable objects and the robot.
        """
        return list(self._ordered_movables)

    def get_object_properties(self):
        """
            Return object properties of objects in scene.
            -------
            Returns
            -------
            response, planner_tets.srv.GetObjectPropertiesResponse

            The response has the following fields:
            obj_names, array of string
            masses, array of floats
            inertias, array of floats
            ground_friction_coeffs, array of floats
            ground_friction_torque_integrals, array of floats
            contact_friction_coeffs, array of floats
            widths, array of floats
            heights, array of floats
            is_static, array of bools
        """
        response = self._get_object_properties_service()
        return response

    def set_object_properties(self, properties):
        """
            Set the given object properties.
            ---------
            Arguments
            ---------
            properties, dict: string -> dict - mapping from object name to a dictionary containing
                object properties for this object. Supported object properties are:
                mass, ground_friction_coeff, ground_friction_torque_integral, contact_friction_coeff

            -------
            Returns
            -------
            success, bool
        """
        req = SetObjectPropertiesRequest()
        for name, prop in properties.iteritems():
            req.obj_names.append(name)
            if 'mass' in prop:
                req.masses.append(prop['mass'])
            else:
                req.masses.append(-1.0)
            if 'ground_friction_coeff' in prop:
                req.ground_friction_coeffs.append(prop['ground_friction_coeff'])
            else:
                req.ground_friction_coeffs.append(-1.0)
            if 'ground_friction_torque_integral' in prop:
                req.ground_friction_torque_integrals.append(prop['ground_friction_torque_integral'])
            else:
                req.ground_friction_torque_integrals.append(-1.0)
            if 'contact_friction_coeff' in prop:
                req.contact_friction_coeffs.append(prop['contact_friction_coeff'])
            else:
                req.contact_friction_coeffs.append(-1.0)
        res = self._set_object_properties_service(req)
        return res.success

    def set_state(self, state, b_active_only=True):
        """
            Set the state of the world.
            ---------
            Arguments
            ---------
            state, numpy array of shape (n, 3) - every row is x, y, theta. First state is robot state.
            b_active_only - if True, set state for active objects only, i.e. n-1 is number of active objects,
                else set states for all (n-1 must be number of total objects)
            --------
            Returns
            --------
            valid_state, bool - True iff the set state is valid
        """
        request = SetStateRequest()
        if b_active_only:
            request.obj_names.extend(self._ordered_active_objects)
        else:
            request.obj_names = self._ordered_movables
        for obj_state in state:
            request.states.append(Pose2D(*obj_state))
        response = self._set_state_service(request)
        if not response.service_success:
            raise RuntimeError("Set state service failed.")
        return response.valid_state

    def get_state(self, b_active_only=True):
        """
            Return the current state of the world.
            ---------
            Arguments
            ---------
            b_active_only, bool - if True, only return state of active objects.
                First state is always robot state.
            -------
            Returns
            -------
            state, numpy array of shape (n, 3) -
                n-1 is the number of active objects
                first state is robot state
                each row is x, y, theta for one object/robot
        """
        response = self._get_state_service()
        name_state_map = dict(zip(
            response.obj_names, [[pose.x, pose.y, pose.theta] for pose in response.states]))
        if b_active_only:
            object_names = self._ordered_active_objects
        else:
            object_names = self._ordered_movables
        return numpy.array([name_state_map[name] for name in object_names])

    def set_active(self, obj_name):
        """
            Set the only the object with the given name active.
            This is a convenience function to set only a single object (and the robot)
            active. Any other object will be inactive.
        """
        active_mapping = {m: False for m in self._ordered_movables}
        active_mapping[self._robot_name] = True
        active_mapping[obj_name] = True
        self.set_active_objects(active_mapping)

    def set_active_objects(self, active_mapping):
        """
            Set which objects to activate.
            ---------
            Arguments
            ---------
            active_mapping, dict - maps object names to bools indicating whether the object
                should be active or not.

            Any object that is not specified in active_mapping, remains in its current state of
            activity.
        """
        request = SetActiveObjectsRequest(active_mapping.keys(), active_mapping.values())
        response = self._set_active_objects_service(request)
        if not response.service_success:
            raise RuntimeError("Could not set activity for objects " + str(active_mapping.keys()))
        old_active_objects = set(self._active_objects)
        deactivated_objects = set([obj_name for (obj_name, b_active) in active_mapping.iteritems() if not b_active])
        activated_objects = set([obj_name for (obj_name, b_active) in active_mapping.iteritems()
                                 if b_active and obj_name != self._robot_name])
        self._active_objects = old_active_objects - deactivated_objects | activated_objects
        self._ordered_active_objects = [o for o in self._ordered_movables if o in self._active_objects]

    def propagate(self, action):
        """
            Forward propagate the given action.
            ---------
            Arguments
            ---------
            action, iterable of floats representing action to forward propagate
            -------
            Returns
            -------
            valid, bool - True if resulting state is valid
        """
        request = PropagateRequest(action)
        response = self._propagate_service(request)
        if not response.service_success:
            # TODO figure out what should be done in this case
            return False
        return response.valid_propagation

    def set_static_pose(self, **kwargs):
        """
            Set the poses of static obstacles.
            ---------
            Arguments
            ---------
            The states are passed as keyword arguments to this function:
            <name>: <state>, where <name> is the name of the static, and <state> is an iterable of
            length 3 describing the pose of the static obstacle.
            -------
            Returns
            -------
            valid, bool - True if no static obstacles intersect with each other
        """
        request = SetStateRequest()
        request.obj_names = kwargs.keys()
        request.states = [Pose2D(*state) for state in kwargs.values()]
        response = self._set_static_pose_service(request)
        if not response.service_success:
            raise RuntimeError("Set state service failed.")
        return response.valid_state
