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
        gs_name = '/' + server_name + '/get_state'
        prop_name = '/' + server_name + '/propagate'
        sao_name = '/' + server_name + '/set_active_objects'
        ss_name = '/' + server_name + '/set_state'
        rospy.wait_for_service(gasi_name)
        rospy.wait_for_service(gop_name)
        rospy.wait_for_service(gs_name)
        rospy.wait_for_service(prop_name)
        rospy.wait_for_service(sao_name)
        rospy.wait_for_service(ss_name)
        self._get_action_space_info_service = rospy.ServiceProxy(gasi_name, GetActionSpaceInfo)
        self._get_object_properties_service = rospy.ServiceProxy(gop_name, GetObjectProperties)
        self._get_state_service = rospy.ServiceProxy(gs_name, GetState)
        self._propagate_service = rospy.ServiceProxy(prop_name, Propagate)
        self._set_active_objects_service = rospy.ServiceProxy(sao_name, SetActiveObjects)
        self._set_state_service = rospy.ServiceProxy(ss_name, SetState)
        resp = self.get_object_properties()
        self._object_names = resp.obj_names
        self._all_entities_names = ['robot']
        self._all_entities_names.extend(self._object_names)
        rospy.loginfo("Ready")

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
            friction_coeffs, array of floats
            widths, array of floats
            heights, array of floats
        """
        response = self._get_object_properties_service()
        return response

    def set_state(self, state):
        """
            Set the state of the world. 
            ---------
            Arguments
            ---------
            state, numpy array of shape (n, 3) - every row is x, y, theta
            --------
            Returns
            --------
            valid_state, bool - True iff the set state is valid
        """
        request = SetStateRequest()
        request.obj_names = self._all_entities_names
        for obj_state in state:
            request.states.append(Pose2D(*obj_state))
        response = self._set_state_service(request)
        if not response.service_success:
            # TODO decide what to do in this case
            return False
        return response.valid_state

    def get_state(self):
        """
            Return the current state of the world.
            -------
            Returns
            -------
            state, numpy array of shape (n, 3) - each row is x, y, theta for one object/robot
        """
        response = self._get_state_service()
        name_state_map = dict(itertools.izip(
            response.obj_names, [[pose.x, pose.y, pose.theta] for pose in response.states]))
        return numpy.array([name_state_map[name] for name in self._all_entities_names])

    def set_active_objects(self):
        """
            TODO: Implement me!
        """
        raise NotImplementedError("Not supported yet!")

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
