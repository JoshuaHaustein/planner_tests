import argparse
import planner_tests.ros_oracle_bridge
import numpy as np
import IPython
from pysmac.optimize import fmin


class FrictionOptimization(object):
    def __init__(self, observations, bridge, validation=0.1):
        self._observations = observations
        self._bridge = bridge
        validation_msk = np.random.rand(len(observations)) < validation
        self._validation_set = self._observations[validation_msk]
        self._training_set = self._observations[~validation_msk]
        # validate that Box2D world and observation set are compatible
        self._state_dim = 0
        self._action_dim = 0
        self._validate_setup()
        self.so2_weight = 0.1
        # first get object names
        props = self._bridge.get_object_properties()
        # ensure the robot is an object
        assert("robot" in props.obj_names)
        # add an element for every object
        self._obj_names = [name for name in props.obj_names if name != "robot"]

    def optimize(self, num_evals=1000):
        """
            Run optimization algorithm to tune friction parameters.
        """
        x0 = self._get_start_value()
        xmin = self._get_lower_bounds()
        xmax = self._get_upper_bounds()
        best_params, error = fmin(self._evaluate, x0=x0,
                                  xmin=xmin, xmax=xmax,
                                  max_evaluations=num_evals)
        validation_error = self._compute_error(best_params['x'], self._validation_set)
        return best_params, error, validation_error

    def _get_lower_bounds(self):
        # for now we assume that all objects have the same friction params
        # parameters are contact_friction_robot, contact_friction_obj, ground_friction_obj, ground_torque_obj
        return (0.01, 0.01, 0.001, 0.0001)

    def _get_upper_bounds(self):
        return (1.5, 1.5, 1.0, 0.5)

    def _get_start_value(self):
        # TODO pick a better one? From initial simulation?
        # return (0.1, 1.5, 0.1, 0.001)
        return (0.3, 1.2, 0.2, 0.01)

    def _validate_setup(self):
        # get state from bridge
        state = self._bridge.get_state()
        action_props = self._bridge.get_action_space_info()
        num_cols = self._observations.shape[1]
        self._state_dim = state.shape[0] * state.shape[1]
        self._action_dim = action_props.dof_action_space
        assert(num_cols == 2 * self._state_dim + self._action_dim)

    def _evaluate(self, x):
        return self._compute_error(x, self._training_set)

    def _so2_error(self, values_a, values_b):
        deltas = values_a - values_b
        for r in range(deltas.shape[0]):
            if abs(deltas[r]) > np.pi:
                if (deltas[r] > 0.0):
                    deltas[r] -= 2.0 * np.pi
                else:
                    deltas[r] += 2.0 * np.pi
        return self.so2_weight * np.sum(np.abs(deltas))

    def _compute_error(self, x, data_set):
        params = {}
        # first set physics parameters
        params["robot"] = {"contact_friction_coeff": x[0]}
        obj_param = {"contact_friction_coeff": x[1],
                     "ground_friction_coeff": x[2], "ground_friction_torque_integral": x[3]}
        for obj_name in self._obj_names:
            params[obj_name] = obj_param
        self._bridge.set_object_properties(params)
        error = 0.0
        # now run over the given data set and compute propagation error
        assert(len(data_set) > 0)
        for obs in data_set:
            # extract initial state from observation
            init_state = obs[:self._state_dim].reshape(self._state_dim / 3, 3)
            target_state = obs[self._state_dim + self._action_dim:].reshape(self._state_dim / 3, 3)
            # set simulator to init state
            bvalid = self._bridge.set_state(init_state)
            if not bvalid:
                print "Warning: Encountered invalid initial state in training data"
                continue
            # extract action
            action = obs[self._state_dim:self._state_dim + self._action_dim]
            # propagate action
            bvalid = self._bridge.propagate(action)
            # TODO what to do about invalid states?
            result_state = self._bridge.get_state()
            delta_state = result_state[:2] - target_state[:2]
            error += np.sum(np.linalg.norm(delta_state, axis=1))
            error += self._so2_error(result_state[:, 2], target_state[:, 2])
        return error / len(data_set)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Tune friction parameters of Box2D given a set of push observations. Requires ros_training_server to be running.')
    parser.add_argument('observations', type=str,
                        help='A csv file containing rows of state,action,state tuples. The first row is ignored.')
    args = parser.parse_args()
    # load observations
    observations = np.load(args.observations)
    # TODO could launch ROSOracleServer from here (with the correct world to load)
    # connect ROS bridge
    bridge = planner_tests.ros_oracle_bridge.ROSOracleBridge()
    problem = FrictionOptimization(observations[:20], bridge)
    IPython.embed()
