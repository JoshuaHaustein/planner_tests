#! /usr/bin/python
from bayes_opt import BayesianOptimization
from planner_tests.ros_oracle_bridge import ROSOracleBridge
import IPython
import numpy as np

ANGLE_SCALE = 0.01


def normalize_radian(val):
    """
        Move val into range [0, 2pi].
        ---------
        Arguments
        ---------
        val, float - angle in radians
        -------
        Returns
        -------
        val', float - angle moved into interval [0, 2pi]
    """
    num_intervals = np.floor(val / (2.0 * np.pi))
    val -= num_intervals * 2.0 * np.pi
    return val


def angle_distance(val1, val2):
    val1 = normalize_radian(val1)
    val2 = normalize_radian(val2)
    if val2 < val1:
        val2, val1 = val1, val2
    return min(val2 - val1, val1 + 2 * np.pi - val2)


class ActionLearningProblem(object):
    def __init__(self, bridge, state, target_state):
        """
            Create a new ActionLearningProblem.
            ---------
            Arguments
            ---------
            bridge, ROSOracleBridge - access to simulator
            state, np array of shape (2, 3) - robot and object state for which to learn the action
            target_state, np array of shape (1, 3) - object target state where the policy should push to
        """
        self.bridge = bridge
        self.start_state = state
        self.target_state = target_state

    def evaluate(self, a1, a2, a3, a4):
        self.bridge.set_state(self.start_state)
        bvalid = self.bridge.propagate([a1, a2, a3, a4])
        if not bvalid:
            return -1e10  # return something really bad
        resulting_state = self.bridge.get_state()
        eucl_error = np.linalg.norm(self.target_state[:2] - resulting_state[1, :2])
        angl_error = angle_distance(self.target_state[2], resulting_state[1, 2])
        return -eucl_error - ANGLE_SCALE * angl_error

    def get_bounds(self):
        aspace_info = self.bridge.get_action_space_info()
        assert aspace_info.dof_action_space == 4
        lower_bounds = aspace_info.lower_bounds
        upper_bounds = aspace_info.upper_bounds
        bounds_tuple = zip(lower_bounds, upper_bounds)
        return dict(zip(['a1', 'a2', 'a3', 'a4'], bounds_tuple))


if __name__ == "__main__":
    bridge = ROSOracleBridge()
    learning_prob = ActionLearningProblem(bridge, bridge.get_state(), np.array([0.9, 0.8, -1]))
    optimizer = BayesianOptimization(f=learning_prob.evaluate,
                                     pbounds=learning_prob.get_bounds(), random_state=1, verbose=2)
    optimizer.maximize(init_points=20, n_iter=30)
    IPython.embed()
