#! /usr/bin/python3

from oracle_server.models import Oracle, Feasibility
from planner_tests.ros_oracle_bridge import ROSOracleBridge
import rospy
import numpy as np
from enum import Enum
import IPython
import time


class OracleRequest:
    def __init__(self, robot_state, obj_state, obj_prop, obj_target):
        self.robot_x, self.robot_y, self.robot_radians = robot_state
        self.object_x, self.object_y, self.object_radians = obj_state
        self.object_width, self.object_height, self.object_mass, self.object_friction = (
            obj_prop
        )
        self.object_x_prime, self.object_y_prime, self.object_radians_prime = obj_target


class FeasbilityRequest:
    def __init__(self, obj_state, obj_prop, obj_target):
        self.object_x, self.object_y, self.object_radians = obj_state
        self.object_width, self.object_height, self.object_mass, self.object_friction = (
            obj_prop
        )
        self.object_x_prime, self.object_y_prime, self.object_radians_prime = obj_target


class ExtensionTest(object):
    class ExtensionProgress(Enum):
        FAIL = 0
        REACHED = 1

    class PushResult(Enum):
        FAIL = 0
        MOVABLES_BLOCK_PUSH = 2
        PROGRESS = 3
        REACHED = 4

    def __init__(self):
        self._oracle = Oracle()
        self._generator = Feasibility()
        self._push_sim = ROSOracleBridge()
        obj_properties = self._push_sim.get_object_properties()
        self._name_to_id = {
            obj_properties.obj_names[idx]: idx
            for idx in range(len(obj_properties.obj_names))
        }
        self._obj_properties = np.array(
            [
                obj_properties.widths,
                obj_properties.heights,
                obj_properties.masses,
                obj_properties.friction_coeffs,
            ]
        ).transpose()
        self._all_movables = range(0, self._obj_properties.shape[0])
        self.num_sample_trials = 10
        self.rad_weight = 0.2
        self.goal_tol = 0.04

    def sample_feasible(self, x_c, goal, t):
        """
            Sample a pushing state for object t given the current world state x_c
            and the goal arrangemet goal.
            ---------
            Arguments
            ---------
            x_c, np,array of shape (m + 1, 3) - current world state
            goal, np.array of shape (m, 3) - goal slice/arrangement
            t, int - object to push
            -------
            Returns
            -------
            x_prime, np.array of shape (m + 1, 3) - world state with x_prime[1:] == x_c, and
                 x_prime[0] being the sampled goal state
        """
        req = FeasbilityRequest(x_c[t + 1], self._obj_properties[t], goal[t])
        res = self._generator.sample(req)
        result = np.array(x_c)
        result[0] = res.robot_x, res.robot_y, res.robot_radians
        return result

    def query_policy(self, x_c, goal, t):
        """
            Query the policy for an action to transport object t from x_c[t + 1] to goal[t] given
            that the robot is at x_c[0].
            ---------
            Arguments
            ---------
            x_c, np.array of shape (m +1, 3) - current state
            goal, np.array of shape (m, 3) - target arrangement
            t, int - target object index
            -------
            Returns
            -------
            u, np.array of shape (4,) - local ramp twist
        """
        req = OracleRequest(x_c[0], x_c[t + 1], self._obj_properties[t], goal[t])
        res = self._oracle.sample(req)
        u = np.array([res.dx, res.dy, res.dr, res.t])
        R = np.array(
            [
                [np.cos(x_c[0, 2]), np.sin(x_c[0, 2]), 0, 0],
                [-np.sin(x_c[0, 2]), np.cos(x_c[0, 2]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        u = R @ u
        ltwist = np.zeros(4)
        ltwist[0] = np.linalg.norm(u[:2])
        ltwist[1] = u[2]
        ltwist[2] = u[3]
        ltwist[3] = np.arctan2(u[1], u[0])
        return ltwist

    def se2_distances(self, a, b):
        diff = a - b
        dist = np.linalg.norm(diff[:, :2], axis=1) + self.rad_weight * np.abs(
            diff[:, 2]
        )
        return dist

    def get_push_blockers(self, x_b, x_a, goal, tol=0.02):
        """
            Return movable objects that in x_a are further from their respective goal pose in goal
            than in x_b.
            ---------
            Arguments
            ---------
            x_b, np array shape (m + 1, 3) - world state before a push
            x_a, np array shape (m + 1, 3) - world state after a push
            goal, np.array shape (m, 3) - goal arrangement 
            tol, float - tolerance value that a movable is allowed to be move away from the goal
            -------
            Returns
            -------
            blockers, set of int - objects that block a push
        """
        distances_before = self.se2_distances(goal, x_b[1:])
        distances_after = self.se2_distances(goal, x_a[1:])
        delta_distances = distances_after - distances_before
        violators = np.where(delta_distances > tol)[0]
        return set(violators)

    def made_progress(self, x_b, x_a, goal, t):
        """
            Return whether or not we made progress transporting t towards its goal.
            ---------
            Arguments
            ---------
            x_b, np array shape (m + 1, 3) - state before
            x_a, np array shape (m + 1, 3) - state after
            goal, np array shape (m, 3) - goal
            t, int - target id
            -------
            Returns
            -------
            made_progress, bool - True if progress was made, else False
        """
        dist_to_goal_before = self.se2_distances(
            x_b[t + 1].reshape((1, 3)), goal[t, :].reshape((1, 3))
        )
        dist_to_goal_after = self.se2_distances(
            x_a[t + 1].reshape((1, 3)), goal[t, :].reshape((1, 3))
        )
        delta = dist_to_goal_after - dist_to_goal_before
        return delta[0] < 0.0

    def close_enough(self, x, goal, t):
        cart_distance = np.linalg.norm(x[t + 1, :2] - goal[t, :2])
        return cart_distance < self.goal_tol

    def try_push(self, x_c, t, goal, movables, b_new_push):
        """
            Try to push the given object t towards its pose in goal given that the world is in state
            x_c. The robot may additionally push the objects in movables, but only if these are not pushed
            away from their respective goal poses.
            ---------
            Arguments
            ---------
            x_c, np.array of shape (m+1, 3) - current_state
            t, int - target object
            goal, np.array of shape (m, 3) - target poses of objects
            movables, set of ints - movable objects that may be moved by the robot
            b_new_push, bool - True indicates that the robot is not in a pushing state yet
            -------
            Returns
            -------
            result, PushResult,
            state, np array of shape (m+1, 3) - resulting state
            blockers, set of int - movables that can be moved and are blocking the push
            action, np array of shape (4) - pushing action
        """
        if self.close_enough(x_c, goal, t):
            return ExtensionTest.PushResult.REACHED, x_c, set([]), np.empty(0)
        blockers = set([])
        min_num_blockers = np.inf
        best_result_state = None
        best_action = None
        for trid in range(self.num_sample_trials):
            # sample pushing state
            if trid > 0 or b_new_push:
                state = self.sample_feasible(x_c, goal, t)
            else:
                # in first iteration keep current state
                state = x_c
            bvalid = self._push_sim.set_state(state)
            if not bvalid:
                # if invalid, try again
                # TODO this would be the point to check what movables are blocking the pushing state
                continue
            # query policy
            action = self.query_policy(state, goal, t)
            # propagate action
            bvalid = self._push_sim.propagate(action)
            if not bvalid:
                # if invalid, try again
                rospy.logwarn(
                    "Invalid propagation. This would need to be treated differently"
                )
                continue
            # check what happened to all movables
            result_state = self._push_sim.get_state()
            bmade_progress = self.made_progress(state, result_state, goal, t)
            new_blockers = self.get_push_blockers(state, result_state, goal)
            movable_blockers = new_blockers & movables
            # if we are allowed to move all blockers, save this as a candidate to return
            if (
                len(movable_blockers) == len(new_blockers)
                and len(new_blockers) < min_num_blockers
                and bmade_progress
            ):
                blockers = new_blockers
                min_num_blockers = len(blockers)
                best_result_state = result_state
                best_action = action
                if min_num_blockers == 0:
                    break
        # check whether we found any solution that either made progress, reached the goal or gives us the chance to
        # clear obstacles
        if min_num_blockers == np.inf:
            return ExtensionTest.PushResult.FAIL, None, None, None
        elif min_num_blockers == 0:
            if self.close_enough(best_result_state, goal, t):
                return (
                    ExtensionTest.PushResult.REACHED,
                    result_state,
                    set([]),
                    best_action,
                )
            return ExtensionTest.PushResult.PROGRESS, result_state, set([]), best_action
        return (
            ExtensionTest.PushResult.MOVABLES_BLOCK_PUSH,
            result_state,
            blockers,
            best_action,
        )

    def recursive_extend(self, t, x_c, targets, remainers, goal):
        """
            Recursive extension function.
            ---------
            Arguments
            ---------
            t, int - id of the target object
            x_c, np array of shape (m+1, 3) - current state
            targets, set of int - remaining target objects
            remainers, set of int - remaining moveables that may be moved
            goal, np.array of shape (m, 3) - target slice
            -------
            Returns
            -------
            progress, ExtensionProgress - either FAIL or REACHED
            resulting_state, np array of shape (m+1, 3)
            remainers, set of int - reamining movables that may be moved (i.e. are not at their goals)
        """
        path = []  # stores tuples (state, action_leading_to_state)
        push_result = None
        b_new_push = True
        pushable_movables = targets | remainers
        while push_result != ExtensionTest.PushResult.REACHED:
            push_result, x_prime, m_b, u = self.try_push(
                x_c, t, goal, pushable_movables, b_new_push
            )
            b_new_push = False
            if push_result == ExtensionTest.PushResult.FAIL:
                return ExtensionTest.ExtensionProgress.FAIL, None, None, []
            # elif push_result == ExtensionTest.PushResult.MOVABLE_APPROACH_FAIL:
            # TODO could we use this information?
            # return ExtensionTest.ExtensionProgress.FAIL, None, None
            elif push_result == ExtensionTest.PushResult.MOVABLES_BLOCK_PUSH:
                sub_result = ExtensionTest.ExtensionProgress.FAIL
                while path and sub_result != ExtensionTest.ExtensionProgress.REACHED:
                    (x_c, u) = path.pop()
                    # run over all blocking movables, and try to clear
                    for m in m_b:
                        m_set = set([m])
                        sub_result, x_prime, remainers_prime, sub_path = self.recursive_extend(
                            m, x_c, m_b - m_set, pushable_movables - m_set - m_b, goal
                        )
                        if sub_result == ExtensionTest.ExtensionProgress.REACHED:
                            remainers = remainers & remainers_prime
                            x_c = x_prime
                            path.extend(sub_path)
                            targets = targets & remainers_prime
                            b_new_push = True
                            break
                if sub_result == ExtensionTest.ExtensionProgress.FAIL:
                    return ExtensionTest.ExtensionProgress.FAIL, None, None, []
            elif (
                push_result == ExtensionTest.PushResult.PROGRESS
                or push_result == ExtensionTest.PushResult.REACHED
            ):
                path.append((x_prime, u))
                x_c = np.array(x_prime)
            else:
                raise ValueError("Unknown PushResult encountered")
        # we succeeded to push t to its goal, try remaining ones
        sub_result = ExtensionTest.ExtensionProgress.REACHED
        for t_prime in targets:
            sub_result, x_prime, remainers_prime, sub_path = self.recursive_extend(
                t_prime, x_c, targets - set([t_prime]), remainers, goal
            )
            if sub_result == ExtensionTest.ExtensionProgress.REACHED:
                x_c = x_prime
                remainers = remainers_prime
                path.extend(sub_path)
                break
        if sub_result == ExtensionTest.ExtensionProgress.REACHED:
            return ExtensionTest.ExtensionProgress.REACHED, x_c, remainers, path
        else:
            return ExtensionTest.ExtensionProgress.FAIL, None, None, []

    def show_path(self, path, sleep_time=0.2):
        """
            Show the given path.
        """
        for wp in path:
            self._push_sim.set_state(wp[0])
            time.sleep(sleep_time)

    def extend(self, current_state, target_slice):
        """
            New extension function to extend a search tree from the current state to the given target
            slice.
        """
        targets = set(self._all_movables)
        path = [(current_state, np.empty(0))]
        for t in self._all_movables:
            result, resulting_state, remainers, sub_path = self.recursive_extend(
                t, current_state, targets - set([t]), set([]), target_slice
            )
            if result == ExtensionTest.ExtensionProgress.REACHED:
                assert len(remainers) == 0
                path.extend(sub_path)
                return ExtensionTest.ExtensionProgress.REACHED, resulting_state, path
        return ExtensionTest.ExtensionProgress.FAIL, current_state, []


if __name__ == "__main__":
    etest = ExtensionTest()
    x_c = etest._push_sim.get_state()
    goal = np.array(x_c[1:])
    # goal[0, 0] = 0.8
    IPython.embed()
    # req = OracleRequest(robot_state, obj_state, obj_prop, target_state)
