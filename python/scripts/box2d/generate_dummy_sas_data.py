import planner_tests.ros_oracle_bridge
import numpy as np
import IPython


def sample_action(bounds, state):
    """
        Sample an action in the direction to the closest object.
    """
    robot_state = state[0]
    delta_pos = state[1:, :2] - robot_state[:2]
    closest_obj_state = state[np.argmin(np.linalg.norm(delta_pos, axis=1)) + 1]
    act_dir = closest_obj_state[:2] - robot_state[:2]
    angle = np.arctan2(act_dir[1], act_dir[0])
    # compute action
    rnd_action = np.random.random(4)
    # random velocity magnitudes
    rnd_action[0] = rnd_action[0] * (bounds[1, 0] - bounds[0, 0]) + bounds[0, 0]
    rnd_action[1] = rnd_action[1] * (bounds[1, 1] - bounds[0, 1]) + bounds[0, 1]
    rnd_action[2] = rnd_action[2] * 2.0 + 0.2
    # direction towards closest object
    # rnd_action[3] = rnd_action[3] * (bounds[1, 2] - bounds[0, 2]) + bounds[0, 2]
    rnd_action[3] = angle - robot_state[2]
    return rnd_action


if __name__ == "__main__":
    # create ROS bridge
    bridge = planner_tests.ros_oracle_bridge.ROSOracleBridge()
    num_samples = 200
    num_action_samples = 10
    # TODO get world bounds from bridge
    world_bounds = np.array([[-0.4, -0.15], [-0.1, 0.35]])
    sas_samples = []
    # get number of objects
    obj_props = bridge.get_object_properties()
    action_props = bridge.get_action_space_info()
    action_bounds = np.array([action_props.lower_bounds, action_props.upper_bounds])
    num_objs = len(obj_props.obj_names)
    # sample sas tuple as long as we do not have enough
    while len(sas_samples) < num_samples:
        print len(sas_samples), "/", num_samples
        # sample a random state for robot and object
        random_state = np.random.random((num_objs + 1, 3))
        random_state[:, 0] = random_state[:, 0] * (world_bounds[1, 0] - world_bounds[0, 0]) + world_bounds[0, 0]
        random_state[:, 1] = random_state[:, 1] * (world_bounds[1, 1] - world_bounds[0, 1]) + world_bounds[0, 1]
        random_state[:, 2] = random_state[:, 2] * 2.0 * np.pi - np.pi
        # repeat if we do not have a valid state
        if not bridge.set_state(random_state):
            continue
        # raw_input("Please place the object somewhere pushable")
        # else sample some actions in the hope to touch anything
        for _ in range(num_action_samples):
            # rand_action = np.random.random(action_props.dof_action_space)
            # rand_action = rand_action * (action_bounds[1] - action_bounds[0]) + action_bounds[0]
            random_state = bridge.get_state()
            rand_action = sample_action(action_bounds, random_state)
            if bridge.propagate(rand_action):
                # get state
                result_state = bridge.get_state()
                delta = np.max(np.abs(result_state[1:] - random_state[1:]))
                if delta > 0.01:
                    state_a = random_state.flatten()
                    state_b = result_state.flatten()
                    elem = np.zeros(state_a.shape[0] + state_b.shape[0] + action_props.dof_action_space)
                    elem[:state_a.shape[0]] = state_a
                    elem[state_a.shape[0]: state_a.shape[0] + action_props.dof_action_space] = rand_action
                    elem[state_a.shape[0] + action_props.dof_action_space:] = state_b
                    sas_samples.append(elem)
    frame = np.array(sas_samples)
    np.save('/tmp/dummy_data', frame)
