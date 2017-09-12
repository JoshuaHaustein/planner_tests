#! /usr/bin/python

import yaml


def create_dictionary(shape, scale, mass, friction, name, friction_coeff=0.45):
    a_dict = dict()
    a_dict['name'] = name + 'link'
    a_dict['geometry'] = list(map(list, shape.get_geometry(scale)))
    a_dict['mass'] = mass
    a_dict['trans_friction'] = friction
    a_dict['rot_friction'] = friction_coeff * friction
    a_dict['restitution'] = 1.0
    a_dict['contact_friction'] = 1.5
    root_dict = dict()
    root_dict['name'] = name
    root_dict['links'] = [a_dict]
    root_dict['joints'] = []
    return root_dict


def create_yaml_desc(file_name, shape, scale, mass, friction, name):
    yaml_desc = create_dictionary(shape, scale, mass, friction, name)
    afile = open(file_name, 'w')
    yaml.dump(yaml_desc, afile)
    afile.close()


def create_world_yaml_desc(file_name, robot_path, object_path):
    root_dict = {'scale': 10.0}
    root_dict['robots'] = [
        {'name': 'robot',
         'filename': robot_path}
    ]
    root_dict['objects'] = [
        {'name': 'target',
         'filename': object_path,
         'static': False}
    ]
    root_dict['world_bounds'] = [-1.8, -1.8, 1.8, 1.8]
    root_dict['states'] = [
        {'name': 'robot',
         'state': {'configuration': [0.0, 0.0, 0.0],
                   'velocity': [0.0, 0.0, 0.0]}
         },
        {'name': 'target',
         'state': {'configuration': [1.0, 1.0, 0.0],
                   'velocity': [0.0, 0.0, 0.0]}
         }
    ]
    a_file = open(file_name, 'w')
    yaml.dump(root_dict, a_file)
    a_file.close()


def create_planning_yaml_desc(file_name, world_path):
    problem_desc = {
        'world_file': world_path,
        'robot_name': 'robot',
        'target_name': 'target',
        'x_limits': [-1.5, 1.5],
        'y_limits': [-1.5, 1.5],
        'z_limits': [0.0, 0.0],
        'max_velocity': 2.0,
        'max_rotation_vel': 2.0,
        'planning_timeout': 60.0,
        'control_limits': {'velocity_limits': [0.6, 0.6, 1.4],
                           'acceleration_limits': [2, 2, 1.4],
                           'duration_limits': [0.05, 0.8]
                           },
        't_max': 8.0,
        'goal_position': [1.0, 1.0, 0.0],
        'goal_region_radius': 0.1,
        'oracle_type': 'None',
        'num_control_samples': 1
    }
    a_file = open(file_name, 'w')
    yaml.dump(problem_desc, a_file)
    a_file.close()