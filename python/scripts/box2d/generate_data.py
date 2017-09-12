#! /usr/bin/python

import argparse
import generate_shapes
import generate_yaml
import subprocess

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate training data for the push planning oracle')
    parser.add_argument('num_samples', type=int, default=10,
                        help='Number of samples to generate for each object instance')
    parser.add_argument('robot_path', type=str)
    parser.add_argument('output_path', type=str, default='training_data/')
    args = parser.parse_args()

    base_path = args.output_path
    robot_file = args.robot_path
    scales = [float(x) / 100 for x in range(2, 18, 5)]
    masses = [float(x) / 100 for x in range(1, 80, 8)]
    friction_coeffs = [0.02 + float(x) / 100 for x in range(1, 30, 6)]
    shapes = generate_shapes.get_all_shapes()
    counter = 0
    for shape in shapes:
        for scale in scales:
            for mu in friction_coeffs:
                for mass in masses:
                    unique_id = shape.get_name() + '_' + str(scale) + '_' + str(mass) + '_' + str(mu)
                    object_file = base_path + '/objects/' + unique_id + '.yaml'
                    generate_yaml.create_yaml_desc(object_file, shape, scale, mass, mu, shape.get_name())
                    world_file = base_path + '/worlds/' +  unique_id + '_world.yaml'
                    generate_yaml.create_world_yaml_desc(world_file, robot_file, object_file)
                    planning_file = base_path + '/planning_files/' + unique_id + '_plandesc.yaml'
                    generate_yaml.create_planning_yaml_desc(planning_file, world_file)
                    # TODO launch data generator
                    subprocess.call(['/home/joshua/projects/planning_catkin/devel/lib/planner_tests/box2d_oracle_generation',
                                     '--output_file', base_path + '/data/' + unique_id + '_',
                                     '--planning_problem', planning_file,
                                     '--threads', str(7),
                                     '--num_samples', str(args.num_samples)])
                    # print object_file
                    # print world_file
                    # print planning_file

    print counter
