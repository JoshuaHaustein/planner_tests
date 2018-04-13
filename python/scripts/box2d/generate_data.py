#! /usr/bin/python

import argparse
import generate_shapes
import generate_yaml
import subprocess
import time
import os
import string
import random


def wait_to_finish(process, max_waiting_time=60.0, delta_t=1.0):
    waited_time = 0.0
    while process.poll() is None and waited_time < max_waiting_time:
        waited_time += delta_t
        time.sleep(delta_t)
    return process.poll() is not None


def create_folder(path):
    if os.path.exists(path):
        return
    os.mkdir(path)


def files_exist(list_of_files):
    file_existance_status = map(lambda x: os.path.exists(x) and os.path.isfile(x), list_of_files)
    return reduce(lambda x,y: x and y, file_existance_status, True)


def create_planning_file_name(unique_id, base_path):
    return base_path + '/planning_files/' + unique_id + '_plandesc.yaml'


def create_object_file_name(unique_id, base_path):
    return base_path + '/objects/' + unique_id + '.yaml'


def create_world_file_name(unique_id, world_file):
    return base_path + '/worlds/' +  unique_id + '_world.yaml'


def create_auxilary_files(unique_id, base_path, robot_file, shape, scale, mass, mu):
    object_file = create_object_file_name(unique_id, base_path)
    planning_file = create_planning_file_name(unique_id, base_path)
    world_file = create_world_file_name(unique_id, base_path)
    generate_yaml.create_yaml_desc(object_file, shape, scale, mass, mu, shape.get_name())
    generate_yaml.create_world_yaml_desc(world_file, robot_file, object_file, object_name=shape.get_name())
    generate_yaml.create_planning_yaml_desc(planning_file, world_file, object_name=shape.get_name())


def generate_data(unique_id, base_path, num_threads, b_overwrite, max_waiting_time, fails_log_file, noisy_propagation):
    output_file_name = base_path + '/data/' + unique_id + '_'
    output_files = [output_file_name + str(i) for i in range(num_threads)]
    planning_file = create_planning_file_name(unique_id, base_path)
    # check whether the output_files already exist, in that case skip this test
    if not b_overwrite and files_exist(output_files):
        print "The output files for case %s already exist. Skipping it." % unique_id
    else:
        process_arguments = ['/home/oracle-trainer/workspace/planning_catkin/devel/lib/planner_tests/box2d_oracle_generation',
                                          '--output_file', output_file_name,
                                          '--planning_problem', planning_file,
                                          '--threads', str(num_threads),
                                          '--num_samples', str(args.num_samples)]
        if noisy_propagation:
            process_arguments.append('--noisy_propagation')
        print "Running child process ", process_arguments
        child_process = subprocess.Popen(process_arguments)
        success = wait_to_finish(child_process, max_waiting_time)
        if not success:
            print "Child process seems to be stuck. Killing it and logging failure."
            child_process.kill()
            failed_file = open(fails_log_file, 'a')
            failed_file.write(unique_id + '\n')
            failed_file.close()
            return True
    return False


def generate_cases(subset_file_name):
    if len(subset_file_name) > 0:  # in case we have a subset file, read cases
        subset_file = open(subset_file_name, 'r')
        unique_ids = [string.rstrip(line, '\n') for line in subset_file]
        subset_file.close()
        return unique_ids, {}
    # else create them here
    scales = [float(x) / 100 for x in range(4, 19, 3)]
    # scales = [0.04]
    masses = [float(x) / 100 for x in range(1, 81, 10)]
    # masses = [0.01]
    friction_coeffs = [0.006 + float(x) / 1000 for x in range(0, 321, 40)]
    # friction_coeffs = [0.006]
    shapes = generate_shapes.get_all_shapes()
    unique_ids = []
    properties = {}
    for shape in shapes:
        for scale in scales:
            for mu in friction_coeffs:
                for mass in masses:
                    unique_id = shape.get_name() + '_' + str(scale) + '_' + str(mass) + '_' + str(mu)
                    unique_ids.append(unique_id)
                    properties[unique_id] = (shape, scale, mass, mu)
    return unique_ids, properties


def random_case():
    scale = 0.08 * random.random() + 0.04
    mass = 0.01 + random.random() * 0.21
    mu = 0.01 + random.random() * 0.315
    shape = random.choice(generate_shapes.get_all_shapes())
    return shape, scale, mass, mu


def generate_list_data(args, max_waiting_time, base_path):
    print "Generating data from a grid of parameters"
    counter = 0
    num_failures = 0
    unique_ids, properties = generate_cases(args.subset)
    total_num_batches = len(unique_ids)
    for unique_id in unique_ids:
        if len(properties) > 0:
            (shape, scale, mass, mu) = properties[unique_id]
            create_auxilary_files(unique_id=unique_id, base_path=base_path,
                                  robot_file=args.robot_path, shape=shape, scale=scale,
                                  mass=mass, mu=mu)
        print "Executing batch %i of %i. That is %.2f percent" % (counter, total_num_batches, float(counter)/float(total_num_batches) * 100.0)
        b_run_failed = generate_data(unique_id=unique_id, base_path=base_path,
                                     num_threads=args.num_threads, b_overwrite=args.overwrite,
                                     max_waiting_time=max_waiting_time,
                                     fails_log_file=args.fails_log_file,
                                     noisy_propagation=args.noisy_propagation)
        if b_run_failed:
            num_failures += 1
        counter += 1
    return num_failures, counter


def generate_random_data(args, max_waiting_time, base_path):
    print "Generating data from a random dynamics parameters"
    num_failures = 0
    for idx in range(0, args.num_random_dynamics):
        (shape, scale, mass, mu) = random_case()
        create_auxilary_files(unique_id='random_data', base_path=base_path,
                              robot_file=args.robot_path, shape=shape, scale=scale,
                              mass=mass, mu=mu)
        print "Executing batch %i of %i. That is %.2f percent" % (idx, args.num_random_dynamics, float(idx)/float(args.num_random_dynamics) * 100.0)
        b_run_failed = generate_data(unique_id='random_data', base_path=base_path,
                                     num_threads=args.num_threads, b_overwrite=True,
                                     max_waiting_time=max_waiting_time,
                                     fails_log_file=args.fails_log_file, noisy_propagation=args.noisy_propagation)
        if b_run_failed:
            num_failures += 1

    return num_failures, args.num_random_dynamics


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate training data for the push planning oracle')
    parser.add_argument('num_samples', type=int, default=1000,
                        help='Number of samples to generate for each object instance')
    parser.add_argument('robot_path', type=str, help="Path to robot file to use")
    parser.add_argument('output_path', type=str, default='training_data/', help="path to folder where to save generated data")
    parser.add_argument('fails_log_file', type=str, default='fails.log', help='path to a file in which the failed cases are stored')
    parser.add_argument('num_threads', type=int, default=6)
    parser.add_argument('--subset', type=str, default='', help='Optionally only generate data for the cases specified in this file')
    parser.add_argument('--overwrite', dest='overwrite', action='store_true', help='overwrite existing files')
    parser.add_argument('--noisy_propagation', dest='noisy_propagation', action='store_true', help='If true, make noisy propagations for each state-action pair')
    parser.add_argument('--num_random_dynamics', type=int, dest='num_random_dynamics', default=0,
                        help='If provided, the script randomly samples dynamic parameters instead of taking these from a list.'
                             ' In this case, the auxillary files are overwritten for each case.')
    args = parser.parse_args()

    base_path = args.output_path
    create_folder(base_path + '/objects')
    create_folder(base_path + '/worlds')
    create_folder(base_path + '/planning_files')
    create_folder(base_path + '/data')
    max_waiting_time = 360.0

    if args.num_random_dynamics > 0:
        num_failures, total_count = generate_random_data(args, max_waiting_time, base_path)
    else:
        num_failures, total_count = generate_list_data(args, max_waiting_time, base_path)

    print "All data generated. There were %i failures out of %i runs" % (num_failures, total_count, noisy_propagation)
    if num_failures > 0:
        print "See %s for failures" % args.fails_log_file
