#! /usr/bin/python

import argparse
import generate_shapes
import generate_yaml
import subprocess
import time
import os


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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate training data for the push planning oracle')
    parser.add_argument('num_samples', type=int, default=1000,
                        help='Number of samples to generate for each object instance')
    parser.add_argument('robot_path', type=str)
    parser.add_argument('output_path', type=str, default='training_data/')
    parser.add_argument('fails_log_file', type=str, default='fails.log')
    parser.add_argument('num_threads', type=int, default=6)
    parser.add_argument('--overwrite', dest='overwrite', action='store_true')
    args = parser.parse_args()

    base_path = args.output_path
    create_folder(base_path + '/objects')
    create_folder(base_path + '/worlds')
    create_folder(base_path + '/planning_files')
    create_folder(base_path + '/data')
    robot_file = args.robot_path
    num_threads = args.num_threads
    max_waiting_time = 120.0
    # scales = [float(x) / 100 for x in range(4, 19, 3)]
    scales = [0.04]
    # masses = [float(x) / 100 for x in range(1, 81, 10)]
    masses = [0.01]
    # friction_coeffs = [0.006 + float(x) / 1000 for x in range(0, 321, 40)]
    friction_coeffs = [0.006]
    shapes = generate_shapes.get_all_shapes()
    counter = 0
    num_failures = 0
    total_num_batches = len(shapes) * len(scales) * len(friction_coeffs) * len(masses)
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
                    print "Executing batch %i of %i. That is %.2f percent" % (counter, total_num_batches, float(counter)/float(total_num_batches) * 100.0)
                    output_file_name = base_path + '/data/' + unique_id + '_'
                    output_files = [output_file_name + str(i) for i in range(num_threads)]
                    # check whether the output_files already exist, in that case skip this test
                    if not args.overwrite and files_exist(output_files):
                        print "The output files for case %s already exist. Skipping it." % unique_id
                    else:
                        child_process = subprocess.Popen(['/home/oracle-trainer/workspace/planning_catkin/devel/lib/planner_tests/box2d_oracle_generation',
                                                          '--output_file', output_file_name,
                                                          '--planning_problem', planning_file,
                                                          '--threads', str(num_threads),
                                                          '--num_samples', str(args.num_samples)])
                        success = wait_to_finish(child_process, max_waiting_time)
                        if not success:
                            print "Child process seems to be stuck. Killing it and logging failure."
                            child_process.kill()
                            num_failures += 1
                            failed_file = open(args.fails_log_file, 'a')
                            failed_file.write(unique_id + '\n')
                            failed_file.close()
                    counter += 1
                    # print object_file
                    # print world_file
                    # print planning_file
    print "All data generated. There were %i failures out of %i runs" % (num_failures, counter)
    if num_failures > 0:
        print "See %s for failures" % args.fails_log_file
