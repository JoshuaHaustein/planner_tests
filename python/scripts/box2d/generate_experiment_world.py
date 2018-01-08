#! /usr/bin/python3

import os
import argparse

import jinja2

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate experiment world with parameters of your choice. Outputs' + \
                                                 'to \'experiment_world.yaml\' in the same folder as the template')
    parser.add_argument('--template_path', type=str, help="Path to template", required=True)
    parser.add_argument('--oracle_type', type=str, choices=['Learned', 'Human'], help="Which oracle to use", required=True)
    parser.add_argument('--algorithm', type=str, choices=['CompleteSliceOracleRRT', 'SliceOracleRRT'], help='Which algorithm to use', required=True)
    args = parser.parse_args()
    with open(args.template_path) as f:
        template = jinja2.Template(f.read())
    rendered = template.render(
        oracle_type=args.oracle_type,
        algorithm=args.algorithm,
    )

    output_path = os.path.join(os.path.dirname(args.template_path), 'experiment_world.yaml')

    with open(output_path, 'w') as f:
        f.write(rendered + '\n')
