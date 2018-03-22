#! /usr/bin/python3

import os
import argparse

import jinja2

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate experiment world with parameters of your choice. Outputs' + \
                                                 'to \'experiment_world.yaml\' in the same folder as the template')
    parser.add_argument('--template_path', type=str, help="Path to template", required=True)
    parser.add_argument('--oracle_type', type=str, choices=['Learned', 'Human'], help="Which oracle to use", required=True)
    parser.add_argument('--algorithm', type=str, choices=['HybridActionRRT', 'SliceOracleRRT', 'OracleRRT'], help='Which algorithm to use', required=True)
    parser.add_argument('--num_control_samples', type=int, default=8, help='Required option for hybrid action RRT', required=False)
    parser.add_argument('--action_noise', type=float, default=0.03, help='Action noise, required for HybridActionRRT', required=False)
    parser.add_argument('--shortcut_type', type=str, choices=['NoShortcut', 'LocalShortcut', 'NaiveShortcut', 'OracleShortcut', 'LocalOracleShortcut'], default='NoShortcut', help='Type of shortcut algorithm.', required=False)
    parser.add_argument('--shortcut_timeout', type=float, default=5.0, help='Timeout for shortcutting algorithm in seconds', required=False)
    args = parser.parse_args()
    with open(args.template_path) as f:
        template = jinja2.Template(f.read())
    kwargs = dict()

    # Same for all (or not used)
    kwargs['state_noise'] = 0.005
    kwargs['num_control_samples'] = args.num_control_samples      # try [1, 4, 8]

    kwargs['action_noise'] = args.action_noise                    # For hybrid, try: [0, 0.5, 1.0]
    kwargs['shortcut_type'] = args.shortcut_type
    kwargs['shortcut_timeout'] = args.shortcut_timeout
    rendered = template.render(
        oracle_type=args.oracle_type,
        algorithm=args.algorithm,
        **kwargs
    )

    output_path = os.path.join(os.path.dirname(args.template_path), 'experiment_world.yaml')

    with open(output_path, 'w') as f:
        print(rendered + '\n')
        f.write(rendered + '\n')
