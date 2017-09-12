#! /usr/bin/python

import shutil
import os
import argparse

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Replace the header of oracle training data files')
    parser.add_argument('files_to_fix_path', type=str)
    parser.add_argument('correct_headers_path', type=str)
    parser.add_argument('output_path', type=str)
    args = parser.parse_args()

    files_to_fix = os.listdir(args.files_to_fix_path)
    correct_headers = os.listdir(args.correct_headers_path)

    header_lookup = {}
    for header_file in correct_headers:
        key = os.path.split(header_file)[1]
        header_lookup[key] = header_file

    for file_to_fix in files_to_fix:
        key = os.path.split(file_to_fix)[1]
        if key in header_lookup:
            correct_header = header_lookup[key]
            header_file = open(args.correct_headers_path + '/' + correct_header, 'r')
            data_file = open(args.files_to_fix_path + '/' + file_to_fix, 'r')
            output_file = open(args.output_path + '/' + key, 'w')
            data_file.readline() # drop the original header
            output_file.write(header_file.readline())
            shutil.copyfileobj(data_file, output_file)
            header_file.close()
            data_file.close()
            output_file.close()
        else:
            print 'Warning: Could not find correct header for file ', file_to_fix
