#! /usr/bin/env python3
import os
import shutil
import argparse

argparser = argparse.ArgumentParser(description="merge h* data")
argparser.add_argument("path", help="directory containing h* data files")
args = argparser.parse_args()

def add_entry(h, hstar, hstar_values):
    if not h in hstar_values:
        hstar_values[h] = {}
    if not hstar in hstar_values[h]:
        hstar_values[h][hstar] = 0
    hstar_values[h][hstar] += 1

hstar_values = {}
for file_name in os.listdir(args.path):
    with open(os.path.join(args.path, file_name)) as f:
        lines = f.readlines()
    for line in lines:
        if not len(line.split()) == 2:
            continue
        h, hstar = [int(s) for s in line.split()]
        add_entry(h, hstar, hstar_values)

for h in sorted(hstar_values):
    value_count = sum(hstar_values[h].values())
    print('{:d} {:d} {}'.format(h, value_count, ' '.join('{:d} {:d}'.format(hstar, hstar_values[h][hstar]) for hstar in sorted(hstar_values[h]))))
