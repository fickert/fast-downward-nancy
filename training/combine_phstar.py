#! /usr/bin/env python3
import os
import shutil
import argparse

argparser = argparse.ArgumentParser(description="merge h* data")
argparser.add_argument("path", help="directory containing h* data files")
args = argparser.parse_args()

def add_entry(df, hstar, hstar_values):
    if not df in hstar_values:
        hstar_values[df] = {}
    if not hstar in hstar_values[df]:
        hstar_values[df][hstar] = 0
    hstar_values[df][hstar] += 1

hstar_values = {}
for file_name in os.listdir(args.path):
    with open(os.path.join(args.path, file_name)) as f:
        lines = f.readlines()
    for line in lines:
        if not len(line.split()) == 3:
            continue
        h, hstar, ph = [int(s) for s in line.split()]
        df = (h,ph)
        add_entry(df, hstar, hstar_values)

for df in sorted(hstar_values):
    value_count = sum(hstar_values[df].values())
    print('{:d} {:d} {:d} {}'.format(df[0],df[1], value_count, ' '.join('{:d} {:d}'.format(hstar, hstar_values[df][hstar]) for hstar in sorted(hstar_values[df]))))
