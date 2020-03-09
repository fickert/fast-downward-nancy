#! /usr/bin/env python3
import argparse
import collections
import math
import os
import shutil
import sys

argparser = argparse.ArgumentParser(description="merge successors data and generate post-expansion beliefs")
argparser.add_argument("path", help="directory containing successors data files")
argparser.add_argument("hstar_data", help="h* data file")
args = argparser.parse_args()

# hstar data entry: {h-value --> (total_samples, {h*-value --> num_samples})}
hstar_data_entry = collections.namedtuple('hstar_data_entry', ['value_count', 'hstar_values'])
# h data entry: {h-value --> (total_samples, {h*-value --> {action cost --> count}})}
h_data_entry = collections.namedtuple('h_data_entry', ['value_count', 'h_values'])

def read_hstar_distribution():
    hstar_data = {}
    with open(args.hstar_data) as f:
        lines = f.readlines()
    for line in lines:
        if not line:
            continue
        raw_values = [int(word) for word in line.split()]
        assert len(raw_values) >= 4
        h = raw_values[0]
        if h not in hstar_data:
            hstar_data[h] = hstar_data_entry(0, {})
        total_samples = raw_values[1]
        assert total_samples > 0
        hstar_data[h] = hstar_data[h]._replace(value_count=hstar_data[h].value_count + total_samples)
        assert hstar_data[h].value_count > 0
        assert len(raw_values) % 2 == 0
        for i in range(2, len(raw_values), 2):
            hstar_value = raw_values[i]
            count = raw_values[i + 1]
            if hstar_value not in hstar_data[h].hstar_values:
                hstar_data[h].hstar_values[hstar_value] = 0
            hstar_data[h].hstar_values[hstar_value] += count
    return hstar_data

def get_average_hstar(h, hstar_averages):
    assert h in hstar_averages
    return hstar_averages[h]

def get_hstar_averages(hstar_data):
    averages = {}
    for h in hstar_data:
        total_samples, hstar_samples = hstar_data[h]
        assert all(num_samples <= total_samples for _, num_samples in hstar_samples.items())
        averages[h] = sum(hstar * (num_samples / total_samples) for hstar, num_samples in hstar_samples.items())
    return averages

def add_entry(h, successors, post_expansion_data_intermediate, hstar_averages):
    if h not in post_expansion_data_intermediate:
        post_expansion_data_intermediate[h] = h_data_entry(0, {})
    # simulate Nancy backup
    best_successor_f = math.inf
    best_successor_h = math.inf
    best_successor_action_cost = math.inf
    for action_cost, successor_h in successors:
        if successor_h not in hstar_averages:
            continue
        successor_f = action_cost + get_average_hstar(successor_h, hstar_averages)
        if successor_f < best_successor_f:
            best_successor_f = successor_f
            best_successor_h = successor_h
            best_successor_action_cost = action_cost
    if math.isinf(best_successor_f):
        # print("Warning: no valid successor found, ignoring...", file=sys.stderr)
        return
    assert best_successor_h in hstar_averages
    post_expansion_data_intermediate[h] = post_expansion_data_intermediate[h]._replace(value_count=post_expansion_data_intermediate[h].value_count + 1)
    if best_successor_h not in post_expansion_data_intermediate[h].h_values:
        post_expansion_data_intermediate[h].h_values[best_successor_h] = {}
    if best_successor_action_cost not in post_expansion_data_intermediate[h].h_values[best_successor_h]:
        post_expansion_data_intermediate[h].h_values[best_successor_h][best_successor_action_cost] = 0
    post_expansion_data_intermediate[h].h_values[best_successor_h][best_successor_action_cost] += 1

print("Reading h* data...", file=sys.stderr)

hstar_data = read_hstar_distribution()
hstar_averages = get_hstar_averages(hstar_data)
post_expansion_data_intermediate = {}
post_expansion_data = {}

print("h* data:", file=sys.stderr)

for h in sorted(hstar_averages):
    print('{:d}: {:f}'.format(h, hstar_averages[h]), file=sys.stderr)

print("Reading successors data and constructing intermediate data representation...", file=sys.stderr)

i = 1
total = len(os.listdir(args.path))
for file_name in os.listdir(args.path):
    with open(os.path.join(args.path, file_name)) as f:
        lines = f.readlines()
    for line in lines:
        if not line:
            continue
        raw_values = [int(word) for word in line.split()]
        assert len(raw_values) >= 3
        h = raw_values[0]
        assert len(raw_values) % 2 == 1
        successors = [(raw_values[i], raw_values[i + 1]) for i in range(1, len(raw_values), 2)]
        add_entry(h, successors, post_expansion_data_intermediate, hstar_averages)
    print("\rProcessed {:d} of {:d} files".format(i, total), end='', file=sys.stderr)
    i += 1
print("", file=sys.stderr)

print("Computing post-expansion beliefs...", file=sys.stderr)

for h in post_expansion_data_intermediate:
    post_expansion_data[h] = hstar_data_entry(0, {})
    for h_value, action_costs in post_expansion_data_intermediate[h].h_values.items():
        assert h_value in hstar_data
        for hstar_value, num_samples in hstar_data[h_value].hstar_values.items():
            post_expansion_data[h] = post_expansion_data[h]._replace(value_count=post_expansion_data[h].value_count + num_samples * sum(action_costs.values()))
            for action_cost in action_costs:
                f_value = hstar_value + action_cost
                if f_value not in post_expansion_data[h].hstar_values:
                    post_expansion_data[h].hstar_values[f_value] = 0
                post_expansion_data[h].hstar_values[f_value] += num_samples * action_costs[action_cost]

print("Dumping combined data...", file=sys.stderr)

for h in sorted(post_expansion_data):
    assert post_expansion_data[h].value_count == sum(num_samples for _, num_samples in post_expansion_data[h].hstar_values.items())
    print('{:d} {:d} {}'.format(h, post_expansion_data[h].value_count, ' '.join('{:d} {:d}'.format(hstar, post_expansion_data[h].hstar_values[hstar]) for hstar in sorted(post_expansion_data[h].hstar_values))))
