#!/usr/bin/env python
import os
import sys
from collections import defaultdict
import numpy

numpy.random.seed(18)
here = os.path.dirname(os.path.realpath(__file__))
instance_folder = os.path.join(here, "instances")
ipc_instances = os.listdir(instance_folder)

instances_by_key = defaultdict(set)
for instance in ipc_instances:
    (_, _, plan_len, total_size, dimensions, target) = instance.split("-")
    (_, _, _, _, num_towers, seed) = target.split("_")

    instances_by_key[(dimensions, num_towers)].add (instance)

(size_x, size_y) = (4, 3)
for range_num_towers, range_height  in [([1,2], range(3,8)),
                                        ([3,4], range(3,6)),
                                        ([5,6], range(3,5))]:
    for height in range_height:
        for num_towers in range_num_towers:
            dimensions = "{}x{}x{}".format(size_x, size_y, height)
            instance_set = sorted(instances_by_key[(dimensions, str(num_towers))])
            if not instance_set:
                continue
            random_selected_instance = numpy.random.choice(instance_set)
            print(os.path.join(instance_folder, random_selected_instance))
