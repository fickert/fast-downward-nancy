#!/usr/bin/env python

import os
import random
import itertools
import gen_random_tower_boards
import argparse
import generate

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--seed", type=int, default=1)
args = parser.parse_args()

seed = args.seed

here = os.path.dirname(os.path.realpath(__file__))
#Optimal
(size_x, size_y) = (4, 3)
for range_num_towers, range_height in [([1, 2], range(3, 8)),
                                       ([3, 4], range(3, 6)),
                                       ([5, 6], range(3, 5)) ]:
    for height in range_height:
        for num_towers in range_num_towers:
            for i in range(4):
                scenario = gen_random_tower_boards.gen_board(size_x, size_y, height, num_towers, seed)
                with open(os.path.join(here, "boards", "empty.txt"), "r") as initf:
                    with open(os.path.join(here, "boards", scenario), "r") as goalf:
                        generate.do_it(initf,
                                       goalf,
                                       "pddlfile",
                                       ensure_plan=True,
                                       store_plan=True,
                                       dont_remove_slack=True)
                seed += 1
