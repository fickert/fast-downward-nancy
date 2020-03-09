# Nancy Training

The Nancy algorithm heavily relies on its beliefs.  One novel approach
that was implemented here, is to use offline generated data as these
beliefs.  This directory contains resources to set up this training
process.

The general workflow is as follows:

1. Generate training instances
2. Run a *compute\_hstar* or *rt\_solve\_all* search on these instances
   to solve some states optimally, generating (h,h\*) samples.
3. Convert the data generated in step 2 into the format used by Nancy.


## 1. Generating Instances

Nancy requires data for each individual domain.  Hence, we need to
generate training instances of each domain.  The script
**make\_training\_instances.sh** is a bash script that automates this
process.  Ideally, you should just be able to run it.  The generators
are implemented in C, C++, Python, Perl, Clojure/Java.  To be able to
run all generators, make sure that the required runtimes are installed
on your system.  (Clojure is only used for Tidybot.  There is a
rewrite in C, which hasn't been tested however and is not guaranteed
to be equivalent.  To try it, replace the call to tidybot-1.0.1.jar
with a call to tidy.)

The script uses the Makefile to compile all generators that need to be
compiled, and then generates the instances.  The parameters given to
the instance generators were chosen to be as close as possible to the
ones used in the IPC, since we want instances that are as similar as
possible.  If the generator accepts a random seed however, we use a
different one, since we don't want exactly the same instances as the
IPC.

When called without any parameters, the script generates instances for
all domains.  To (re-)generate instances for a subset of domains,
provide their lower-case names as command line parameters, i.e. a
subset of barman, blocks, elevators, parking, rovers, satellites,
termes, tidybot, transport, visitall.

The generators themselves were taken from [the FF Domain
Collection](https://fai.cs.uni-saarland.de/hoffmann/ff-domains.html)
and [the Planning Researchers Bitbucket
Repository](https://bitbucket.org/planning-researchers/pddl-generators).

## 2. Training Runs

With the training instances in place, you need to run a training
variant of the search.

    ./fast-downward.py \
        /path/to/domain.pddl \
        /path/to/instance.pddl \
        --search "compute_hstar(h=lmcut(),
                                cost_type=NORMAL,
                                collect_parent_h=false,
                                w=2,
                                max_time=46800,
                                reserved_time=3600,
                                hstar_file=hstar_values_for_instance.txt,
                                successors_file=successor_values_for_instance.txt)"

This runs weighted A\* with a weight of w=2 on the given instance.  The
relevant parameters here are max\_time which is the maximum time in
seconds after which the algorithm is terminated, and reserved_time
which is the time in seconds reserved to just dumping the values to a
file.  Generating data requires solving a bunch of states optimally.
To make sure that enough data is generated, max\_time should be set to
at least a few hours to give the algorithm enough time to look at a
sufficiently large number of example states.  Keep in mind though,
that when running for longer than 10 hours on a large instance, the
algorithm may need a lot of memory to keep all the states (> 8Gb).
The parameters hstar\_file (and successors\_file) specify where the
algorithm should write the resulting hstar values.  To use the
experimental approach that also records the parent h, set
collect\_parent\_h to true.  Otherwise just leave it out.

There is also an alternative variant of the above call that enables
you to use a different initial search algorithm than weighted A\*.
Here is an example call that runs LSS-LRTA\* as the initial search.

    ./fast-downward.py \
        /path/to/domain.pddl \
        /path/to/instance.pddl \
        --search "rt_solve_all(real_time(h=lmcut(),
                                         distance_heuristic=lmcut(transform=adapt_costs(cost_type=1)),
                                         lookahead_search=A_STAR_COLLECT,
                                         lookahead_bound=100,
                                         decision_strategy=MINIMIN
                                         learning=DIJKSTRA),
                               cost_type=NORMAL,
                               w=2,
                               max_time=46800,
                               reserved_time=3600,
                               hstar_file=hstar_values_for_instance.txt,
                               successors_file=successor_values_for_instance.txt)"

To use an algorithm as the initial search algorithm in this training
step, it needs to be a search engine that implements
get\_expanded\_states().  In this project, this has only been done for
LSS-LRTA\* as an example.  It would definitely be technically feasible
to use assumption-driven Nancy as the initial search and interesting
to see the effects of doing so.  But this is still open at the time of
writing and would have to be implemented first.

## 3 Data Conversion

The data generated in the previous step comes in the form of (h,h\*)
samples.  Nancy expects them to be aggregated in a histogram format
first.  Use the **combine\_hstar.py** script for this.  As a parameter, it
expects the path to the directory containing the raw hstar data files.
The aggregated beliefs are printed to stdout (and can be redirected to
a file for example).  If collect\_parent\_h was set to true, use
combine\_phstar.py instead.  To aggregate data for the
post\_expansion\_belief, use **combine\_post.py**.  It requires both the
path to the directory containing raw post data files, and the path to
the hstar file that was generated with combine_hstar.  Using data for
the post expansion belief that takes the parent h into account is
currently not supported.
