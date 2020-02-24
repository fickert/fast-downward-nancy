Fast Downward is a domain-independent planning system.  This project
is an implementation of a Real-Time planning plugin within Fast
Downward.  The Real-Time constraint is implemented as an expansion and
as a time bound.  The project contains an implementation of LSS-LRTA*,
and Nancy.  These algorithms can run with certain parameters.

Build
-------------------------------------------------------------------------------

The project can be built by simply invoking:

```
build.py
```

The default if no build type is specified is the *minrt* configuration
which contains only the Real-Time plugin and its dependencies.  Use
*build.py release* for a full build including all other available
plugins.


Example Usage
-------------------------------------------------------------------------------

The real_time plugin accepts several parameters.  Not all are
compatibly with each other and not all are used by all search methods.
To avoid confusion, here are some examples for the typical use cases.

Use the following call to run LSS-LRTA* on the given problem instance
using lmcut as the heuristic and an expansion bound of 100.

    ./fast-downward.py \
        --build=minrt \
        /path/to/domain.pddl \
        /path/to/instance.pddl \
        --search "real_time(h=lmcut(),
    	                    cost_type=NORMAL,
    						lookahead_bound=100,
    						rtbound_type=EXPANSIONS,
    						lookahead_search=A_STAR,
    						decision_strategy=MINIMIN,
    						learning=DIJKSTRA)"

This next call runs assumption-based Nancy with an expansion bound of
1000.

    ./fast-downward.py \
        --build=minrt \
        /path/to/domain.pddl \
        /path/to/instance.pddl \
        --search "real_time(lmcut(),
                            cost_type=NORMAL,
                            lookahead_bound=1000,
                            distance_heuristic=lmcut(transform=adapt_costs(cost_type=1)),
                            lookahead_search=ONLINE_RISK,
                            decision_strategy=ONLINE_NANCY,
                            learning=DIJKSTRA"

One more example which runs data-driven Nancy with a time bound of
1000ms.

    ./fast-downward.py \
        --build=minrt \
        /path/to/domain.pddl \
        /path/to/instance.pddl \
        --search "real_time(lmcut(),
                            cost_type=NORMAL,
                            lookahead_bound=1000,
                            distance_heuristic=lmcut(transform=adapt_costs(cost_type=1)),
                            lookahead_search=RISK,
                            decision_strategy=NANCY,
                            learning=NANCY,
                            hstar_data=/path/to/hdata.txt,
                            feature_kind=JUST_H,
                            post_feature_kind=JUST_H)"




All Parameters
-------------------------------------------------------------------------------

This is a complete list of all parameters accepted by the real_time plugin.

- "h": The heuristic function
- "distance_heuristic": The distance heuristic which is used by the
  error-model to estimate heuristic error.  It should be set to a
  unit-cost variant of the heuristic "h".
- "rtbound_type": There are two kinds of bounds:
  * "EXPANSIONS": With this option, the algorithm can perform a fixed
    number of lookahead iterations until it has to select an action.
    This is the default.
  * "TIME": With this option, the algorithm runs under a time bound
    instead.
- "lookahead_bound": The number of lookahead iterations in a single
  search episode.  Naturally, this option is only relevant when
  rtbound_type is set to EXPANSIONS.  Typical values are 100,
  300, 1000. While there is no implemented limit, the algorithms do
  degrade significantly if the bound is set below 30 or above 16000.
- "time_bound": The time in milliseconds before an action should be
  executed. This option only matters when rtbound_type is set to TIME.
  This parameter is hardware dependent, but as a rough guideline,
  choosing values between 100 and 1000 is reasonable.  Note that the
  algorithm assumes that there is at least enough time to do one
  expansion + backup step.
- "lookahead_search": The search method to use.  This is the main
  parameter driving the behavior of the real-time search algorithm.
  The options are:
  * "A_STAR": The basic A* approach follows minimum f during lookahead
  * "A_STAR_COLLECT": This variant behaves exactly like A*, but collects all
    expanded states
  * "F_HAT": This method follows minimum f_hat instead of f during
    lookahead.  Here f_hat refers to g + h_hat, where h_hat is a
    corrected heuristic value which is computed using an error model.
  * "BREADTH_FIRST": Breadth-first lookahead expands nodes in the
    order they were generated
  * "ONLINE_RISK": This approach represents goal distance beliefs as
    probability distributions, and determines under which top level
    action to do more search by using a risk measure.  the beliefs
    about the goal distances are constructed online by making
    assumptions about heuristic behavior.
  * "RISK": This approach works just like online_risk, but is intended
    to be used with beliefs generated from data.  this results in a
    different implementation how the beliefs are constructed and
    cached, hence this is a separate lookahead option.
- "learning": The learning method to use.  Please be aware that the
  chosen lookahead method determines which learning methods are
  compatible.
  * "NONE": Do not perform any updates of the heuristic.  this results
    in an incomplete search algorithm that can be trapped in local
    minima.
  * "DIJKSTRA": Update the heuristic values in a dijkstra like way.
    this updates only heuristic values.  hence, it is available for
    the lookahead methods BFS, A*, f_hat, and online_risk
  * "NANCY": This update strategy works just like the dijkstra backup,
    but backs up distributions instead of scalar heuristic values.  It
    is only compatible with risk (not online_risk), as it is the only
    lookahead method that directly stores distributions.
- "decision_strategy": The decision strategy determines which action
  is chosen in the end.
  * "MINIMIN": Choose the action with minimum f value.
  * "BELLMAN": Choose the action with minimum f_hat value.
  * "ONLINE_NANCY": Also choose the action with minimum f_hat value,
    but persist on a path of lowest f_hat value over several search
    episodes if no better alternative is found in the meantime.
  * "NANCY": This method makes a decision just like online_nancy, but
    performs computations on probability distributions instead.  Hence
    it should be used with risk lookahead.
- "hstar_data": The path to a text file containing the hstar data for
  the risk lookahead.  If no file is provided, risk lookahead falls
  back to the assumption-based approach to construct its
  distributions.
- "post_expansion_belief_data": The path to a text file containing the
  hstar data for the risk lookahead.  If no file is provided, risk
  lookahead falls back to the assumption-based approach to construct
  its distributions.
- "feature_kind": What kind of feature to use to look up the
  distribution in the data.  Note that the chosen feature kind has to
  match the type of data in the text file.  The algorithm may crash
  otherwise.  Currently there are two feature options:
  * "JUST_H": The distribution is looked up by the h value of a
    state. This is the default.
  * "WITH_PARENT_H": The distribution is looked up by a pair of the h
    value and the h value of the parent state.
- "post_feature_kind": Same options as feature_kind, but for the post
  expansion belief.
- "expansion_delay_window_size": This parameter is only relevant for
  methods that use the error model.  It is an int to set the size of
  the sliding average to compute the expansion delay. A value of 0
  sets this to a global average (which is the default).


Contact and License
-------------------------------------------------------------------------------

For documentation and contact information about fast-downward in
general see http://www.fast-downward.org/.

The following directories are not part of Fast Downward as covered by this
license:

* ./src/search/ext

For the rest, the following license applies:

```
Fast Downward is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Fast Downward is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
```
