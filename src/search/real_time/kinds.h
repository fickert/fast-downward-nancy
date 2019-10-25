#ifndef REAL_TIME_KINDS_H
#define REAL_TIME_KINDS_H

namespace real_time
{

enum class LearningMethod
{
 NONE,
 DIJKSTRA,
 NANCY,
};

enum class DecisionStrategy
{
 MINIMIN,
 BELLMAN,
 NANCY,
 CSERNA,
 K_BEST
};

enum class LookaheadSearchMethod
{
 A_STAR,
 A_STAR_COLLECT,
 F_HAT,
 BREADTH_FIRST,
 RISK
};

enum class BoundKind
{
 EXPANSIONS,
 TIME,
};

}

#endif
