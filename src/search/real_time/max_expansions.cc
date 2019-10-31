#include "max_expansions.h"

#ifdef TRACKSC
#include <iostream>
#define BEGINF(X) std::cout << "SC: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "SC: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "SC: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif

namespace real_time
{

MaxExpansions::MaxExpansions(int b): stats(nullptr), bound(b) {}

bool MaxExpansions::lookahead_ok()
{
	assert(stats != nullptr);
	return stats->get_expanded() <= bound;
}

bool MaxExpansions::learning_ok() const
{
	// when only limiting the number of expansions, the learning
	// phase is unbounded.
	return true;
}

void MaxExpansions::initialize(LookaheadSearch const &ls)
{
	stats = &ls.get_statistics();
	assert(stats->get_expanded() == 1);
}

void MaxExpansions::adjust_learning(size_t effort, size_t remaining)
{
	// The expansion bound approach can always takes as long as it
	// needs to during learning.  Therefore there's nothing to
	// adjust.
	(void)&effort;
	(void)&remaining;
}

void MaxExpansions::print_surplus() const
{
	assert(stats != nullptr);
	//TRACKP("Expansions left: " << bound - stats->get_expanded() << "\n");
}

}
