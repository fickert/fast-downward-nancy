#include "max_expansions.h"

namespace real_time
{

MaxExpansions::MaxExpansions(int b): stats(nullptr), bound(b) {}

bool MaxExpansions::lookahead_ok()
{
	assert(stats != nullptr);
	return stats->get_expanded() < bound;
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

void MaxExpansions::print_surplus() const
{
	assert(stats != nullptr);
}

}
