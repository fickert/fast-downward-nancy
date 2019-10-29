#include "tlas.h"

namespace real_time
{

size_t TLAs::size() const
{
	assert(ops.size() == open_lists.size() &&
	       ops.size() == op_costs.size() &&
	       ops.size() == beliefs.size() &&
	       ops.size() == states.size() &&
	       ops.size() == eval_contexts.size());
	return ops.size();
}

void TLAs::clear()
{
	ops.clear();
	op_costs.clear();
	open_lists.clear();
	beliefs.clear();
	post_beliefs.clear();
	states.clear();
	eval_contexts.clear();
}

void TLAs::reserve(size_t n)
{
	ops.reserve(n);
	op_costs.reserve(n);
	open_lists.reserve(n);
	beliefs.reserve(n);
	post_beliefs.reserve(n);
	states.reserve(n);
	eval_contexts.reserve(n);
}

TLAs::QueueEntry const &TLAs::min(size_t tla_id) const
{
	return open_lists[tla_id].top();
}

TLAs::QueueEntry TLAs::remove_min(size_t tla_id)
{
	auto res = min(tla_id);
	open_lists[tla_id].pop();
	return res;
}

}
