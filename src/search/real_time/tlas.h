#ifndef REAL_TIME_TLAS_H
#define REAL_TIME_TLAS_H

// the struct for tlas.  Used by lookahead methods that explicitly
// reason about the top level actions as such, and do lookahead by
// searching under a specific tla (and need a separate open list for
// each of them).

#include <queue>
#include <vector>

#include "../global_state.h"
#include "../operator_id.h"
#include "../evaluation_context.h"
#include "DiscreteDistribution.h"

namespace real_time
{

// This struct is used in the open list to sort nodes by expected
// value, with h as tie breaker.
struct NodeEvaluation
{
	double expected; // g + h_hat
	int g;
	int h;
	NodeEvaluation(double h_hat, int g, int h)
		: expected(h_hat + static_cast<double>(g)), g(g), h(h) {}
	~NodeEvaluation() {}
};

struct TLAs
{
	using QueueEntry = std::pair<NodeEvaluation, StateID>;
	using QueueEntryCompare =
		struct {
			bool operator() (QueueEntry const &l, QueueEntry const &r) const
			{
			 if (l.first.expected != r.first.expected) {
				return l.first.expected > r.first.expected;
			 } else if (l.first.h != r.first.h) {
				return l.first.h > r.first.h;
			 } else {
				 return l.second.hash() < r.second.hash();
			 }
			}
		};
	using Queue =
		std::priority_queue<QueueEntry,
				    std::vector<QueueEntry>,
				    QueueEntryCompare>;
	std::vector<OperatorID> ops;
	std::vector<int> op_costs;
	std::vector<Queue> open_lists;
	// backed up belief for each tla
	std::vector<ShiftedDistribution> beliefs;
	std::vector<ShiftedDistribution> post_beliefs;
	// "states" stores the current best state and h_hat in the frontier for each tla.
	// h_hat is used later in the decision strategy, but I don't need the entire distribution there.
	std::vector<std::pair<StateID, double> > states;
	std::vector<EvaluationContext> eval_contexts;

	GlobalState const *current_state;

	void reserve(std::size_t n);
	void clear();
	size_t size() const;
	QueueEntry remove_min(size_t tla_id);
	QueueEntry const &min(size_t tla_id) const;
};


}

#endif
