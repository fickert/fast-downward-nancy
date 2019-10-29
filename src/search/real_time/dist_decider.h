#ifndef REAL_TIME_DIST_DECISION_H
#define REAL_TIME_DIST_DECISION_H

#include "decision.h"

#include "tlas.h"

namespace real_time
{

// not a fan of this name
struct DistributionDecider : public Decision
{
	const TLAs *tlas;
	SearchEngine const &engine; // need this just so I can call get_adjusted_cost

	double target_h_hat;
	double target_f_hat;
	std::vector<OperatorID> target_path;
	StateID target_state_id;

	DistributionDecider(StateRegistry const &state_registry,
			    TLAs const *tlas,
			    SearchEngine const &engine,
			    StateID dummy_id);
	virtual ~DistributionDecider() = default;

	OperatorID decide(std::vector<StateID> const &frontier, SearchSpace &ss) final;

};

}

#endif
