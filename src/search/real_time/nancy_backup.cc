#include "nancy_backup.h"

// #define TRACKNB

#ifdef TRACKNB
#define BEGINF(X) std::cout << "NB: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "NB: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "NB: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif


namespace real_time
{

NancyBackup::NancyBackup(StateRegistry const &state_registry,
			 SearchEngine const *search_engine,
			 Beliefs *beliefs,
			 Beliefs *post_beliefs)
	: Learning(state_registry, search_engine), beliefs(beliefs), post_beliefs(post_beliefs)
{
}


void NancyBackup::initialize(
	const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors_,
	const std::vector<StateID> &frontier,
	std::unordered_set<StateID> &closed_)
{
	predecessors = &predecessors_;
	closed = &closed_;
	initial_effort = closed->size();

	assert(learning_queue.empty());
	for (const auto &state_id : (*closed)) {
		auto const &state = state_registry.lookup_state(state_id);
		(*beliefs)[state].expected_value = std::numeric_limits<double>::infinity();
	}

	for (const auto &state_id : frontier) {
		auto const &state = state_registry.lookup_state(state_id);
		learning_queue.emplace((*beliefs)[state], state_id);
		closed->erase(state_id);
	}
}



void NancyBackup::step()
{
	assert(!done());

 get_entry:
	if (done())
		return;
	auto &top = learning_queue.top();
	ShiftedDistribution dstr = top.first;
	StateID state_id = top.second;
	learning_queue.pop();
	auto const &state = state_registry.lookup_state(state_id);
	assert((*beliefs)[state].distribution != nullptr);

	if (dstr.expected_value == std::numeric_limits<double>::infinity())
		goto get_entry;

	auto cls = closed->find(state_id);

	if (cls != closed->end())
		closed->erase(cls);

	auto preds = predecessors->find(state_id);
	if (preds == predecessors->end())
		return; // return to make the step not so long

	for (auto const &[p_id, op] : preds->second) {
		auto cls = closed->find(p_id);
		if (cls == closed->end())
			continue;

		auto const &predecessor = state_registry.lookup_state(p_id);
		auto const new_exp = dstr.expected_cost() + search_engine->get_adjusted_cost(op);
		ShiftedDistribution &p_belief = (*beliefs)[predecessor];
		auto const p_exp = p_belief.expected_cost();
		if (p_exp > new_exp) {
			// to be clear here:
			// - the belief is only backed up if its expected value increased
			//   compared to before.
			// - the new belief that's stored for the predecessor is
			//   1. the same raw distribution as the current state.
			//   2. a shift given by op cost + the shift for the current state.
			// - the new expected value for the predecessor will be the
			//   current state's expected value + the operator cost which is
			//   the same as the expected value of the raw distribution + shift

			// backup the main belief
			p_belief.set_and_shift(dstr, search_engine->get_adjusted_cost(op));
			assert(std::abs(new_exp - p_belief.expected_cost()) < 0.001);

			// backup the post expansion belief
			ShiftedDistribution &p_post_belief = (*post_beliefs)[predecessor];
			ShiftedDistribution &s_post_belief = (*post_beliefs)[state];
			assert(dstr.shift == s_post_belief.shift);
			assert(s_post_belief.distribution);
			p_post_belief.set_and_shift(s_post_belief, search_engine->get_adjusted_cost(op));

			learning_queue.emplace(p_belief, p_id);
		}
	}
}


bool NancyBackup::done()
{
	return learning_queue.empty();
}

size_t NancyBackup::effort()
{
	return initial_effort;
}

size_t NancyBackup::remaining()
{
	assert(closed != nullptr);
	return closed->size();
}

}

#undef TRACKNB
#undef BEGINF
#undef ENDF
#undef TRACKP
