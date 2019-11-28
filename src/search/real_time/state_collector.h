#ifndef REAL_TIME_STATE_COLLECTOR
#define REAL_TIME_STATE_COLLECTOR

#include <unordered_set>
#include "lookhead_search.h"
#include "../state_id.h"

namespace real_time
{

class EagerCollector : public EagerLookaheadSearch
{
private:
	std::unique_ptr<std::unordered_set<StateID> > expanded_states;
	void mark_expanded(SearchNode &node) final;
	void collect(StateID id);
protected:
	virtual auto create_open_list() const -> std::unique_ptr<StateOpenList> = 0;
public:
	std::unique_ptr<std::unordered_set<StateID> > get_expanded_states() final;
	EagerCollector(StateRegistry &state_registry,
		       bool store_exploration_data,
		       ExpansionDelay *expansion_delay,
		       HeuristicError *heuristic_error,
		       SearchEngine const *search_engine);
	~EagerCollector() {}
};

class AStarCollect : public EagerCollector {
	std::shared_ptr<Evaluator> f_evaluator;
	std::shared_ptr<Evaluator> heuristic;
protected:
	auto create_open_list() const -> std::unique_ptr<StateOpenList> override;
public:
	AStarCollect(StateRegistry &state_registry,
		     std::shared_ptr<Evaluator> heuristic,
		     bool store_exploration_data,
		     ExpansionDelay *expansion_delay,
		     HeuristicError *heuristic_error,
		     SearchEngine const *search_engine);
	~AStarCollect() override = default;

	AStarCollect(const AStarCollect &) = delete;
	AStarCollect(AStarCollect &&) = delete;

	auto operator=(const AStarCollect &) = delete;
	auto operator=(AStarCollect &&) = delete;
};


}


#endif
