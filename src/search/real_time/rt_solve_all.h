#ifndef REAL_TIME_RT_SOLVE_ALL_H
#define REAL_TIME_RT_SOLVE_ALL_H

#include "../open_list.h"
#include "../search_engine.h"
#include "../utils/countdown_timer.h"
#include "real_time_search.h"

#include <memory>
#include <vector>
#include <tuple>
#include <unordered_set>

class Evaluator;
class PruningMethod;

namespace options {
class Options;
}

namespace real_time {

// struct ExpNode
// {
// 	GlobalState s;
// 	SearchNode n;
// 	bool b;
// ExpNode(GlobalState &s, SearchNode &n, bool b) : s(s), n(n), b(b) {}
// 	~ExpNode() {}
// };

class RtSolveAll : public SearchEngine {

	// SearchEngine solver;
	std::unique_ptr<RealTimeSearch> gatherer;

	std::unique_ptr<std::unordered_set<StateID> > expanded_states;
	SearchStatus collect_status;
	std::unique_ptr<StateOpenList> open_list;
	GlobalState initial_state;
	std::unique_ptr<SearchSpace> search_space;
	std::shared_ptr<Evaluator> f_evaluator;
	std::shared_ptr<Evaluator> evaluator;
	const bool reopen_closed_nodes;

	const std::string hstar_file;
	void dump_hstar_values() const;

	const std::string successors_file;
	void dump_succ_values() const;

	std::tuple<GlobalState, SearchNode, bool> fetch_next_node();
	void eval_node(SearchNode &sn, OperatorProxy const &op, SearchNode const &n);
	SearchStatus update_hstar(SearchNode const &node, int hstar);

	std::unordered_map<StateID, std::tuple<int, int, int>> solved_states;
	const bool collect_parent_h;

	std::vector<OperatorID> applicables;

	const double reserved_time;
	utils::CountdownTimer timer;


protected:
	void initialize() final;
	SearchStatus step() final;
public:
	explicit RtSolveAll(const options::Options &opts);
	virtual ~RtSolveAll() = default;
	void print_statistics() const override;
	//void dump_search_space() const;
};


}


#endif
