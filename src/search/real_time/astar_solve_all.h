#ifndef REAL_TIME_ASTAR_SOLVE_ALL_H
#define REAL_TIME_ASTAR_SOLVE_ALL_H

#include "../open_list.h"
#include "../search_engine.h"
#include "../utils/countdown_timer.h"

#include <memory>
#include <vector>
#include <tuple>

class Evaluator;
class PruningMethod;

namespace options {
class Options;
}

namespace astar_solve_all {
class AStarSolveAll : public SearchEngine {
    const bool reopen_closed_nodes;

    std::unique_ptr<StateOpenList> open_list;
	GlobalState current_initial_state;
	std::unique_ptr<SearchSpace> current_search_space;
    std::shared_ptr<Evaluator> f_evaluator;

	std::shared_ptr<Evaluator> evaluator;
	const int weight;
    std::vector<Evaluator *> path_dependent_evaluators;
    std::vector<std::shared_ptr<Evaluator> > preferred_operator_evaluators;

    std::shared_ptr<PruningMethod> pruning_method;

	const std::string hstar_file;
	void dump_hstar_values() const;

	const std::string successors_file;
	void compute_and_dump_successors_data();

	std::unordered_set<StateID> expanded_states;
	std::unordered_map<StateID, std::tuple<int, int, int>> solved_states;

	const bool find_early_solutions;
	const bool collect_parent_h;
	StateID best_solution_state;
	int best_solution_cost;

	bool computing_initial_solution;
	Plan initial_plan;

	const double reserved_time;
	utils::CountdownTimer timer;

	auto update_hstar_from_state(const SearchNode &node, int hstar) -> SearchStatus;

    std::pair<SearchNode, bool> fetch_next_node();
    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(const SearchNode &node);
    void reward_progress();
    void print_checkpoint_line(int g) const;

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    explicit AStarSolveAll(const options::Options &opts);
    virtual ~AStarSolveAll() = default;

    virtual void print_statistics() const override;

    void dump_search_space() const;
};
}

#endif
