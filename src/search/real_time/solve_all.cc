#include "solve_all.h"

#include <cmath>
#include "../options/plugin.h"
#include "../evaluators/g_evaluator.h"
#include "../open_lists/best_first_open_list.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../operator_id.h"
#include <fstream>

// this feels morally wrong, but it saves the unnecessary function
// call if we assume we're exclusively running unit cost
#define get_adjusted_cost(X) 1

namespace solve_complete
{

SolveAll::SolveAll(Options const &opts)
  : SearchEngine(opts),
    open_list(create_open_list()),
    reopen_closed_nodes(opts.get<bool>("reopen_closed")),
    evaluator(opts.get<std::shared_ptr<Evaluator>>("eval")),
    initial_id(state_registry.get_initial_state().get_id()),
    planning(true),
    reserved_time(opts.get<double>("reserved_time")),
    timer(std::numeric_limits<double>::infinity()),
    hstar_file(opts.get<std::string>("hstar_file")),
    successors_file(opts.get<std::string>("successors_file"))
{
}

std::unique_ptr<StateOpenList> SolveAll::create_open_list()
{
	auto options = options::Options();
	options.set<std::shared_ptr<Evaluator> >("eval", std::make_shared<g_evaluator::GEvaluator>());
	options.set("pref_only", false);
  options.set("unsafe_pruning", false);
	return std::make_unique<standard_scalar_open_list::BestFirstOpenListFactory>(options)->create_state_open_list();
}

void SolveAll::initialize()
{
  std::cout << "Conducting best first search"
            << (reopen_closed_nodes ? " with" : " without")
            << " reopening closed nodes, (real) bound = " << bound
            << "\n";
  assert(open_list);

  std::vector<OperatorID> apps;

  const GlobalState &initial_state = state_registry.get_initial_state();
  initial_id = initial_state.get_id();
  EvaluationContext eval_context(initial_state, 0, true, &statistics);
  hs[initial_id] = eval_context.get_evaluator_value(evaluator.get());
  statistics.inc_evaluated_states();

  if (open_list->is_dead_end(eval_context)) {
    std::cout << "Initial state is a dead end.\n";
  } else {
    if (search_progress.check_progress(eval_context))
      print_checkpoint_line(0);
    SearchNode node = search_space.get_node(initial_state);
    node.open_initial();

    open_list->insert(eval_context, initial_state.get_id());
  }

  print_initial_evaluator_values(eval_context);

  //pruning_method->initialize(task);

	if (!std::isinf(max_time)) {
		assert(max_time - reserved_time > 0);
		timer = utils::CountdownTimer(max_time - reserved_time);
	}

  int g_level = 0;

  while (!open_list->empty()) {
    StateID cur_id = open_list->remove_min();
    GlobalState s = state_registry.lookup_state(cur_id);
    SearchNode node = search_space.get_node(s);

    if (node.is_closed())
      continue;

    if (task_properties::is_goal_state(task_proxy, s)) {
      goals.push_back(cur_id);
      statistics.inc_expanded();
      node.close();
      expanded_states.insert(cur_id);
      continue;
    }

    statistics.inc_expanded();
    node.close();

    int cur_g = node.get_g();
    if (cur_g > g_level) {
      print_checkpoint_line(cur_g);
      g_level = cur_g;
    }

    apps.clear();
    successor_generator.generate_applicable_ops(s, apps);
    statistics.inc_expanded();
    if (apps.size() > 0)
      expanded_states.insert(cur_id);

    for (OperatorID op_id : apps) {
      OperatorProxy op = task_proxy.get_operators()[op_id];
      GlobalState succ = state_registry.get_successor_state(s, op);
      StateID succ_id = succ.get_id();
      SearchNode succ_node = search_space.get_node(succ);

      if (succ_node.is_dead_end()) {
        continue;
      }

      // to make sure the h* values propagate back to all
      // predecessors, we need to add this state as a predecessor even
      // if it is not the cheapest way to reach the state succ.
      preds[succ_id].emplace_back(cur_id, op);

      if (expanded_states.find(succ_id) != expanded_states.end()) {
        continue;
      }

      int const adjcost = get_adjusted_cost(op);
      assert(1 == adjcost);
      int const new_g = cur_g + adjcost;
      statistics.inc_generated();
      auto eval_context = EvaluationContext(succ, new_g, false, nullptr, false);
      statistics.inc_evaluated_states();

      if (succ_node.is_new()) {
        bool is_dead_end = eval_context.is_evaluator_value_infinite(evaluator.get());
        if (is_dead_end) {
          succ_node.mark_as_dead_end();
          statistics.inc_dead_ends();
          continue;
        }
        succ_node.open(node, op, adjcost);
        int h = eval_context.get_evaluator_value(evaluator.get());
        hs[succ_id] = h;
        open_list->insert(eval_context, succ.get_id());
      } else if (succ_node.get_g() > new_g) {
        // int h = eval_context.get_evaluator_value(evaluator.get());
        // hs[succ_id] = h;
        assert(hs.find(succ_id) != hs.end());
        if (succ_node.is_closed())
					statistics.inc_reopened();
				succ_node.reopen(node, op, adjcost);
        open_list->insert(eval_context, succ.get_id());
      }
    }
  }

  std::cout << "found " << goals.size() << " goal states. propagating hstar values\n";

  for (StateID const goal : goals) {
    d_queue.push(std::make_pair(goal, 0));
    hstars[goal] = 0;
  }

  hstars.reserve(hs.size());
  while (!d_queue.empty()) {
    auto const front = d_queue.front();
    auto const cur_s = front.first;
    int const hstar = front.second;
    d_queue.pop();
    auto const pre_hstar = hstars.find(cur_s);
    if (pre_hstar != hstars.end() && pre_hstar->second < hstar)
      continue;
    assert(hs.find(cur_s) != hs.end());
    const auto &predecessors = preds[cur_s];
    for (const auto &[parent, op] : predecessors) {
      int const new_hstar = hstar + get_adjusted_cost(op);
      auto const it = hstars.find(parent);
      if (it == hstars.end() || new_hstar < it->second) {
        hstars[parent] = new_hstar;
        best_succ.emplace(parent, std::make_pair(cur_s, op));
        d_queue.push(std::make_pair(parent, new_hstar));
      }
    }
  }

  std::cout << "computed all hstar values\n";
}

void SolveAll::print_statistics() const {
  statistics.print_detailed_statistics();
  search_space.print_statistics();
}

void SolveAll::print_checkpoint_line(int g) const {
  std::cout << "[g=" << g << ", ";
  statistics.print_basic_statistics();
  std::cout << "]\n";
}

SearchStatus SolveAll::step()
{
  if (goals.size() == 0) {
    return FAILED;
  }

  // make the plan
  {
    std::cout << "reconstructing an optimal plan\n";
    StateID cur_id = initial_id;
    while (1) {
      auto it = hstars.find(cur_id);
      if (it == hstars.end()) {
        std::cerr  << "unknown hstar value when tracing plan\n";
        return FAILED;
      }
      if (it->second == 0) {
        break;
      }
      auto succ = best_succ.find(cur_id);
      if (succ == best_succ.end()) {
        std::cerr << "unknown successor for state when tracing plan\n";
        return FAILED;
      }
      initial_plan.push_back(OperatorID(succ->second.second.get_id()));
      cur_id = succ->second.first;
    }
    set_plan(initial_plan);
  }

  // dump hstar values
  {
    std::cout << "dumping h* values\n";
    auto out = std::ofstream(hstar_file);
    for (const StateID sid : expanded_states) {
      auto h_it = hs.find(sid);
      if (h_it == hs.end()) {
        std::cerr << "unknown h value for state\n";
        continue;
      }
      int h = h_it->second;
      auto hstar_it = hstars.find(sid);
      if (hstar_it == hstars.end()) {
        //std::cerr << "unknown hstar value for state\n";
        continue;
      }
      int hstar = hstar_it->second;
      out << h << " " << hstar << '\n';
    }
    out.close();
    std::cout << "dumped h* values to " << hstar_file << "\n";
  }

  // dump successor values
  {
    std::cout << "dumping successor h* values\n";
    auto out = std::ofstream(successors_file);
    std::vector<OperatorID> apps;
    auto const &ops = task_proxy.get_operators();
    for (auto const &s : expanded_states) {
      int h = hs[s];
      // int hstar = hstars[s];
      auto state = state_registry.lookup_state(s);
      apps.clear();
      successor_generator.generate_applicable_ops(state, apps);
      out << h;
      for (auto const op_id : apps) {
        auto const &op = ops[op_id];
        auto const &succ = state_registry.get_successor_state(state, op);
        out << " " << get_adjusted_cost(op) << " " << hs[succ.get_id()];
      }
      out << "\n";
    }
    out.close();
    std::cout << "dumped successor h* values to " << successors_file << "\n";
  }

  return SOLVED;
}

static std::shared_ptr<SearchEngine> _parse(options::OptionParser &parser)
{
	// parser.document_synopsis(
	// 	"A* search (eager)",
	// 	"A* is a special case of eager best first search that uses g+h "
	// 	"as f-function. "
	// 	"We break ties using the evaluator. Closed nodes are re-opened.");
	// parser.document_note(
	// 	"Equivalent statements using general eager search",
	// 	"\n```\n--search astar(evaluator)\n```\n"
	// 	"is equivalent to\n"
	// 	"```\n--evaluator h=evaluator\n"
	// 	"--search eager(tiebreaking([sum([g(), h]), h], unsafe_pruning=false),\n"
	// 	"               reopen_closed=true, f_eval=sum([g(), h]))\n"
	// 	"```\n", true);
	parser.add_option<std::shared_ptr<Evaluator>>("eval", "evaluator for h-value");
	parser.add_option<std::string>("hstar_file", "file name to dump h* values", "hstar_values.txt");
	parser.add_option<std::string>("successors_file", "file name to dump post-expansion data", "successors_data.txt");
	parser.add_option<double>("reserved_time", "reserved time to dump data", "0", Bounds("0", ""));

	SearchEngine::add_pruning_option(parser);
	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();

	std::shared_ptr<SolveAll> engine;
	if (!parser.dry_run()) {
		auto g = std::make_shared<g_evaluator::GEvaluator>();
		auto h = opts.get<std::shared_ptr<Evaluator>>("eval");
		opts.set<std::shared_ptr<Evaluator>>("eval", h);
		opts.set("reopen_closed", true);
		engine = std::make_shared<SolveAll>(opts);
	}

	return engine;
}

static options::Plugin<SearchEngine> _plugin("solve_hstar", _parse);

}
