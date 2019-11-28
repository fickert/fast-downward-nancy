#include "real_time_search.h"

#include "../plugin.h"
#include "../utils/system.h"
#include "util.h"
#include <algorithm>
#include <numeric> // for accumulate

#include <iostream>

// #define TRACKRT

#ifdef TRACKRT
#define BEGINF(X) std::cout << "RT: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "RT: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "RT: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif

namespace real_time
{

auto RealTimeSearch::get_expanded_states() -> std::unique_ptr<std::unordered_set<StateID> >
{
	return std::move(sc.ls->get_expanded_states());
}

void RealTimeSearch::print_statistics() const {
	statistics.print_detailed_statistics();
	std::cout << "Number of lookahead phases: " << num_rts_phases << std::endl;
	std::cout << "Total solution cost: " << solution_cost << std::endl;
	if (heuristic_error) {
		std::cout << "Average heuristic error: " << heuristic_error->get_average_heuristic_error() << std::endl;
		if (distance_heuristic)
			std::cout << "Average distance error: " << heuristic_error->get_average_distance_error() << std::endl;
	}
	if (hstar_data) {
		// auto num_entries = hstar_data->data.size();
		// std::vector<size_t> v;
		// v.reserve(num_entries);
		// std::cout << "number of samples:\n";
		// for (const auto a : hstar_data->data) {
		// 	size_t tmp = 0;
		// 	for (auto b : a.values) {
		// 		tmp += b.hstar_values.size();
		// 	}
		// 	v.push_back(tmp);
		// 	std::cout << tmp << '\n';
		// }
		// std::sort(v.begin(), v.end());
		// auto avg = std::accumulate(v.begin(), v.end(), 0, [](auto a, auto b) {return a + b;}) / num_entries;
		// auto min = v[0];
		// auto max = v[v.size()-1];
		// auto med = ((v.size() & 1) == 0) ? (v[v.size()/2-1]+v[v.size()/2])/2:v[v.size()/2];
		// std::cout << "Average number of samples: " << avg << "\n"
		// 	  << "Minimum number of samples: " << min << "\n"
		// 	  << "Maximum number of samples: " << max << "\n"
		// 	  << "Median number of samples: " << med << "\n";
	}
	std::cout << "Fallback to gaussian (decision strategy): " << gaussian_fallback_count << std::endl;
	// ugly hack because the method is const, but i just want to
	// sort some values first.
	(const_cast<RealTimeSearch*>(this))->sc.prepare_statistics();
	sc.print_statistics();
}

static auto _parse(options::OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.document_synopsis("Lazy enforced hill-climbing", "");
	parser.add_option<std::shared_ptr<Evaluator>>("h", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance_heuristic", "distance heuristic", options::OptionParser::NONE);
	parser.add_option<int>("lookahead_bound","Lookahead bound in number of expansions.", "100");
	// TODO: check if the default value here makes any sense.
	parser.add_option<int>("time_bound","Lookahead bound in milli seconds.", "200");
	parser.add_enum_option("rtbound_type", {"EXPANSIONS", "TIME"}, "Type of bound the algorithm is running under", "EXPANSIONS");
	parser.add_enum_option("lookahead_search", {"A_STAR", "A_STAR_COLLECT", "F_HAT", "BREADTH_FIRST", "RISK", "ONLINE_RISK"}, "Lookahead search algorithm", "A_STAR");
	parser.add_enum_option("learning", {"NONE","DIJKSTRA","NANCY"}, "What kind of learning update to perform (DIJKSTRA for heuristic values, NANCY for beliefs)", "NONE");
	parser.add_enum_option("decision_strategy", {"MINIMIN", "BELLMAN", "NANCY", "ONLINE_NANCY", "CSERNA", "K_BEST"}, "Top-level action selection strategy", "MINIMIN");
	parser.add_enum_option("feature_kind", {"JUST_H", "WITH_PARENT_H"}, "Kind of features to look up the beliefs in the data (the data format has to match)", "JUST_H");
	parser.add_enum_option("post_feature_kind", {"JUST_H", "WITH_PARENT_H"}, "Kind of features to look up the post beliefs in the data (the data format has to match)", "JUST_H");
	// parser.add_option<int>("k", "Value for k-best decision strategy", "3");
	parser.add_option<int>("expansion_delay_window_size", "Sliding average window size used for the computation of expansion delays (set this to 0 to use the global average)", "0", options::Bounds("0", ""));
	parser.add_option<std::string>("hstar_data", "file containing h* data", options::OptionParser::NONE);
	parser.add_option<std::string>("post_expansion_belief_data", "file containing post-expansion belief data", options::OptionParser::NONE);

	SearchEngine::add_options_to_parser(parser);
	const auto opts = parser.parse();

	if (parser.dry_run())
		return nullptr;
	return std::make_shared<RealTimeSearch>(opts);
}

static Plugin<SearchEngine> _plugin("real_time", _parse);
}


#undef BEGINF
#undef ENDF
