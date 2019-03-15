#include "expansion_delay.h"

#include "../search_statistics.h"

#include <cassert>

namespace real_time {

ExpansionDelay::ExpansionDelay(std::size_t moving_average_size)
	: avg_expansion_delay(0),
	  total_expansions(0),
	  last_delays_sum(0),
	  moving_average_size(moving_average_size) {}

void ExpansionDelay::update_expansion_delay(int delay) {
	assert(delay >= 1);
	++total_expansions;
	avg_expansion_delay += (delay - avg_expansion_delay) / total_expansions;
	if (moving_average_size != 0u) {
		last_delays.push_back(delay);
		last_delays_sum += delay;
		if (last_delays.size() > moving_average_size) {
			last_delays_sum -= last_delays.front();
			last_delays.pop_front();
		}
	}
}

auto ExpansionDelay::get_avg_expansion_delay() const -> double {
	if (total_expansions < 2)
		return 1.;
	return moving_average_size == 0u ? avg_expansion_delay : last_delays_sum / static_cast<double>(last_delays.size());
}

}
