#include "max_time.h"

#include <cmath>

#define TRACKSC

#ifdef TRACKSC
#include <iostream>
#define BEGINF(X) std::cout << "MT: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "MT: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "MT: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif

namespace real_time
{

MaxTime::MaxTime(int ms)
	: start(std::chrono::system_clock::now()),
	  max_ms(ms),
	  final_bound(start + max_ms),
	  lookahead_part(95),
	  lookahead_ms(ms * lookahead_part / 100),
	  lookahead_bound(start + lookahead_ms),
	  ls(nullptr),
	  lookahead_ub(1) //assume lookahead always takes at least one ms
{}

bool MaxTime::lookahead_ok()
{
	lookahead_ub = std::max(lookahead_ub, ls->get_duration());
	return (std::chrono::system_clock::now() + lookahead_ub) < lookahead_bound;
}

bool MaxTime::learning_ok() const
{
	return std::chrono::system_clock::now() < final_bound;
}

void MaxTime::initialize(LookaheadSearch const &ls)
{
	BEGINF(__func__);
	this->ls = &ls;
	start = final_bound;
	final_bound = start + max_ms;
	// Note that lookahead_ms might have changed in adjust_learning
	lookahead_bound = start + lookahead_ms;
	ENDF(__func__);
}


// TODO: come up with a better scheme to better schedule lookahead vs
// learning.
// Currently we do the following:
// To increase the time for learning (if it didn't finish):
//   Compute which fraction of the states was covered during learning.
//   Multiply the time for learning by the inverse.  E.g. we did 1/3
//   of the states, we'll ask for 3 times as much time next iteration.
// To decrease the time for learning (if it finished with lots of time
// to spare):
//   If we only used 1/2 of the time we had in learning, give a away
//   1/4 of it.

// The scheme kinda works, but it has some issues:
// - Increasing time by a factor always at least doubles the allocated
//   time (I'm doing integer math).  Seems fine, but it may be too
//   greedy.
// - Decreasing the time doesn't really have a good way to determine
//   how much is safe to give away.
// - Decreasing the time is very conservative and there may be cases
//   where we can give away more time.  Then this scheme won't realize
//   it.  That's bad, because we always want to spend as much time in
//   the lookahead as we can.
// - There is currently no limit, and I may give myself more for
//   learning that I have availbale in total.  That's an extreme case,
//   but it needs to be considered, as it's a case that should be
//   prevented.
void MaxTime::adjust_learning(size_t effort, size_t remaining)
{
	auto done = effort - remaining;
	auto learn_ms = max_ms - lookahead_ms;

	if (0 == done) {
		TRACKP("didn't finish a single entry during learning");
	} else {
		TRACKP("in learning remained " << remaining << " out of " << effort);
	}


	if (0 == remaining) {
		auto n = std::chrono::system_clock::now();
		assert(final_bound > n);
		// this would indicate a massive waste of time in the learning phase.
		if (2 * (final_bound - n) >= learn_ms) {
			auto new_learn_ms = std::max(std::chrono::milliseconds{1},(learn_ms/2)+(learn_ms/4));
			auto new_lookahead_ms = max_ms - new_learn_ms;
			TRACKP("Decreasing the learning ms from " << learn_ms.count() << " to " << new_learn_ms.count() << ".  Increasing lookahead ms from " << lookahead_ms.count() << " to " << new_lookahead_ms.count());
			this->lookahead_ms = new_lookahead_ms;
		}
	} else {
		// (effort / done) rounded up
		auto ratio = 0 != done ? (effort + done - 1) / done : 2;
		assert(ratio > 0);

		decltype(learn_ms) new_learn_ms{learn_ms.count() * ratio};
		auto new_lookahead_ms = max_ms - new_learn_ms;
		TRACKP("Increasing the learning ms from " << learn_ms.count() << " to " << new_learn_ms.count() << " (ratio: " << ratio << ").  Decreasing lookahead ms from " << lookahead_ms.count() << " to " << new_lookahead_ms.count());
		if (new_learn_ms > max_ms) {
			// This basically means we'd need more time for learning than we have available at all.
			// Abandon all hope.
			// new_learn_ms = max_ms;
			assert(false);
		}
		this->lookahead_ms = new_lookahead_ms;
	}
}

void MaxTime::print_surplus() const
{
	// TRACKP("Time left: " << (final_bound > std::chrono::system_clock::now()));
}
}
