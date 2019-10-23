#ifndef LOOKAHEAD_CONTROL_H
#define LOOKAHEAD_CONTROL_H

#include <memory>
#include <chrono> // for a more accurate timer
#include "lookhead_search.h"
#include "../utils/timer.h"
#include "../search_engine.h" // for SearchStatus

namespace real_time
{

// I'd rather make this a template, but with the current setup, it's not known at compile time
struct LookaheadBound
{
	virtual ~LookaheadBound() = default;
	virtual bool ok() const = 0;
	virtual void initialize(LookaheadSearch const &ls) = 0;
};

struct LookaheadControl
{
	std::unique_ptr<LookaheadSearch> ls;
	std::unique_ptr<LookaheadBound> lb;

	// TODO: allow in place construction maybe
	LookaheadControl();
	virtual ~LookaheadControl();

	void initialize(GlobalState const &s);
	SearchStatus search();
};

struct ExpansionBound : LookaheadBound
{
	SearchStatistics const *stats;
	int bound;

	ExpansionBound(int b);
	virtual ~ExpansionBound() = default;

	bool ok() const final;
	void initialize(LookaheadSearch const &ls) final;
};

struct RtTimer
{
	std::chrono::time_point<std::chrono::system_clock> start;

	RtTimer();
	~RtTimer();

	std::chrono::duration<int, std::milli> diff() const;
	void reset();
};

struct TimeBound : LookaheadBound
{
	const std::chrono::duration<int, std::milli> max_ms;
	RtTimer timer;

	TimeBound(int max_ms);
	virtual ~TimeBound() = default;

	bool ok() const final;
	void initialize(LookaheadSearch const &ls) final;
};


}

#endif
