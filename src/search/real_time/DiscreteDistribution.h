#pragma once

#include "util.h"
#include "belief_data.h"

#include <cmath>
#include <vector>
#include <algorithm>
#include <cassert>

struct ProbabilityNode
{
	double cost;
	double probability;

	ProbabilityNode();
	ProbabilityNode(double x, double prob);

	bool operator<(const ProbabilityNode& node) const;
	bool operator>(const ProbabilityNode& node) const;
	bool operator==(const ProbabilityNode& node) const;
	bool operator!=(const ProbabilityNode& node) const;
};

struct ProbabilityPair
{
	ProbabilityNode first;
	ProbabilityNode second;
	ProbabilityPair* left;
	ProbabilityPair* right;

	ProbabilityPair(ProbabilityNode lower, ProbabilityNode upper);
};

struct CompareDistance
{
	bool operator()(ProbabilityPair* p1, ProbabilityPair* p2)
	{
		return (p1->second.cost - p1->first.cost) > (p2->second.cost - p2->first.cost);
	}
};

class DiscreteDistribution
{
	std::vector<ProbabilityNode> distribution;
	int maxSamples;

	double probabilityDensityFunction(double x, double mu, double var);

public:

	DiscreteDistribution() {}
	explicit DiscreteDistribution(int maxSamples) : maxSamples(maxSamples) {}
	// Creates a gaussian assumption-based belief
	DiscreteDistribution(int maxSamples, double f, double mean, double d, double error);
	// Creates a spike belief
	DiscreteDistribution(int maxSamples, double d);
	template<typename CountT>
	DiscreteDistribution(int maxSamples, const real_time::HStarEntry<CountT> &hstar_data);
	DiscreteDistribution(double g, double h, bool& retSuccess);
	DiscreteDistribution(DiscreteDistribution const &other);
	DiscreteDistribution(DiscreteDistribution const &other, int shift);
	DiscreteDistribution(DiscreteDistribution const &other, double shift);
	DiscreteDistribution(DiscreteDistribution const *other);
	DiscreteDistribution(DiscreteDistribution const *other, int shift);
	DiscreteDistribution(DiscreteDistribution const *other, double shift);

	static const DiscreteDistribution dead_distribution;

	DiscreteDistribution& squish(double factor);
	double expectedCost() const;

	DiscreteDistribution& operator=(const DiscreteDistribution& rhs);
	// DiscreteDistribution operator*(const DiscreteDistribution& rhs);

	std::vector<ProbabilityNode>::iterator begin();
	std::vector<ProbabilityNode>::iterator end();
	std::vector<ProbabilityNode>::const_iterator begin() const;
	std::vector<ProbabilityNode>::const_iterator end() const;
};


// The belief for a state is a distribution and an offset.  Since the
// distribution itself never changes after its creation, it can be
// shared for all states, and we can just keep a pointer to it.
struct ShiftedDistribution
{
	const DiscreteDistribution *distribution;
	double expected_value;
	int shift;
	ShiftedDistribution() : distribution(nullptr), expected_value(-1), shift(-1) {}
	~ShiftedDistribution() {}

	double expected_cost() const;
	void set(const DiscreteDistribution *distribution, int shift);
	void set_and_shift(const ShiftedDistribution &d, int shift);

private:
	void set(const DiscreteDistribution *distribution, double exp_value, int shift);
};

double variance(DiscreteDistribution const &d);
double variance(ShiftedDistribution const &d);

std::pair<double, double> variance_and_stddev(DiscreteDistribution const &);
std::pair<double, double> variance_and_stddev(ShiftedDistribution const &);

template<typename CountT>
DiscreteDistribution::DiscreteDistribution(
	int maxSamples,
	const real_time::HStarEntry<CountT> &hstar_data
	)
	: maxSamples(maxSamples)
{
	// we have hstar_data with 10_000 samples.  it would be great to use
	// all of it, but that's computationally infeasible and most likely
	// wouldn't change the outcome of the risk_analysis much.  So I just
	// downsample the data very coarsely to get at most maxSamples
	// entries per distribution.  maxSamples is typically set to
	// something like 64, 100, or 128
	auto const &values = hstar_data.hstar_values;
	int const scale = hstar_data.value_count / maxSamples;
	if (scale < 2) {
		for (const auto &[hstar_value, count] : values) {
			distribution.emplace_back(hstar_value, count / static_cast<double>(hstar_data.value_count));
		}
	} else {
		int mass = 0;
		long int hsvalue = 0;
		double totalp = 0.0;
		double const invscale = 1.0 / static_cast<double>(scale);
		double prob;
		for (const auto &[hstar_value, count] : values) {
			hsvalue += hstar_value * count;
			mass += count;
			if (mass >= scale) {
				prob = (mass * invscale) / maxSamples;
				totalp += prob;
				assert(hsvalue / static_cast<double>(mass) >= 0.0);
				distribution.emplace_back(hsvalue / static_cast<double>(mass), prob);
				mass = 0;
				hsvalue = 0;
			}
		}

		// normalize
		double const invtotalp = 1.0 / totalp;
		for (auto &node : distribution) {
			node.probability *= invtotalp;
		}
	}
}
