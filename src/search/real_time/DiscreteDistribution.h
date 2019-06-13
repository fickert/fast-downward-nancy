#pragma once

#include "util.h"

#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <map>
#include <set>

using namespace std;

# define M_PI  3.14159265358979323846  /* pi */

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
	set<ProbabilityNode> distribution;
	int maxSamples;

	static unordered_map<double, vector<ProbabilityNode>> hValueTable;

  double probabilityDensityFunction(double x, double mu, double var);
	void resize(map<double, double>& distroMap);


public:

	DiscreteDistribution() {}
	explicit DiscreteDistribution(int maxSamples) : maxSamples(maxSamples) {}
	DiscreteDistribution(int maxSamples, double f, double mean, double d, double error);
	// Creates a discrete distribution based on Pemberton's belief distribution, a uniform between 0 and 1, offset by some g-value
	DiscreteDistribution(int maxSamples, double g, double d);
	// Creates a discrete distribution based on Pemberton's belief distribution, a uniform between 0 and 1, offset by some g-value
	DiscreteDistribution(int maxSamples, double g, double d, int bf);
	// Creates a delta spike belief
	DiscreteDistribution(int maxSamples, double deltaSpikeValue);
	template<class count_type>
	DiscreteDistribution(int maxSamples, const real_time::hstar_data_entry<count_type> &hstar_data);
  DiscreteDistribution(double g, double h, bool& retSuccess);
  DiscreteDistribution(DiscreteDistribution const *other);

	void createFromUniform(int maxSamples, double g, double d);
	void createFromGaussian(double f, double mean, double d, double error);
	DiscreteDistribution& squish(double factor);
	double expectedCost() const;

	DiscreteDistribution& operator=(const DiscreteDistribution& rhs);
	DiscreteDistribution operator*(const DiscreteDistribution& rhs);

	set<ProbabilityNode>::iterator begin();
	set<ProbabilityNode>::iterator end();
	set<ProbabilityNode>::const_iterator begin() const;
	set<ProbabilityNode>::const_iterator end() const;
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

template<class count_type>
DiscreteDistribution::DiscreteDistribution(int maxSamples, const real_time::hstar_data_entry<count_type> &hstar_data)
  : maxSamples(maxSamples)
{
  for (const auto &[hstar_value, count] : hstar_data.hstar_values)
    distribution.emplace(hstar_value, count / static_cast<double>(hstar_data.value_count));
}
