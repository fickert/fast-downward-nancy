#include "DiscreteDistribution.h"

const DiscreteDistribution DiscreteDistribution::dead_distribution{1, std::numeric_limits<double>::infinity()};

ProbabilityNode::ProbabilityNode()
{
}

ProbabilityNode::ProbabilityNode(double x, double prob)
	: cost(x), probability(prob)
{
}

bool ProbabilityNode::operator<(const ProbabilityNode& node) const
{
	return this->cost < node.cost;
}

bool ProbabilityNode::operator>(const ProbabilityNode& node) const
{
	return this->cost > node.cost;
}

bool ProbabilityNode::operator==(const ProbabilityNode& node) const
{
	return (this->cost == node.cost) && (this->probability == node.probability);
}

bool ProbabilityNode::operator!=(const ProbabilityNode& node) const
{
	return !(*this == node);
}

ProbabilityPair::ProbabilityPair(ProbabilityNode lower, ProbabilityNode upper)
	: first(lower), second(upper), left(NULL), right(NULL)
{
}

double DiscreteDistribution::probabilityDensityFunction(double x, double mu, double var)
{
	return ((1 / std::sqrt(2 * M_PI * var)) * std::exp(-(std::pow(x - mu, 2) / (2 * var))));
}

DiscreteDistribution::DiscreteDistribution(int maxSamples, double f, double mean, double d, double error)
	: maxSamples(maxSamples)
{
	// This is a goal node, belief is a spike at true value
	if (d == 0) {
		distribution.emplace_back(mean, 1.0);
		return;
	}

	if (error < 0.0) {
		error = 0.0;
	}

	double stdDev = error / 2.0;
	auto var = std::pow(stdDev, 2);

	// Create a Discrete Distribution from a gaussian
	double lower = f;
	double upper = mean + 3 * stdDev;
	if (lower > upper)
		std::swap(lower, upper);

	double sampleStepSize = (upper - lower) / static_cast<double>(maxSamples);
	double currentX = lower;
	double probSum = 0.0;

	// Take the samples and build the discrete distribution
	for (int i = 0; i < maxSamples; i++) {
		double prob = probabilityDensityFunction(currentX, mean, var);
		if (std::isnan(prob) && stdDev == 0)
			prob = 1.0;

		distribution.emplace_back(currentX, prob);

		probSum += prob;
		currentX += sampleStepSize;
	}

	// Normalize the distribution probabilities
	if (probSum > 0.0) {
		for (auto& n : distribution) {
			if (n.probability < 1.0)
				n.probability /= probSum;
		}
	}
}

// Creates a delta spike belief
DiscreteDistribution::DiscreteDistribution(int maxSamples, double d)
	: maxSamples(maxSamples)
{
	distribution.emplace_back(d, 1.0);
}

DiscreteDistribution::DiscreteDistribution(DiscreteDistribution const &other)
	:maxSamples(other.maxSamples)
{
	for (ProbabilityNode const &n : other.distribution) {
		distribution.emplace_back(n.cost, n.probability);
	}
}

DiscreteDistribution::DiscreteDistribution(DiscreteDistribution const &other, int shift)
	:DiscreteDistribution(other, static_cast<double>(shift))
{
}

DiscreteDistribution::DiscreteDistribution(DiscreteDistribution const &other, double shift)
	:maxSamples(other.maxSamples)
{
	for (ProbabilityNode const &n : other.distribution) {
		distribution.emplace_back(n.cost + shift, n.probability);
	}
}

DiscreteDistribution::DiscreteDistribution(DiscreteDistribution const *other)
	:DiscreteDistribution(*other)
{
}

DiscreteDistribution::DiscreteDistribution(DiscreteDistribution const *other, int shift)
	: DiscreteDistribution(*other, shift)
{
}

DiscreteDistribution::DiscreteDistribution(DiscreteDistribution const *other, double shift)
	: DiscreteDistribution(*other, shift)
{
}

DiscreteDistribution& DiscreteDistribution::squish(double f)
{
	const double mean = expectedCost();

	if (f == 1.0)
		distribution = std::vector<ProbabilityNode>{ProbabilityNode(mean, 1.0)};
	else
		std::for_each(distribution.begin(), distribution.end(),
			      [mean,f](auto &n) { n.cost += f * (mean - n.cost); });

	return *this;
}

double DiscreteDistribution::expectedCost() const
{
	double E = 0.0;

	for (auto const &n : distribution) {
		E += n.cost * n.probability;
	}

	return E;
}

double variance(DiscreteDistribution const &d)
{
	// Var[x] = E[X^2] - E^2[X]
	double e = d.expectedCost();
	double e2x = e * e;
	double ex2 = 0.0;
	for (auto const &n : d) {
		ex2 += n.cost * n.cost * n.probability;
	}
	return ex2 - e2x;
}

double variance(ShiftedDistribution const &d)
{
	// Var[x] = E[X^2] - E^2[X]
	double e = d.expected_cost();
	double e2x = e * e;
	double ex2 = 0.0;
	for (auto const &n : *d.distribution) {
		double sh_cost = n.cost + d.shift;
		ex2 += (sh_cost * sh_cost) * n.probability;
	}
	return ex2 - e2x;
}


std::pair<double, double> variance_and_stddev(DiscreteDistribution const &d)
{
	double v = variance(d);
	return std::make_pair(v, std::sqrt(v));
}

std::pair<double, double> variance_and_stddev(ShiftedDistribution const &d)
{
	double v = variance(d);
	return std::make_pair(v, std::sqrt(v));
}

DiscreteDistribution& DiscreteDistribution::operator=(const DiscreteDistribution& rhs)
{
	if (&rhs == this) {
		return *this;
	}

	distribution.clear();

	distribution = rhs.distribution;
	maxSamples = rhs.maxSamples;

	return *this;
}

std::vector<ProbabilityNode>::iterator DiscreteDistribution::begin()
{
	return distribution.begin();
}

std::vector<ProbabilityNode>::iterator DiscreteDistribution::end()
{
	return distribution.end();
}


std::vector<ProbabilityNode>::const_iterator DiscreteDistribution::begin() const
{
	return distribution.begin();
}

std::vector<ProbabilityNode>::const_iterator DiscreteDistribution::end() const
{
	return distribution.end();
}

double ShiftedDistribution::expected_cost() const
{
	//assert(expected_value >= 0);
	return expected_value;
}

void ShiftedDistribution::set(DiscreteDistribution const *d, int s)
{
	this->distribution = d;
	this->shift = s;
	// Always remember, the expected value is linear.
	this->expected_value = d->expectedCost() + this->shift;
}

void ShiftedDistribution::set(const DiscreteDistribution *d, double e, int s)
{
	this->distribution = d;
	this->expected_value = e;
	this->shift = s;
	assert(std::abs((d->expectedCost() + s) - e) < 0.00001);
}

void ShiftedDistribution::set_and_shift(const ShiftedDistribution &d, int shift)
{
	set(d.distribution, d.expected_value + shift, d.shift + shift);
}
