#include "DiscreteDistribution.h"

unordered_map<double, vector<ProbabilityNode>> DiscreteDistribution::hValueTable;

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


bool DiscreteDistribution::is_sorted() const
{
  for (size_t i = 1; i < distribution.size(); ++i) {
    if (distribution[i].cost < distribution[i-1].cost) {
      return false;
    }
  }
  return true;
}

void DiscreteDistribution::resize(map<double, double>& distroMap)
{
  // Maybe we don't need to merge any buckets...
  if (static_cast<int>(distroMap.size()) <= maxSamples) {
    return;
  }

  // Gotta merge some buckets...
  priority_queue<ProbabilityPair*, vector<ProbabilityPair*>, CompareDistance> heap;

  // Groups probabilities into adjacent pairs and does some pointer assignment for tracking merges

  map<double, double>::iterator it = distroMap.begin();
  if (it == distroMap.end()) {
    return;
  }
  ProbabilityNode lastNode(it->first, it->second);
  it++;
  int cnt = 1;
  ProbabilityPair* lastPair = NULL;
  for (; it != distroMap.end(); it++) {
    ProbabilityNode n(it->first, it->second);
    ProbabilityPair* p = new ProbabilityPair(lastNode, n);
    heap.push(p);

    p->left = lastPair;
    p->right = NULL;
    if (lastPair)
      lastPair->right = p;
    lastPair = p;
    lastNode = n;
    cnt++;
  }

  // Now, while we still have too many samples, and the heap isn't empty, merge buckets
  while (static_cast<int>(distroMap.size()) > maxSamples && !heap.empty()) {
    // Get the pair with the lowest distance between buckets
    ProbabilityPair* merge = heap.top();
    heap.pop();

    // Calculate the new probability and X of the merged bucket
    double newProb = merge->first.probability + merge->second.probability;
    double newX = (merge->first.probability / newProb) * merge->first.cost + (merge->second.probability / newProb) * merge->second.cost;

    ProbabilityNode newNode(newX, newProb);

    // Either add this probability to the existing bucket or make a new bucket for it
    distroMap[newX] += newProb;

    // Remove the old probabilities
    distroMap.erase(merge->first.cost);
    distroMap.erase(merge->second.cost);

    // If merge has a pair on its left, update it
    if (merge->left) {
      merge->left->second = newNode;
      merge->left->right = merge->right;
    }
    // If merge has a pair on its right, update it
    if (merge->right) {
      merge->right->first = newNode;
      merge->right->left = merge->left;
    }

    // Delete the merged pair
    delete merge;
  }

  // Delete everything on the heap
  while (!heap.empty()) {
    ProbabilityPair* p = heap.top();
    heap.pop();
    delete p;
  }

  // If we still have too many samples, do it again
  if (static_cast<int>(distroMap.size()) > maxSamples)
    resize(distroMap);
}




DiscreteDistribution::DiscreteDistribution(int maxSamples, double f, double mean, double d, double error)
  : maxSamples(maxSamples)
{
  // std::cout << "fallback distribution constructor\n";
  // This is a goal node, belief is a spike at true value
  if (d == 0) {
    distribution.push_back(ProbabilityNode(mean, 1.0));
    return;
  }

  double stdDev = error / 2.0;
  auto var = std::pow(stdDev, 2);

  // Create a Discrete Distribution from a gaussian
  double lower = f;
  double upper = mean + 3 * stdDev;
  if (lower > upper)
    std::swap(lower, upper);

  double sampleStepSize = (upper - lower) / maxSamples;

  double currentX = lower;

  //std::cout << f << ", " << mean << ", " << stdDev << ", " << var << ", " << lower << ", " << upper << "\n";

  double probSum = 0.0;

  // Take the samples and build the discrete distribution
  for (int i = 0; i < maxSamples; i++) {
    // Get the probability for this x value
    double prob = probabilityDensityFunction(currentX, mean, var);

    // So if this a goal node, we know the cost
    if (std::isnan(prob) && stdDev == 0)
      prob = 1.0;

    probSum += prob;

    ProbabilityNode node(currentX, prob);

    distribution.push_back(node);

    currentX += sampleStepSize;
  }

  // Normalize the distribution probabilities
  if (probSum > 0.0) {
    for (ProbabilityNode& n : distribution) {
      if (n.probability < 1.0)
        n.probability = n.probability / probSum;
    }
  }

  //assert(is_sorted());
}

// Creates a discrete distribution based on Pemberton's belief distribution, a uniform between 0 and 1, offset by some g-value
DiscreteDistribution::DiscreteDistribution(int maxSamples, double g, double d)
  : maxSamples(maxSamples)
{
  // This is a goal node, belief is a spike at true value
  if (d == 0) {
    distribution.push_back(ProbabilityNode(g, 1.0));
    return;
  }

  // Create a Discrete Distribution from a gaussian
  double lower = g;
  double upper = 1.0 + g;

  double sampleStepSize = (upper - lower) / maxSamples;

  double currentX = lower;

  double sum = 0.0;
  vector<ProbabilityNode> tmp;

  // Take the samples and build the discrete distribution
  for (int i = 0; i < maxSamples; i++) {
    sum += 2 * (1 + g - currentX);
    ProbabilityNode node(currentX, 2 * (1 + g - currentX));

    tmp.push_back(node);

    currentX += sampleStepSize;
  }

  // Normalize the distribution probabilities
  for (ProbabilityNode& n : tmp) {
    n.probability = n.probability / sum;
    distribution.push_back(n);
  }
}

// Creates a discrete distribution based on Pemberton's belief distribution, a uniform between 0 and 1, offset by some g-value
DiscreteDistribution::DiscreteDistribution(int maxSamples, double g, double d, int bf)
  : maxSamples(maxSamples)
{
  vector<DiscreteDistribution> uniforms;

  for (int i = 0; i < bf; i++) {
    DiscreteDistribution u(maxSamples);
    uniforms.push_back(u);
  }

  // This is a goal node, belief is a spike at true value
  if (d == 0) {
    distribution.push_back(ProbabilityNode(g, 1.0));
    return;
  }

  // Leaf nodes in this case are a convolution of bf uniform distributions between 0 and 1
  double lower = 0.0;
  double upper = 1.0;

  double sampleStepSize = (upper - lower) / maxSamples;

  double currentX = lower;

  for (int i = 0; i < maxSamples; i++) {
    // Shift the uniform distros by the leaf's g-value
    ProbabilityNode node(currentX + g, sampleStepSize);

    for (DiscreteDistribution& uniform : uniforms)
      uniform.distribution.push_back(node);

    currentX += sampleStepSize;
  }

  // Now convolute the uniform distributions
  for (int i = 1; i < static_cast<int>(uniforms.size()); i++) {
    uniforms[0] = uniforms[0] * uniforms[i];
  }

  this->distribution = uniforms[0].distribution;
}


// Creates a delta spike belief
DiscreteDistribution::DiscreteDistribution(int maxSamples, double deltaSpikeValue)
  : maxSamples(maxSamples)
{
  distribution.push_back(ProbabilityNode(deltaSpikeValue, 1.0));
}

DiscreteDistribution::DiscreteDistribution(DiscreteDistribution const *other)
  :maxSamples(other->maxSamples)
{
  for (ProbabilityNode n : other->distribution) {
    distribution.push_back(n);
  }
}

DiscreteDistribution::DiscreteDistribution(double g, double h, bool& retSuccess)
{
  if (hValueTable.find(h) == hValueTable.end()) {
    retSuccess = false;
    return;
  }

  auto& probNodeList = hValueTable[h];

  for (auto& probNode : probNodeList) {
    probNode.cost += g;
    distribution.push_back(probNode);
  }

  retSuccess = true;
}

void DiscreteDistribution::createFromUniform(int maxSamples, double g, double d)
{
  this->maxSamples = maxSamples;
  // Clear existing distro
  distribution.clear();

  // This is a goal node, belief is a spike at true value
  if (d == 0) {
    distribution.push_back(ProbabilityNode(g, 1.0));
    return;
  }

  // Create a Discrete Distribution from a gaussian
  double lower = g;
  double upper = 1.0 + g;

  double sampleStepSize = (upper - lower) / maxSamples;

  double currentX = lower;

  double probStep = 1.0 / maxSamples;

  // Take the samples and build the discrete distribution
  for (int i = 0; i < maxSamples; i++) {
    ProbabilityNode node(currentX, probStep);

    distribution.push_back(node);

    currentX += sampleStepSize;
  }
}

void DiscreteDistribution::createFromGaussian(double f, double mean, double d, double error)
{
  // Clear existing distro
  distribution.clear();

  // This is a goal node, belief is a spike at true value
  if (d == 0) {
    distribution.push_back(ProbabilityNode(mean, 1.0));
    return;
  }

  double stdDev = error / 2.0;
  auto var = std::pow(stdDev, 2);

  // Create a Discrete Distribution from a gaussian
  double lower = f;
  double upper = mean + 3 * stdDev;

  double sampleStepSize = (upper - lower) / maxSamples;

  double currentX = lower;

  double probSum = 0.0;

  vector<ProbabilityNode> tmp;

  // Take the samples and build the discrete distribution
  for (int i = 0; i < maxSamples; i++) {
    // Get the probability for this x value
    double prob = probabilityDensityFunction(currentX, mean, var);

    // So if this a goal node, we know the cost
    if (std::isnan(prob) && stdDev == 0)
      prob = 1.0;

    probSum += prob;

    ProbabilityNode node(currentX, prob);

    tmp.push_back(node);

    currentX += sampleStepSize;
  }

  // Normalize the distribution probabilities
  for (ProbabilityNode& n : tmp) {
    n.probability = n.probability / probSum;
    distribution.push_back(n);
  }
}

// TODO: doesn't matter if we're using data anyway, but this is a little inefficient
DiscreteDistribution& DiscreteDistribution::squish(double factor)
{
  std::vector<ProbabilityNode> newDistribution;
  double mean = expectedCost();

  // If the squish factor is 1, all values in distribution will be moved to the mean.
  if (factor == 1) {
    newDistribution.push_back(ProbabilityNode(mean, 1.0));
    distribution.clear();
    distribution = newDistribution;

    return *this;
  }

  /*
    cout << "Before Squish Cost,Before Squish Probability" << endl;
    for (ProbabilityNode n : distribution)
    {
    cout << n.cost << "," << n.probability << endl;
    }
    cout << endl;
  */

  for (ProbabilityNode n : distribution) {
    double distanceToMean = abs(n.cost - mean);
    double distanceToShift = distanceToMean * factor;

    double shiftedCost = n.cost;

    if (shiftedCost > mean)
      shiftedCost -= distanceToShift;
    else if (shiftedCost < mean)
      shiftedCost += distanceToShift;

    newDistribution.push_back(ProbabilityNode(shiftedCost, n.probability));
  }

  distribution.clear();
  distribution = newDistribution;

  /*
    cout << "Squish Cost,Squish Probability" << endl;
    for (ProbabilityNode n : distribution)
    {
    cout << n.cost << "," << n.probability << endl;
    }
    cout << endl;
  */

  return *this;
}

double DiscreteDistribution::expectedCost() const
{
  double E = 0.0;

  for (ProbabilityNode n : distribution) {
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

DiscreteDistribution DiscreteDistribution::operator*(const DiscreteDistribution& rhs)
{
  DiscreteDistribution csernaDistro(min(maxSamples, rhs.maxSamples));

  map<double, double> results;

  for (ProbabilityNode n1 : distribution) {
    for (ProbabilityNode n2 : rhs.distribution) {
      double probability = (n1.probability * n2.probability);

      // Don't add to the distribution if the probability of this cost is 0
      if (probability > 0)
        results[min(n1.cost, n2.cost)] += probability;
    }
  }

  csernaDistro.resize(results);

  for (map<double, double>::iterator it = results.begin(); it != results.end(); it++) {
    csernaDistro.distribution.push_back(ProbabilityNode(it->first, it->second));
  }

  /*
    cout << csernaDistro.expectedCost() << endl;
    double cdf;
    cout << "Path Cost Node 1,Probability Node 1,CDF Node 1" << endl;
    cdf = 0.0;
    for (ProbabilityNode n1 : distribution)
    {
    cdf += n1.probability;
    cout << n1.cost << "," << n1.probability << "," << cdf << endl;
    }
    cout << endl << endl;
    cout << "Path Cost Node 2,Probability Node 2,CDF Node 2" << endl;
    cdf = 0.0;
    for (ProbabilityNode n1 : rhs.distribution)
    {
    cdf += n1.probability;
    cout << n1.cost << "," << n1.probability << "," << cdf << endl;
    }
    cout << endl << endl;
    cout << "Path Cost Cserna,Probability Cserna,CDF Cserna" << endl;
    cdf = 0.0;
    for (ProbabilityNode n1 : csernaDistro.distribution)
    {
    cdf += n1.probability;
    cout << n1.cost << "," << n1.probability << "," << cdf << endl;
    }
    cout << endl << endl;
    exit(1);
  */

  return csernaDistro;
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
  assert(expected_value >= 0);
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
