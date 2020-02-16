#include <map>

using ObjectId = int;

struct Observation {
  ObjectId object_id;
  double t;
  double s;
  double v;
  double a;
};

class BayesianHypothesis {
public:
  BayesianHypothesis(double prior);
  void AddNewObservation(Observation observation);

  double GetLikelihood() { return likelihood_; }
  // Should only be used by MergeHypotheses, not called by user directly
  void SetLikelihood(double likelihood) { likelihood_ = likelihood; }

private:
  double likelihood_;
};

// Two hypotheses - either actor is asserting or yielding to us.
class MergeHypotheses {
public:
  // Both hypotheses should be initialized
  MergeHypotheses(Observation observation);

  void AddNewObservation(Observation observation) {
    asserting_hypothesis.AddNewObservation(observation);
    yielding_hypothesis.AddNewObservation(observation);
    NormalizeHypotheses(&asserting_hypothesis, &yielding_hypothesis);
  }

private:
  void NormalizeHypotheses(BayesianHypothesis *asserting_hypothesis,
                           BayesianHypothesis *yielding_hypothesis);
  BayesianHypothesis asserting_hypothesis;
  BayesianHypothesis yielding_hypothesis;
};

class Predictor {
public:
  void ObserveNewData(const std::vector<Observation> &observations);

private:
  // Keep track of actors that have been seen so far.
  std::map<ObjectId, MergeHypotheses> belief_map;
};
