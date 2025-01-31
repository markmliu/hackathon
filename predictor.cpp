#include "predictor.h"

BayesianHypothesis::BayesianHypothesis(double prior) : likelihood_(prior) {}

MergeHypotheses::MergeHypotheses(Observation observation)
    : asserting_hypothesis(0.5), yielding_hypothesis(0.5) {}

void BayesianHypothesis::AddNewObservation(Observation observation) {
  // TODO: fill this in.
  return;
}

void MergeHypotheses::NormalizeHypotheses(
    BayesianHypothesis *asserting_hypothesis,
    BayesianHypothesis *yielding_hypothesis) {

  double asserting_likelihood = asserting_hypothesis->GetLikelihood();
  double yielding_likelihood = yielding_hypothesis->GetLikelihood();
  double total_prob = asserting_likelihood + yielding_likelihood;
  asserting_hypothesis->SetLikelihood(asserting_likelihood / total_prob);
  yielding_hypothesis->SetLikelihood(asserting_likelihood / total_prob);
}

void Predictor::ObserveNewData(const std::vector<Observation> &observations) {}
