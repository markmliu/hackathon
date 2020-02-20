#pragma once

#include <random>
#include <vector>

#include "types.h"

struct UpdateInfo {
  // Particlese after action and before resampling
  std::vector<Scene> intermediateParticles;
  std::vector<double> maneuverProbabilities;
};

class ParticleFilter {
  // initialize it with particles set to observations
  // add random gaussian noise to them.
  // (need some inputs for standard deviations for all measurements)

  // On new measurement:
  // For each particle:
  // Sample update particle from prev timestep + probabilistic model
  // Weight (importance factor for the particle, given the observation)
  // X-bar contains all weight, sample pairs.
  // For num particles:
  // - draw with replacement from x-bar.
  // This approximates our new X-bar.
public:
  ParticleFilter(int numParticles);
  void Init(const Scene &scene);
  UpdateInfo Update(const Scene &scene);
  std::vector<Scene> GetParticles() const;
  void PrintParticles();
  // exposed for testing.
  std::vector<State> GetMeanStates() const;

private:
  double RelativeLikelihood(State observation, State expectation) const;
  std::default_random_engine generator_;
  const int PosStdDev_ = 2; // m
  const int VelStdDev_ = 2; // m/s
  const int AccStdDev_ = 2; // m/s2

  int numParticles_;
  std::vector<Scene> particles_;
  double current_timestamp_;
};
