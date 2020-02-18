#include "particle_filter.h"

void ParticleFilter::Init(const Scene &scene) {
  // Each particle represents a version of true values for all actors.
  for (int i = 0; i < NUM_PARTICLES; ++i) {
    Scene particle;
    for (const auto &objectState : scene.states) {
      std::normal_distribution<double> posDistribution(
          /*mean=*/objectState.second.s, /*stdDev=*/PosStdDev_);
      std::normal_distribution<double> velDistribution(
          /*mean=*/objectState.second.v, /*stdDev=*/VelStdDev_);
      std::normal_distribution<double> accDistribution(
          /*mean=*/objectState.second.a, /*stdDev=*/AccStdDev_);
      State state(posDistribution(generator_), velDistribution(generator_),
                  accDistribution(generator_));
      particle.states.emplace(objectState.first, state);
    }
    particles_.push_back(particle);
  }
}

void ParticleFilter::Update(const Scene &scene) {
  // Predict particles forward.
}
