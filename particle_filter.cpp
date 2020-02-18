#include "particle_filter.h"

#include <assert.h>
#include <iostream>

namespace {
void AddManeuverToState(int maneuver, State *state) {
  if (maneuver == 0) {
    state->leadObject = NO_OBJECT;
  } else if (maneuver == 1) {
    state->leadObject = EGO_ID;
  }
  assert("should only have those two options for now");
  return;
}
} // anonymous namespace

void ParticleFilter::Init(const Scene &scene) {
  // Each particle represents a version of true values for all actors.
  for (int i = 0; i < NUM_PARTICLES; ++i) {
    Scene particle;
    for (const auto &objectState : scene.states) {
      // kinematic
      std::normal_distribution<double> posDistribution(
          /*mean=*/objectState.second.s, /*stdDev=*/PosStdDev_);
      std::normal_distribution<double> velDistribution(
          /*mean=*/objectState.second.v, /*stdDev=*/VelStdDev_);
      std::normal_distribution<double> accDistribution(
          /*mean=*/objectState.second.a, /*stdDev=*/AccStdDev_);
      State state(posDistribution(generator_), velDistribution(generator_),
                  accDistribution(generator_));

      // maneuver intention
      // start off by assuming only one merging vehicle and ego, so only choices
      // are no leader or ego vehicle.
      std::discrete_distribution<int> maneuverDistribution{1, 1};
      int maneuver = maneuverDistribution(generator_);

      particle.states.emplace(objectState.first, state);
    }
    particles_.push_back(particle);
  }
}

void ParticleFilter::PrintParticles() {
  for (int i = 0; i < NUM_PARTICLES; ++i) {
    for (const auto &objectState : particles_[i].states) {
      std::cout << "particle " << i << " object id: " << objectState.first
                << " s: " << objectState.second.s
                << " v: " << objectState.second.v
                << " a: " << objectState.second.a << std::endl;
    }
  }
}

void ParticleFilter::Update(const Scene &scene) {
  // First step: predict particles forward only using model assumptions, without
  // looking at the observation..
  const double dt = 0.5;
}
