#include "particle_filter.h"

#include <assert.h>
#include <iostream>

namespace {
void AddManeuverToState(int maneuver, State *state) {
  if (maneuver == 0) {
    state->leadObject = NO_OBJECT;
    state->rearObject = EGO_ID;
  } else if (maneuver == 1) {
    state->leadObject = EGO_ID;
    state->rearObject = NO_OBJECT;
  }
  assert("should only have those two options for now");
  return;
}

// Only two types of scenes for now -
// actor is yielding to ego
// actor is beating ego
// Return the kinematic means for each situation
std::pair<Scene, Scene>
GetYieldingBeatingMeanStates(const std::vector<Scene> &particles) {
  int numYieldingScenesSoFar = 0;
  int numBeatingScenesSoFar = 0;

  // Assume only one object
  assert(particles.size() != 0);
  assert(particles[0].states.size() != 0);
  ObjectId objectId = particles[0].states.begin()->first;

  Scene yieldingMeanScene;
  Scene beatingMeanScene;
  yieldingMeanScene.states.emplace(objectId, State(0, 0, 0));
  beatingMeanScene.states.emplace(objectId, State(0, 0, 0));

  for (const auto &particle : particles) {
    const auto &state = particle.states.at(objectId);
    if (state.leadObject == EGO_ID) {
      // yielding
      std::cout << "updating yielding particle" << std::endl;
      numYieldingScenesSoFar++;
      int n = numYieldingScenesSoFar;
      auto &yieldingState = yieldingMeanScene.states.at(objectId);
      yieldingState.s += (state.s - yieldingState.s) / n;
      yieldingState.v += (state.v - yieldingState.v) / n;
      yieldingState.a += (state.a - yieldingState.a) / n;
    } else {
      // beating
      std::cout << "updating beating particle" << std::endl;
      numBeatingScenesSoFar++;
      int n = numBeatingScenesSoFar;
      auto &beatingState = beatingMeanScene.states.at(objectId);
      beatingState.s += (state.s - beatingState.s) / n;
      beatingState.v += (state.v - beatingState.v) / n;
      beatingState.a += (state.a - beatingState.a) / n;
    }
  }
  return std::make_pair(yieldingMeanScene, beatingMeanScene);
}
} // anonymous namespace

ParticleFilter::ParticleFilter(int numParticles)
    : numParticles_(numParticles) {}

std::pair<Scene, Scene> ParticleFilter::GetMeanScenes() const {
  return GetYieldingBeatingMeanStates(particles_);
}

void ParticleFilter::Init(const Scene &scene) {
  // Each particle represents a version of true values for all actors.
  for (int i = 0; i < numParticles_; ++i) {
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
      AddManeuverToState(maneuver, &state);

      particle.states.emplace(objectState.first, state);
    }
    particles_.push_back(particle);
  }
}

void ParticleFilter::PrintParticles() {
  for (int i = 0; i < numParticles_; ++i) {
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

  // To reduce computation burden, compute mean kinematic state of all agents
  // for each maneuver intention.
  // For now, this means just compute min kinematic state of agent if yielding
  // vs beating.
  std::pair<Scene, Scene> yielding_beating_states =
      GetYieldingBeatingMeanStates(particles_);

  // Absolute max/min's which will be further bounded by maneuver intentions.
  const double maxVehicleAccel = 10.0;
  const double minVehicleAccel = -5.0;

  const double dt = 0.5;

  double softMaxAccel = maxVehicleAccel;
  double softMinAccel = minVehicleAccel;
}
