#include "particle_filter.h"
#include "idm.h"

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

// Updates s to dt seconds later, assuming constant accel.
// Ignores a in s, replaces it with accel.
void ApplyAccel(double accel, double dt, State *state) {
  state->s += (state->v * dt) + (.5 * accel * dt * dt);
  state->v += accel * dt;
  state->a = accel;
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
  // Assume we're stepping forward by dt.
  const double dt = 0.5;

  // std dev used when sampling accel.
  const double accelSamplingStdDev = 1.0;
  // First step: predict particles forward only using model assumptions, without
  // looking at the observation..

  // To reduce computation burden, compute mean kinematic state of all agents
  // for each maneuver intention.
  // For now, this means just compute min kinematic state of agent if yielding
  // vs beating.
  std::pair<Scene, Scene> yieldingBeatingScenes =
      GetYieldingBeatingMeanStates(particles_);

  // ---------------Sample agent acceleration--------
  // Absolute max/min's which will be further bounded by maneuver intentions.
  const double maxVehicleAccel = 10.0;
  const double minVehicleAccel = -5.0;

  // Do yielding scene first.
  {
    auto &yieldingScene = yieldingBeatingScenes.first;
    auto &yieldingState = yieldingScene.states.begin()->second;
    double maxAccelYielding = maxVehicleAccel;
    double minAccelYielding = minVehicleAccel;
    maxAccelYielding =
        std::min(maxAccelYielding,
                 getMaxAccelBehindEgoAtConflictRegion(
                     /*actorDistanceToConflictPoint=*/yieldingState.s +
                         scene.distToCriticalPoint,
                     /*actorVelocity=*/yieldingState.v,
                     /*egoDistanceToConflictPoint=*/scene.distToCriticalPoint,
                     /*egoVelocity=*/scene.egoVelocity));

    // Paper says to sample from gaussian centered at maxAccel - stdDev, but
    // want to make sure that mean is always less than that for beating
    //
    double accelSamplingMean =
        std::min((maxAccelYielding + minAccelYielding) / 2,
                 maxAccelYielding - accelSamplingStdDev);
    std::normal_distribution<double> accDistribution(
        /*mean=*/accelSamplingMean, /*stdDev=*/accelSamplingStdDev);
    double sampledAccel = accDistribution(generator_);
    ApplyAccel(sampledAccel, dt, &yieldingState);
  }

  // Now beating scene
  {
    auto &beatingScene = yieldingBeatingScenes.second;
    auto &beatingState = beatingScene.states.begin()->second;
    double maxAccelBeating = maxVehicleAccel;
    double minAccelBeating = minVehicleAccel;
    minAccelBeating =
        std::max(minAccelBeating,
                 getMinAccelInFrontOfEgoAtConflictRegion(
                     /*actorDistanceToConflictPoint=*/beatingState.s +
                         scene.distToCriticalPoint,
                     /*actorVelocity=*/beatingState.v,
                     /*egoDistanceToConflictPoint=*/scene.distToCriticalPoint,
                     /*egoVelocity=*/scene.egoVelocity));
    // Paper says to sample from gaussian centered at maxAccel - stdDev, but
    // want to make sure that mean is always higher than that for yielding.
    double accelSamplingMean = std::max((maxAccelBeating + minAccelBeating) / 2,
                                        maxAccelBeating - accelSamplingStdDev);
    std::normal_distribution<double> accDistribution(
        /*mean=*/accelSamplingMean, /*stdDev=*/accelSamplingStdDev);
    double sampledAccel = accDistribution(generator_);
    ApplyAccel(sampledAccel, dt, &beatingState);
  }
}
