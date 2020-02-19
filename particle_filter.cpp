#include "particle_filter.h"
#include "idm.h"

#include <assert.h>
#include <iostream>

namespace {

// Only two types of scenes for now -
// actor is yielding to ego
// actor is beating ego
// Return the kinematic means for each situation
std::vector<State> GetManeuverMeanStates(const std::vector<Scene> &particles,
                                         int numManeuvers) {

  std::vector<int> maneuverCount(numManeuvers, 0);

  // Assume only one object
  assert(particles.size() != 0);
  assert(particles[0].states.size() != 0);
  ObjectId objectId = particles[0].states.begin()->first;

  std::vector<State> meanManeuverStates(numManeuvers, State(0, 0, 0));

  for (const auto &particle : particles) {
    const auto &state = particle.states.at(objectId);
    int maneuver = state.m;
    maneuverCount[maneuver]++;
    int n = maneuverCount[maneuver];
    meanManeuverStates[maneuver].s +=
        (state.s - meanManeuverStates[maneuver].s) / n;
    meanManeuverStates[maneuver].v +=
        (state.v - meanManeuverStates[maneuver].v) / n;
    meanManeuverStates[maneuver].a +=
        (state.a - meanManeuverStates[maneuver].a) / n;
  }
  return meanManeuverStates;
}

void ApplyManeuverSpecificAccelConstraints(Maneuver maneuver,
                                           const State &objectState,
                                           const State &egoState,
                                           double criticalPointS,
                                           double *maxAccel, double *minAccel) {
  if (maneuver == Maneuver::YIELDING) {
    *maxAccel = std::min(
        *maxAccel,
        getMaxAccelBehindEgoAtConflictRegion(
            /*actorDistanceToConflictPoint=*/criticalPointS - objectState.s,
            /*actorVelocity=*/objectState.v,
            /*egoDistanceToConflictPoint=*/criticalPointS - egoState.s,
            /*egoVelocity=*/egoState.v));
  }
  if (maneuver == Maneuver::BEATING) {
    *minAccel = std::max(
        *minAccel,
        getMinAccelInFrontOfEgoAtConflictRegion(
            /*actorDistanceToConflictPoint=*/criticalPointS - objectState.s,
            /*actorVelocity=*/objectState.v,
            /*egoDistanceToConflictPoint=*/criticalPointS - egoState.s,
            /*egoVelocity=*/egoState.v));
  }
  if (maneuver == Maneuver::IGNORING) {
    *maxAccel =
        std::min(*maxAccel, getIDMAccelFreeRoad(objectState.v,
                                                /*desiredVel=*/30,
                                                /*vehicleMaxAccel=*/5.0));
  }
}

// Updates s to dt seconds later, assuming constant accel.
// Ignores a in s, replaces it with accel.
void ApplyAccel(double accel, double dt, State *state) {
  state->s += (state->v * dt) + (.5 * accel * dt * dt);
  state->v += accel * dt;
  state->a = accel;
}

double pdf_gaussian(double x, double m, double s) {
  return (1 / (s * sqrt(2 * M_PI))) * exp(-0.5 * pow((x - m) / s, 2.0));
}

} // anonymous namespace

ParticleFilter::ParticleFilter(int numParticles)
    : numParticles_(numParticles) {}

std::vector<State> ParticleFilter::GetMeanStates() const {
  return GetManeuverMeanStates(particles_, Maneuver::NUM_MANEUVERS);
}

double ParticleFilter::RelativeLikelihood(State observation,
                                          State expectation) const {
  return pdf_gaussian(observation.s, expectation.s, PosStdDev_) *
         pdf_gaussian(observation.v, expectation.v, VelStdDev_) *
         pdf_gaussian(observation.a, expectation.a, AccStdDev_);
}

void ParticleFilter::Init(const Scene &scene) {
  // Each particle represents a version of true values for all actors.
  current_timestamp_ = scene.timestamp;
  for (int i = 0; i < numParticles_; ++i) {
    Scene particle(scene.egoState);
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
      std::discrete_distribution<int> maneuverDistribution{1, 1, 1};
      int maneuver = maneuverDistribution(generator_);
      state.m = Maneuver(maneuver);

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
  assert(scene.timestamp == current_timestamp_ + dt);
  current_timestamp_ = scene.timestamp;

  // std dev used when sampling accel.
  const double accelSamplingStdDev = 1.0;
  // First step: predict particles forward only using model assumptions, without
  // looking at the observation..

  // To reduce computation burden, compute mean kinematic state of all agents
  // for each maneuver intention.
  // For now, this means just compute min kinematic state of agent if yielding
  // vs beating.
  std::vector<State> maneuverMeanStates = GetMeanStates();

  // ---------------Sample agent acceleration--------
  // Absolute max/min's which will be further bounded by maneuver intentions.
  const double maxVehicleAccel = 5.0;
  const double minVehicleAccel = -5.0;

  for (int maneuver = 0; maneuver < Maneuver::NUM_MANEUVERS; ++maneuver) {
    auto &state = maneuverMeanStates[maneuver];
    double maxAccel = maxVehicleAccel;
    double minAccel = minVehicleAccel;
    ApplyManeuverSpecificAccelConstraints((Maneuver)maneuver, state,
                                          scene.egoState, scene.criticalPointS,
                                          &maxAccel, &minAccel);
    double accelSamplingMean =
        std::min((maxAccel + minAccel) / 2, maxAccel - accelSamplingStdDev);
    std::normal_distribution<double> accDistribution(
        /*mean=*/accelSamplingMean, /*stdDev=*/accelSamplingStdDev);
    double sampledAccel = accDistribution(generator_);
    std::cout << "for maneuver: " << maneuver << ", min accel is " << minAccel
              << " and max accel is " << maxAccel << std::endl;
    std::cout << "sampled accel: " << sampledAccel << " from mean "
              << accelSamplingMean << std::endl;
    ApplyAccel(sampledAccel, dt, &state);
    state.print();
  }

  // Update weights - what's likelihood of seeing actual observation in either
  // case.
  const auto &observedState = scene.states.begin()->second;
  std::cout << "observed scene: " << std::endl;
  scene.print();

  std::vector<double> weights(Maneuver::NUM_MANEUVERS);
  {
    double total = 0.0;
    for (int maneuver = 0; maneuver < Maneuver::NUM_MANEUVERS; ++maneuver) {
      weights[maneuver] =
          RelativeLikelihood(observedState, maneuverMeanStates[maneuver]);
      total += weights[maneuver];
    }
    for (int maneuver = 0; maneuver < Maneuver::NUM_MANEUVERS; ++maneuver) {
      weights[maneuver] /= total;
      std::cout << "likelihood of maneuver: " << maneuver << ": "
                << weights[maneuver] << std::endl;
    }
  }
}
