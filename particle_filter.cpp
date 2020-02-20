#include "particle_filter.h"
#include "idm.h"

#include <assert.h>
#include <iostream>

namespace {

// Return the kinematic means for each maneuver.
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

std::vector<Scene> ParticleFilter::GetParticles() const { return particles_; }

void ParticleFilter::Init(const Scene &scene) {
  // Each particle represents a version of true values for all actors.
  current_timestamp_ = scene.timestamp;
  for (int i = 0; i < numParticles_; ++i) {
    particles_.push_back(SampleFromCurrentMeasurementDistribution(scene));
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

UpdateInfo ParticleFilter::Update(const Scene &scene) {
  // Assume we're stepping forward by dt.
  const double dt = 0.5;
  std::cout << "scene timestamp: " << scene.timestamp
            << " current timestamp: " << current_timestamp_ << std::endl;
  assert(scene.timestamp == current_timestamp_ + dt);
  current_timestamp_ = scene.timestamp;

  // std dev used when sampling accel.
  const double accelSamplingStdDev = 1.0;

  // First step: predict particles forward only using model assumptions, without
  // looking at the observation..

  // ---------------Sample agent acceleration--------
  // Absolute max/min's which will be further bounded by maneuver intentions.
  const double maxVehicleAccel = 5.0;
  const double minVehicleAccel = -5.0;

  for (int i = 0; i < numParticles_; ++i) {
    auto &state = particles_[i].states.begin()->second;

    double maxAccel = maxVehicleAccel;
    double minAccel = minVehicleAccel;
    ApplyManeuverSpecificAccelConstraints(state.m, state, scene.egoState,
                                          scene.criticalPointS, &maxAccel,
                                          &minAccel);
    double accelSamplingMean =
        std::min((maxAccel + minAccel) / 2, maxAccel - accelSamplingStdDev);
    std::normal_distribution<double> accDistribution(
        /*mean=*/accelSamplingMean, /*stdDev=*/accelSamplingStdDev);
    double sampledAccel = accDistribution(generator_);
    ApplyAccel(sampledAccel, dt, &state);
  }

  // Update weights - what's likelihood of seeing actual observation in either
  // case.
  const auto &observedState = scene.states.begin()->second;
  scene.print();

  std::vector<double> weights(numParticles_);
  {
    double total = 0.0;
    for (int i = 0; i < numParticles_; ++i) {
      weights[i] = RelativeLikelihood(observedState,
                                      particles_[i].states.begin()->second);
      total += weights[i];
    }

    std::vector<double> maneuverProbs(Maneuver::NUM_MANEUVERS);
    for (int i = 0; i < numParticles_; ++i) {
      weights[i] /= total;
    }
  }

  // Resample
  std::vector<Scene> particlesResampled;
  particlesResampled.reserve(numParticles_);
  std::discrete_distribution<int> weightedSampler(weights.begin(),
                                                  weights.end());
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  std::vector<int> maneuverCounter(Maneuver::NUM_MANEUVERS);
  for (int i = 0; i < numParticles_; ++i) {
    // with some small probability, resample from environment
    // rather than from updated particles to prevent
    // particle deprivation.
    if (uniform(generator_) < 0.05) {
      particlesResampled.push_back(
          SampleFromCurrentMeasurementDistribution(scene));
    } else {
      particlesResampled.push_back(particles_[weightedSampler(generator_)]);
    }
    Maneuver m = particlesResampled[i].states.begin()->second.m;
    maneuverCounter[m]++;
  }

  std::swap(particles_, particlesResampled);

  UpdateInfo info;
  info.intermediateParticles = particlesResampled;
  for (int i = 0; i < Maneuver::NUM_MANEUVERS; ++i) {
    info.maneuverProbabilities.push_back((double)maneuverCounter[i] /
                                         numParticles_);
  }

  return info;
}

Scene ParticleFilter::SampleFromCurrentMeasurementDistribution(
    const Scene &observation) {
  Scene particle(observation.egoState);
  for (const auto &objectState : observation.states) {
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
  return particle;
}
