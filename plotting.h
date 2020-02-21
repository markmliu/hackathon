#pragma once

#include <string>
#include <vector>

#include "types.h"

struct TrajectoriesForPlotting {
  TrajectoriesForPlotting(double criticalPointS_);
  std::vector<double> egoTravels;
  std::vector<double> egoVelocities;
  std::vector<double> objectTravels;
  std::vector<double> objectVelocities;
  std::vector<double> timestamps;
  double criticalPointS;
  void update(const Scene &scene);
};

struct ManeuverProbabilitiesForPlotting {
  ManeuverProbabilitiesForPlotting() : probabilities(Maneuver::NUM_MANEUVERS) {}
  std::vector<std::vector<double>> probabilities;
  std::vector<double> timestamps;
};

void PlotInfo(const std::vector<Scene> &before,
              const std::vector<Scene> &intermediate,
              const std::vector<Scene> &resampled, const State &observation,
              const TrajectoriesForPlotting &trajectoriesSoFar,
              const ManeuverProbabilitiesForPlotting &maneuverProbabilities,
              const std::string &strategyName);

// Utils for writing files should probably go somewhere else.
struct ParticlesByTimestep {
  std::vector<std::vector<Scene>> beforeParticles;
  std::vector<std::vector<Scene>> intermediateParticles;
  std::vector<std::vector<Scene>> resampledParticles;

  std::vector<double> timesteps;
};

void WriteParticlesToFile(ParticlesByTimestep particlesByTimestep,
                          std::ofstream &file);

void WriteTrajectoriesToFile(TrajectoriesForPlotting trajectories,
                             std::ofstream &file);
