#pragma once

#include <vector>

#include "types.h"

struct TrajectoriesForPlotting {
  TrajectoriesForPlotting(double criticalPointS_);
  std::vector<double> egoTravels;
  std::vector<double> objectTravels;
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
              const ManeuverProbabilitiesForPlotting &maneuverProbabilities);