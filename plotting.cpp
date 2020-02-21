#include "plotting.h"
#include "matplotlibcpp.h"

#include <fstream>

namespace plt = matplotlibcpp;

namespace {
void PlotParticles(const std::vector<Scene> &particles,
                   const State &observation, const std::string &title) {
  // we only care about the object state for each particle.
  std::vector<State> particleStates;
  std::transform(
      particles.begin(), particles.end(), std::back_inserter(particleStates),
      [](const Scene &scene) { return scene.states.begin()->second; });
  // Plot the s,v for each state.
  std::vector<std::vector<double>> s(Maneuver::NUM_MANEUVERS);
  std::vector<std::vector<double>> v(Maneuver::NUM_MANEUVERS);
  std::vector<std::unordered_map<std::string, std::string>> options = {
      {{"color", "red"}}, {{"color", "green"}}, {{"color", "black"}}};

  for (int i = 0; i < particleStates.size(); i++) {
    int m = particleStates[i].m;
    s[m].push_back(particleStates[i].s);
    v[m].push_back(particleStates[i].v);
  }
  plt::xlabel("travel");
  plt::ylabel("velocity");

  plt::scatter(s[0], v[0], /*markerSize=*/3.0, options[0]);
  plt::scatter(s[1], v[1], /*markerSize=*/3.0, options[1]);
  plt::scatter(s[2], v[2], /*markerSize=*/3.0, options[2]);
  // Observation
  plt::scatter(std::vector<double>({observation.s}),
               std::vector<double>({observation.v}), /*markerSize=*/20.0);
  plt::title(title);
}
void PlotTrajectories(const TrajectoriesForPlotting &trajectories) {
  plt::xlabel("time");
  plt::ylabel("travel");
  plt::named_plot("ego trajectory", trajectories.timestamps,
                  trajectories.egoTravels);
  plt::named_plot("object trajectory", trajectories.timestamps,
                  trajectories.objectTravels);
  plt::named_plot("critical point", trajectories.timestamps,
                  std::vector<double>(trajectories.egoTravels.size(),
                                      trajectories.criticalPointS));
  plt::ylim(0.0, trajectories.criticalPointS + 20);
  plt::title("Trajectories");
  plt::legend();
}

void PlotManeuverProbabilities(
    const ManeuverProbabilitiesForPlotting &maneuverProbabilities) {
  plt::xlabel("time");
  plt::ylabel("probability");
  plt::named_plot("yielding", maneuverProbabilities.timestamps,
                  maneuverProbabilities.probabilities[0], "r--");
  plt::named_plot("beating", maneuverProbabilities.timestamps,
                  maneuverProbabilities.probabilities[1], "g--");
  plt::named_plot("ignoring", maneuverProbabilities.timestamps,
                  maneuverProbabilities.probabilities[2], "k--");
  plt::title("Maneuver probabilities");
  plt::legend();
}
}

void PlotInfo(const std::vector<Scene> &before,
              const std::vector<Scene> &intermediate,
              const std::vector<Scene> &resampled, const State &observation,
              const TrajectoriesForPlotting &trajectoriesSoFar,
              const ManeuverProbabilitiesForPlotting &maneuverProbabilities,
              const std::string &strategyName) {
  plt::subplot(2, 3, 1);
  PlotParticles(before, observation, "before");
  plt::subplot(2, 3, 2);
  PlotParticles(intermediate, observation, "intermediate");
  plt::subplot(2, 3, 3);
  PlotParticles(resampled, observation, "resampled");
  plt::subplot(2, 3, 4);
  PlotTrajectories(trajectoriesSoFar);
  plt::subplot(2, 3, 5);
  PlotManeuverProbabilities(maneuverProbabilities);
  std::map<std::string, double> options = {{"hspace", 0.5}, {"wspace", 0.5}};
  plt::subplots_adjust(options);
  // plt::show();
  // hack - get the current timestep from somewhere
  std::string filename = "figs/fig" + strategyName +
                         std::to_string(trajectoriesSoFar.timestamps.size()) +
                         ".png";
  plt::save(filename);
  plt::clf();
}

TrajectoriesForPlotting::TrajectoriesForPlotting(double criticalPointS_)
    : criticalPointS(criticalPointS_) {}

void TrajectoriesForPlotting::update(const Scene &scene) {
  egoTravels.push_back(scene.egoState.s);
  egoVelocities.push_back(scene.egoState.v);
  objectTravels.push_back(scene.states.begin()->second.s);
  objectVelocities.push_back(scene.states.begin()->second.v);
  timestamps.push_back(scene.timestamp);
}

void WriteParticlesToFile(ParticlesByTimestep particlesByTimestep,
                          std::ofstream &file) {
  for (int i = 0; i < particlesByTimestep.beforeParticles.size(); ++i) {
    double timestep = particlesByTimestep.timesteps[i];
    int dist_i = 0;
    for (const auto &dist : {particlesByTimestep.beforeParticles,
                             particlesByTimestep.intermediateParticles,
                             particlesByTimestep.resampledParticles}) {
      for (const auto &particle : dist[i]) {
        auto &particleState = particle.states.begin()->second;
        // timestep, s, v, m, before/intermediate/resampled
        file << timestep << ",";
        file << particleState.s << ",";
        file << particleState.v << ",";
        file << particleState.m << ",";
        file << dist_i << std::endl;
      }
      dist_i++;
    }
  }
  file.close();
}

void WriteTrajectoriesToFile(TrajectoriesForPlotting trajectories,
                             std::ofstream &file) {
  // write the travel velocity pairs out
  for (int i = 0; i < trajectories.egoTravels.size(); ++i) {
    // ego travel, ego vel, object travel, object vel
    file << trajectories.egoTravels[i] << ",";
    file << trajectories.egoVelocities[i] << ",";
    file << trajectories.objectTravels[i] << ",";
    file << trajectories.objectVelocities[i] << std::endl;
  }
  file.close();
}
