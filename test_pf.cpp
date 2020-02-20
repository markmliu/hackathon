#include "kinematics.h"
#include "matplotlibcpp.h"
#include "particle_filter.h"

#include <iostream>
#include <unordered_map>

namespace plt = matplotlibcpp;

const int NUM_PARTICLES = 5000;

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

struct TrajectoriesForPlotting {
  std::vector<double> egoTravels;
  std::vector<double> objectTravels;
  std::vector<double> timestamps;
  double criticalPointS;
};

struct ManeuverProbabilitiesForPlotting {
  ManeuverProbabilitiesForPlotting() : probabilities(Maneuver::NUM_MANEUVERS) {}
  std::vector<std::vector<double>> probabilities;
  std::vector<double> timestamps;
};

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
  plt::legend();
}

void PlotInfo(const std::vector<Scene> &before,
              const std::vector<Scene> &intermediate,
              const std::vector<Scene> &resampled, const State &observation,
              const TrajectoriesForPlotting &trajectoriesSoFar,
              const ManeuverProbabilitiesForPlotting &maneuverProbabilities) {
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
  plt::show();
}

int main() {
  ParticleFilter pf(NUM_PARTICLES);
  Scene scene(State(/*s=*/0,
                    /*v=*/20,
                    /*a=*/0));

  scene.states.emplace(/*objectId=*/123, State(/*s=*/-20,
                                               /*v=*/20,
                                               /*a=*/0));
  scene.criticalPointS = 200;
  scene.timestamp = 1.0;

  TrajectoriesForPlotting trajectories;
  trajectories.criticalPointS = scene.criticalPointS;
  trajectories.egoTravels.push_back(scene.egoState.s);
  trajectories.objectTravels.push_back(scene.states.begin()->second.s);
  trajectories.timestamps.push_back(scene.timestamp);

  ManeuverProbabilitiesForPlotting maneuverProbabilities;

  pf.Init(scene);
  for (int i = 0; i < 20; ++i) {
    auto before = pf.GetParticles();
    // Let's evolve the scene at a dt of 0.5
    // Assume the actor is in same relative position, but we
    // are 15m closer to the critical point.
    double dt = 0.5;
    Scene updatedScene = scene;
    EvolveScene(dt, &updatedScene);

    UpdateInfo info = pf.Update(updatedScene);
    auto resampled = pf.GetParticles();
    State &observation = updatedScene.states.begin()->second;

    trajectories.egoTravels.push_back(updatedScene.egoState.s);
    trajectories.objectTravels.push_back(updatedScene.states.begin()->second.s);
    trajectories.timestamps.push_back(updatedScene.timestamp);

    for (int i = 0; i < info.maneuverProbabilities.size(); i++) {
      maneuverProbabilities.probabilities[i].push_back(
          info.maneuverProbabilities[i]);
    }
    maneuverProbabilities.timestamps.push_back(updatedScene.timestamp);

    PlotInfo(before, info.intermediateParticles, resampled, observation,
             trajectories, maneuverProbabilities);
    scene = updatedScene;
  }
}
