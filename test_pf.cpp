#include "kinematics.h"
#include "matplotlibcpp.h"
#include "particle_filter.h"

#include <iostream>
#include <unordered_map>

namespace plt = matplotlibcpp;

const int NUM_PARTICLES = 500;

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

void PlotBeforeAfter(const std::vector<Scene> &before,
                     const std::vector<Scene> &expected,
                     const std::vector<Scene> &resampled,
                     const State &observation) {
  plt::subplot(2, 2, 1);
  PlotParticles(before, observation, "before");
  plt::subplot(2, 2, 2);
  PlotParticles(expected, observation, "expected");
  plt::subplot(2, 2, 3);
  PlotParticles(resampled, observation, "resampled");
  plt::show();
}

int main() {
  ParticleFilter pf(NUM_PARTICLES);
  Scene scene(State(/*s=*/0,
                    /*v=*/20,
                    /*a=*/0));
  scene.states.emplace(/*objectId=*/123, State(/*s=*/0,
                                               /*v=*/20,
                                               /*a=*/0));
  scene.criticalPointS = 200;
  scene.timestamp = 1.0;

  pf.Init(scene);
  auto before = pf.GetParticles();
  // what do our particles look like?
  {
    // Let's evolve the scene at a dt of 0.5
    // Assume the actor is in same relative position, but we
    // are 15m closer to the critical point.
    double dt = 0.5;
    Scene updatedScene = scene;
    EvolveState(dt, &updatedScene.egoState);
    EvolveState(dt, &updatedScene.states.at(123));
    updatedScene.timestamp = 1.5;

    auto expected = pf.Update(updatedScene);
    auto resampled = pf.GetParticles();
    State &observation = updatedScene.states.begin()->second;
    PlotBeforeAfter(before, expected, resampled, observation);
  }
}
