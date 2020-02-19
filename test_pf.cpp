#include "kinematics.h"
#include "matplotlibcpp.h"
#include "particle_filter.h"

#include <iostream>

namespace plt = matplotlibcpp;

const int NUM_PARTICLES = 500;

void PlotParticles(std::vector<Scene> &particles) {
  // we only care about the object state for each particle.
  std::vector<State> particleStates;
  std::transform(
      particles.begin(), particles.end(), std::back_inserter(particleStates),
      [](const Scene &scene) { return scene.states.begin()->second; });
  // Plot the s,v for each state.
  std::vector<double> s(particleStates.size());
  std::vector<double> v(particleStates.size());

  for (int i = 0; i < particleStates.size(); i++) {
    s[i] = particleStates[i].s;
    v[i] = particleStates[i].v;
  }
  plt::xlabel("travel");
  plt::ylabel("velocity");
  plt::scatter(s, v);
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
  auto particles = pf.GetParticles();
  // what do our particles look like?
  PlotParticles(particles);
  {
    // Let's evolve the scene at a dt of 0.5
    // Assume the actor is in same relative position, but we
    // are 15m closer to the critical point.
    double dt = 0.5;
    Scene updatedScene = scene;
    EvolveState(dt, &updatedScene.egoState);
    EvolveState(dt, &updatedScene.states.at(123));
    updatedScene.timestamp = 1.5;

    pf.Update(updatedScene);
  }
}
