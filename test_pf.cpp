#include "kinematics.h"
#include "particle_filter.h"
#include "plotting.h"

#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

const int NUM_PARTICLES = 5000;

namespace {
void RunWithStrategy(Strategy objectStrategy, std::string fileNameHint) {
  ParticleFilter pf(NUM_PARTICLES);
  Scene scene(State(/*s=*/0,
                    /*v=*/20,
                    /*a=*/0));

  scene.states.emplace(/*objectId=*/123, State(/*s=*/-20,
                                               /*v=*/20,
                                               /*a=*/0));
  scene.criticalPointS = 200;
  scene.timestamp = 1.0;

  TrajectoriesForPlotting trajectories(scene.criticalPointS);
  trajectories.update(scene);

  ManeuverProbabilitiesForPlotting maneuverProbabilities;

  pf.Init(scene);
  while (scene.egoState.s < scene.criticalPointS + 20) {
    auto before = pf.GetParticles();
    // Let's evolve the scene at a dt of 0.5
    // Assume the actor is in same relative position, but we
    // are 15m closer to the critical point.
    double dt = 0.5;
    Scene updatedScene = scene;

    EvolveScene(dt, &updatedScene,
                /*egoStrategy=*/Strategy::CONSTANT_ACCEL, objectStrategy);

    UpdateInfo info = pf.Update(updatedScene);
    auto resampled = pf.GetParticles();
    State &observation = updatedScene.states.begin()->second;

    trajectories.update(updatedScene);

    for (int i = 0; i < info.maneuverProbabilities.size(); i++) {
      maneuverProbabilities.probabilities[i].push_back(
          info.maneuverProbabilities[i]);
    }
    maneuverProbabilities.timestamps.push_back(updatedScene.timestamp);

    // If not saving to file, display progress after each timestep.
    PlotInfo(before, info.intermediateParticles, resampled, observation,
             trajectories, maneuverProbabilities, fileNameHint);
    scene = updatedScene;
  }
}
}

int main() {
  RunWithStrategy(Strategy::CONSTANT_ACCEL, "constant_accel");
  RunWithStrategy(Strategy::MAX_ACCEL_LATE, "max_accel_late");
  RunWithStrategy(Strategy::ACCEL_THEN_DECEL_LATE, "accel_then_decel_late");
}
