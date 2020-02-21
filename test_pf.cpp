#include "kinematics.h"
#include "particle_filter.h"
#include "plotting.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

const int NUM_PARTICLES = 5000;

namespace {
void RunWithStrategy(const State &egoStartState, const State &objectStartState,
                     Strategy objectStrategy, double objectAggressiveness,
                     std::string fileNameHint) {
  ParticleFilter pf(NUM_PARTICLES, objectAggressiveness);
  Scene scene(egoStartState);

  scene.states.emplace(/*objectId=*/123, objectStartState);
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

    auto t0 = std::chrono::high_resolution_clock::now();
    UpdateInfo info = pf.Update(updatedScene);
    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> inferenceTime = t1 - t0;
    std::cout << "inference time: " << inferenceTime.count() << std::endl;
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
             trajectories, maneuverProbabilities, info.sampledAccelsByManeuver,
             fileNameHint);
    scene = updatedScene;
  }
}
}

int main() {
  RunWithStrategy(/*egoStartState=*/State(/*s=*/0, /*v=*/20, /*a=*/0),
                  /*objectStartState=*/State(/*s=*/-30, /*v=*/20, /*a=*/0),
                  Strategy::CONSTANT_ACCEL,
                  /*objectAggressiveness=*/0.0, "stay_behind");
  RunWithStrategy(/*egoStartState=*/State(/*s=*/0, /*v=*/20, /*a=*/0),
                  /*objectStartState=*/State(/*s=*/-20, /*v=*/20, /*a=*/0),
                  Strategy::MAX_ACCEL_LATE,
                  /*objectAggressiveness=*/0.0, "speed_up_late");
  RunWithStrategy(/*egoStartState=*/State(/*s=*/0, /*v=*/20, /*a=*/0),
                  /*objectStartState=*/State(/*s=*/-30, /*v=*/20, /*a=*/0),
                  Strategy::ACCEL_THEN_DECEL_LATE,
                  /*objectAggressiveness=*/0.0, "start_beating_then_yield");
  // RunWithStrategy(/*egoStartState=*/State(/*s=*/0, /*v=*/20, /*a=*/0),
  //                 /*objectStartState=*/State(/*s=*/-30, /*v=*/20, /*a=*/0),
  //                 Strategy::ACCEL_THEN_DECEL_LATE,
  //                 /*objectAggressiveness=*/1.0,
  //                 "start_beating_then_yield_aggressive");
}
