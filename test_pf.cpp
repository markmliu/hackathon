#include "kinematics.h"
#include "particle_filter.h"
#include "plotting.h"

#include <string.h>
#include <iostream>
#include <unordered_map>

const int NUM_PARTICLES = 5000;

int main(int argc, char *argv[]) {
  bool saveToFile = argc != 1 && strcmp(argv[1], "save") == 0;
  if (saveToFile) {
      std::cout << "todo: save to file!" << std::endl;
  }
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

    // Strategy objectStrategy = Strategy::MAX_ACCEL_LATE;
    Strategy objectStrategy =Strategy::ACCEL_THEN_DECEL_LATE;
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

    PlotInfo(before, info.intermediateParticles, resampled, observation,
             trajectories, maneuverProbabilities);
    scene = updatedScene;
  }
}
