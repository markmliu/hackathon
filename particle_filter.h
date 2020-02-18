#pragma once

#include <random>
#include <vector>

#include "types.h"

class ParticleFilter {
  // initialize it with particles set to observations
  // add random gaussian noise to them.
  // (need some inputs for standard deviations for all measurements)

  // On new measurement:
  // For each particle:
  // Sample update particle from prev timestep + probabilistic model
  // Weight (importance factor for the particle, given the observation)
  // X-bar contains all weight, sample pairs.
  // For num particles:
  // - draw with replacement from x-bar.
  // This approximates our new X-bar.
public:
  void Init(const Scene &scene);

private:
  const int NUM_PARTICLES = 500;
  std::default_random_engine generator_;
  const int PosStdDev_ = 2; // m
  const int VelStdDev_ = 2; // m/s
  const int AccStdDev_ = 2; // m/s2

  std::vector<Scene> particles_;
};
