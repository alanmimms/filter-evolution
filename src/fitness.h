#pragma once
#include <vector>
#include <string>
#include <memory>
#include "genome.h"
#include "spice_interface.h"

struct HarmonicSpec {
  std::string bandName;
  double fundamentalFreqMhz;
  int harmonicOrder;
  double harmonicFreqMhz;
  double levelDbc;
};

class FitnessEvaluator {
public:
  FitnessEvaluator(const std::vector<HarmonicSpec>& harmonics);

  double Evaluate(CircuitGenome& genome);

  // Separate simulation submission from fitness computation for parallelism
  double ComputeFitness(CircuitGenome& genome, const SimulationResult& simResult);

  void PrintDetailedPerformance(const CircuitGenome& genome) const;

private:
  std::vector<HarmonicSpec> harmonicSpecs;
  // Removed simulator member - now using singleton

  static constexpr double TargetOutputDbc = -50.0;
  static constexpr double MaxPassbandLossDb = 1.0;
  static constexpr double InductorCostPerNh = 0.001;

  double InterpolateAtFrequency(const FrequencyResponse& response,
                                 double freqHz) const;
};
