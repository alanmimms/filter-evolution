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
  void PrintDetailedPerformance(const CircuitGenome& genome) const;
  
private:
  std::vector<HarmonicSpec> harmonicSpecs;
  std::unique_ptr<SpiceSimulator> simulator;
  
  static constexpr double TargetOutputDbc = -50.0;
  static constexpr double MaxPassbandLossDb = 0.5;
  static constexpr double InductorCostPerNh = 0.001;
  
  double InterpolateAtFrequency(const FrequencyResponse& response, 
                                 double freqHz) const;
};