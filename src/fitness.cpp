#include "fitness.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

FitnessEvaluator::FitnessEvaluator(const std::vector<HarmonicSpec>& harmonics)
  : harmonicSpecs(harmonics) {
  // Use singleton instance instead of per-evaluator instance
}

double FitnessEvaluator::Evaluate(CircuitGenome& genome) {
  if (!genome.IsValid()) {
    genome.fitness = 1e9;
    return genome.fitness;
  }

  // Submit simulation to the worker thread and wait for result
  auto future = SpiceSimulator::GetInstance().RunAcAnalysis(genome);
  SimulationResult simResult = future.get();

  // Handle different failure types
  if (!simResult.IsSuccess()) {
    switch (simResult.failureType) {
      case SimulationFailureType::InvalidCircuit:
        // Circuit structure is invalid - high penalty
        genome.fitness = 1e8;
        return genome.fitness;

      case SimulationFailureType::SimulationError:
        // NGSpice couldn't simulate (convergence, etc.) - penalize but less than invalid
        genome.fitness = 1e7;
        return genome.fitness;

      case SimulationFailureType::InternalError:
        // Bug in our code or NGSpice - report and treat as very bad
        std::cerr << "Internal error: " << simResult.errorMessage << std::endl;
        genome.fitness = 1e9;
        return genome.fitness;

      default:
        genome.fitness = 1e9;
        return genome.fitness;
    }
  }

  const FrequencyResponse& response = simResult.response;
  double penalty = 0.0;

  // Evaluate each harmonic specification
  for (const auto& harmonic : harmonicSpecs) {
    double freqHz = harmonic.harmonicFreqMhz * 1e6;
    double attenuation = -InterpolateAtFrequency(response, freqHz);

    if (harmonic.harmonicOrder == 1) {
      // Fundamental - check passband loss
      if (attenuation > MaxPassbandLossDb) {
        penalty += (attenuation - MaxPassbandLossDb) * 100.0;
      }
    } else {
      // Harmonic - check stopband attenuation
      double outputLevel = harmonic.levelDbc - attenuation;

      if (outputLevel > TargetOutputDbc) {
        double shortfall = outputLevel - TargetOutputDbc;
        // Weight by harmonic order - 2nd harmonic is most critical
        double weight = 50.0 * (5.0 / harmonic.harmonicOrder);
        penalty += shortfall * weight;
      }
    }
  }

  // Inductor cost penalty
  int inductorCount = genome.CountActiveInductors();
  penalty += inductorCount * 10.0;  // Prefer fewer inductors

  genome.fitness = penalty;
  return penalty;
}

double FitnessEvaluator::InterpolateAtFrequency(const FrequencyResponse& response,
                                                 double freqHz) const {
  if (response.frequencies.empty()) return 0.0;
  
  // Find bracketing frequencies
  auto it = std::lower_bound(response.frequencies.begin(), 
                             response.frequencies.end(), freqHz);
  
  if (it == response.frequencies.begin()) {
    return response.magnitudeDb[0];
  }
  if (it == response.frequencies.end()) {
    return response.magnitudeDb.back();
  }
  
  size_t idx = std::distance(response.frequencies.begin(), it);
  double f1 = response.frequencies[idx - 1];
  double f2 = response.frequencies[idx];
  double m1 = response.magnitudeDb[idx - 1];
  double m2 = response.magnitudeDb[idx];
  
  // Log-linear interpolation (appropriate for frequency domain)
  double logF = std::log10(freqHz);
  double logF1 = std::log10(f1);
  double logF2 = std::log10(f2);
  
  double t = (logF - logF1) / (logF2 - logF1);
  return m1 + t * (m2 - m1);
}

void FitnessEvaluator::PrintDetailedPerformance(const CircuitGenome& genome) const {
  if (!genome.IsValid()) {
    std::cout << "    Circuit is invalid - no connectivity path\n";
    return;
  }

  // Submit simulation and wait for result
  auto future = SpiceSimulator::GetInstance().RunAcAnalysis(genome);
  SimulationResult simResult = future.get();

  if (!simResult.IsSuccess()) {
    std::cout << "    Simulation failed: " << simResult.errorMessage << "\n";
    return;
  }

  const FrequencyResponse& response = simResult.response;
  std::cout << "    Harmonic Performance:\n";
  
  // Group by band
  std::string currentBand = "";
  for (const auto& harmonic : harmonicSpecs) {
    if (harmonic.bandName != currentBand) {
      currentBand = harmonic.bandName;
      std::cout << "      " << currentBand << ":\n";
    }
    
    double freqHz = harmonic.harmonicFreqMhz * 1e6;
    double attenuation = -InterpolateAtFrequency(response, freqHz);
    
    if (harmonic.harmonicOrder == 1) {
      // Fundamental - show passband loss
      std::cout << "        F0 (" << harmonic.harmonicFreqMhz << " MHz): " 
                << std::setw(6) << std::fixed << std::setprecision(2) << attenuation << " dB loss";
      if (attenuation <= MaxPassbandLossDb) {
        std::cout << " ✓";
      } else {
        std::cout << " ✗ (>" << MaxPassbandLossDb << " dB)";
      }
      std::cout << "\n";
    } else {
      // Harmonic - show stopband attenuation
      double outputLevel = harmonic.levelDbc - attenuation;
      std::cout << "        H" << harmonic.harmonicOrder << " (" << harmonic.harmonicFreqMhz 
                << " MHz): " << std::setw(6) << std::fixed << std::setprecision(2) 
                << outputLevel << " dBc";
      if (outputLevel <= TargetOutputDbc) {
        std::cout << " ✓";
      } else {
        std::cout << " ✗ (>" << TargetOutputDbc << " dBc)";
      }
      std::cout << "\n";
    }
  }
}