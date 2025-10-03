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
  // This method is deprecated - use EvaluateGenome() instead
  // Only kept for PrintDetailedPerformance
  if (!genome.IsValid()) {
    genome.fitness = -1e9;
    return genome.fitness;
  }

  // Evaluate genome (worker does simulation + fitness computation)
  EvaluationResult result = SpiceSimulator::GetInstance().EvaluateGenome(genome).get();
  genome = std::move(result.genome);
  return genome.fitness;
}

double FitnessEvaluator::ComputeFitness(CircuitGenome& genome, const SimulationResult& simResult) {
  // Higher fitness is better, negative fitness is penalty
  if (!genome.IsValid()) {
    genome.fitness = -1e9;
    return genome.fitness;
  }

  // Handle different failure types with negative fitness
  if (!simResult.IsSuccess()) {
    switch (simResult.failureType) {
      case SimulationFailureType::InvalidCircuit:
        // Circuit structure is invalid - high penalty
        genome.fitness = -1e8;
        return genome.fitness;

      case SimulationFailureType::SimulationError:
        // NGSpice couldn't simulate (convergence, etc.) - penalize but less than invalid
        genome.fitness = -1e7;
        return genome.fitness;

      case SimulationFailureType::InternalError:
        // Bug in our code or NGSpice - report and treat as very bad
        std::cerr << "Internal error: " << simResult.errorMessage << std::endl;
        genome.fitness = -1e9;
        return genome.fitness;

      default:
        genome.fitness = -1e9;
        return genome.fitness;
    }
  }

  const FrequencyResponse& response = simResult.response;
  double fitness = 0.0;  // Start at zero, add rewards, subtract penalties

  // Evaluate each harmonic specification
  for (const auto& harmonic : harmonicSpecs) {
    double freqHz = harmonic.harmonicFreqMhz * 1e6;
    double attenuation = -InterpolateAtFrequency(response, freqHz);

    if (harmonic.harmonicOrder == 1) {
      // Fundamental - check passband loss
      // Give reward for low insertion loss, heavy penalty for high loss
      if (attenuation <= MaxPassbandLossDb) {
        // Reward for meeting spec - more reward for lower loss
        fitness += 1000.0 * (MaxPassbandLossDb - attenuation);
      } else {
        // Heavy penalty for exceeding max passband loss
        // This is the critical issue - need to penalize heavily
        double excess = attenuation - MaxPassbandLossDb;
        fitness -= excess * 10000.0;  // Much heavier penalty
      }
    } else {
      // Harmonic - check stopband attenuation
      double outputLevel = harmonic.levelDbc - attenuation;

      if (outputLevel <= TargetOutputDbc) {
        // Reward for meeting harmonic suppression spec
        double margin = TargetOutputDbc - outputLevel;
        // Weight by harmonic order - 2nd harmonic is most critical
        double weight = 500.0 * (5.0 / harmonic.harmonicOrder);
        fitness += margin * weight;
      } else {
        // Penalty for not meeting harmonic suppression
        double shortfall = outputLevel - TargetOutputDbc;
        // Weight by harmonic order - 2nd harmonic is most critical
        double weight = 5000.0 * (5.0 / harmonic.harmonicOrder);
        fitness -= shortfall * weight;
      }
    }
  }

  // Inductor cost penalty (prefer fewer inductors)
  int inductorCount = genome.CountActiveInductors();
  fitness -= inductorCount * 100.0;

  genome.fitness = fitness;
  return fitness;
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

  // Evaluate genome to get simulation result
  CircuitGenome tempGenome = genome;  // Make a copy since EvaluateGenome moves it
  EvaluationResult result = SpiceSimulator::GetInstance().EvaluateGenome(std::move(tempGenome)).get();

  if (!result.simResult.IsSuccess()) {
    std::cout << "    Simulation failed: " << result.simResult.errorMessage << "\n";
    return;
  }

  const FrequencyResponse& response = result.simResult.response;
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
        std::cout << " ✗ (need better than " << MaxPassbandLossDb << " dB)";
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
        std::cout << " ✗ (need better than " << TargetOutputDbc << " dBc)";
      }
      std::cout << "\n";
    }
  }
}