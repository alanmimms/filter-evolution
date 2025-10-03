#pragma once
#include <vector>
#include <random>
#include <memory>
#include "genome.h"
#include "fitness.h"
#include "spice_interface.h"  // For EvolutionConfig

class EvolutionaryOptimizer {
public:
  EvolutionaryOptimizer(const EvolutionConfig& config,
                        std::shared_ptr<FitnessEvaluator> evaluator);
  
  CircuitGenome Optimize();
  
private:
  EvolutionConfig config;
  std::shared_ptr<FitnessEvaluator> fitnessEvaluator;
  std::mt19937 rng;
  std::vector<CircuitGenome> population;
  
  void InitializePopulation();
  void EvaluatePopulation();
  CircuitGenome& TournamentSelect();
  CircuitGenome Crossover(const CircuitGenome& parent1, 
                          const CircuitGenome& parent2);
  void NextGeneration();
  void SortPopulation();
  
  CircuitGenome CreateLadderSeed(int stages) const;
};