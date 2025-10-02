#pragma once
#include <vector>
#include <random>
#include <memory>
#include "genome.h"
#include "fitness.h"

struct EvolutionConfig {
  int populationSize = 100;
  int generations = 500;
  int eliteCount = 5;
  double mutationRate = 0.15;
  double crossoverRate = 0.7;
  int tournamentSize = 5;
  int numThreads = 8;
};

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