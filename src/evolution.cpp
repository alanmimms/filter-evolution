#include "evolution.h"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <thread>
#include <future>

EvolutionaryOptimizer::EvolutionaryOptimizer(
    const EvolutionConfig& config,
    std::shared_ptr<FitnessEvaluator> evaluator)
  : config(config), fitnessEvaluator(evaluator) {
  std::random_device rd;
  rng.seed(rd());
}

CircuitGenome EvolutionaryOptimizer::Optimize() {
  std::cout << "Initializing population...\n";
  InitializePopulation();
  
  std::cout << "Starting evolution for " << config.generations << " generations\n";
  std::cout << std::string(90, '=') << "\n";
  std::cout << "Progress Legend:\n";
  std::cout << "  Best:      Lowest fitness score (lower is better)\n";
  std::cout << "  Avg:       Average fitness across population\n";
  std::cout << "  Valid:     Circuits with valid topology (input->output path)\n";
  std::cout << "  Comps:     Average active components in valid circuits\n";
  std::cout << "  Inductors: Average inductors in valid circuits\n";
  std::cout << std::string(90, '-') << "\n";
  
  for (int gen = 0; gen < config.generations; ++gen) {
    EvaluatePopulation();
    SortPopulation();
    
    if (gen % 5 == 0) {
      // Calculate statistics
      double avgFitness = 0.0;
      int validCircuits = 0;
      int totalActiveComponents = 0;
      int totalActiveInductors = 0;
      
      for (const auto& genome : population) {
        avgFitness += genome.fitness;
        if (genome.IsValid()) {
          validCircuits++;
          totalActiveComponents += genome.CountActiveComponents();
          totalActiveInductors += genome.CountActiveInductors();
        }
      }
      avgFitness /= population.size();
      
      double avgComponents = validCircuits > 0 ? (double)totalActiveComponents / validCircuits : 0;
      double avgInductors = validCircuits > 0 ? (double)totalActiveInductors / validCircuits : 0;
      
      std::cout << "Gen " << std::setw(4) << gen 
                << " | Best: " << std::setw(10) << std::fixed << std::setprecision(1) 
                << population[0].fitness
                << " | Avg: " << std::setw(10) << std::fixed << std::setprecision(1) << avgFitness
                << " | Valid: " << std::setw(3) << validCircuits << "/" << population.size()
                << " | Comps: " << std::setw(4) << std::fixed << std::setprecision(1) << avgComponents
                << " | Inductors: " << std::setw(4) << std::fixed << std::setprecision(1) << avgInductors
                << "\n";
      
      // Show best circuit details every 25 generations
      if (gen % 25 == 0 && population[0].IsValid()) {
        std::cout << "    Best circuit: " << population[0].CountActiveComponents() 
                  << " components (" << population[0].CountActiveInductors() << " inductors)\n";
        fitnessEvaluator->PrintDetailedPerformance(population[0]);
      }
    }
    
    NextGeneration();
  }
  
  // Final evaluation
  EvaluatePopulation();
  SortPopulation();
  
  std::cout << std::string(90, '=') << "\n";
  std::cout << "Evolution complete!\n";
  std::cout << "Best fitness: " << population[0].fitness << "\n";
  std::cout << "Active components: " << population[0].CountActiveComponents() << "\n";
  std::cout << "Active inductors: " << population[0].CountActiveInductors() << "\n\n";
  
  if (population[0].IsValid()) {
    std::cout << "FINAL PERFORMANCE ANALYSIS:\n";
    fitnessEvaluator->PrintDetailedPerformance(population[0]);
  } else {
    std::cout << "Warning: Best circuit is invalid!\n";
  }
  
  return population[0];
}

void EvolutionaryOptimizer::InitializePopulation() {
  population.clear();
  population.reserve(config.populationSize);
  
  // Add some seeded designs
  for (int stages = 3; stages <= 7; stages += 2) {
    population.push_back(CreateLadderSeed(stages));
  }
  
  // Fill rest with random
  while (population.size() < static_cast<size_t>(config.populationSize)) {
    CircuitGenome genome;
    genome.Randomize(rng);
    population.push_back(std::move(genome));
  }
}

void EvolutionaryOptimizer::EvaluatePopulation() {
  // Batch parallel evaluation to respect thread limit
  size_t numThreads = static_cast<size_t>(config.numThreads);
  size_t batchSize = numThreads;
  
  for (size_t start = 0; start < population.size(); start += batchSize) {
    size_t end = std::min(start + batchSize, population.size());
    std::vector<std::future<void>> futures;
    
    // Process batch of individuals
    for (size_t i = start; i < end; ++i) {
      futures.push_back(std::async(std::launch::async, [this, i]() {
        fitnessEvaluator->Evaluate(population[i]);
      }));
    }
    
    // Wait for this batch to complete before starting next
    for (auto& future : futures) {
      future.wait();
    }
  }
}

void EvolutionaryOptimizer::SortPopulation() {
  std::sort(population.begin(), population.end(),
            [](const CircuitGenome& a, const CircuitGenome& b) {
              return a.fitness < b.fitness;
            });
}

CircuitGenome& EvolutionaryOptimizer::TournamentSelect() {
  std::uniform_int_distribution<> dist(0, population.size() - 1);
  
  int bestIdx = dist(rng);
  double bestFitness = population[bestIdx].fitness;
  
  for (int i = 1; i < config.tournamentSize; ++i) {
    int idx = dist(rng);
    if (population[idx].fitness < bestFitness) {
      bestIdx = idx;
      bestFitness = population[idx].fitness;
    }
  }
  
  return population[bestIdx];
}

CircuitGenome EvolutionaryOptimizer::Crossover(const CircuitGenome& parent1,
                                                const CircuitGenome& parent2) {
  CircuitGenome offspring;
  std::uniform_real_distribution<> prob(0.0, 1.0);
  
  for (size_t i = 0; i < parent1.components.size(); ++i) {
    offspring.components[i] = prob(rng) < 0.5 ? 
                              parent1.components[i] : parent2.components[i];
  }
  
  return offspring;
}

void EvolutionaryOptimizer::NextGeneration() {
  std::vector<CircuitGenome> newPopulation;
  newPopulation.reserve(config.populationSize);
  
  // Elitism - keep best individuals
  for (int i = 0; i < config.eliteCount && i < static_cast<int>(population.size()); ++i) {
    newPopulation.push_back(population[i]);
  }
  
  // Generate offspring
  std::uniform_real_distribution<> prob(0.0, 1.0);
  
  while (newPopulation.size() < static_cast<size_t>(config.populationSize)) {
    CircuitGenome& parent1 = TournamentSelect();
    CircuitGenome& parent2 = TournamentSelect();
    
    CircuitGenome offspring;
    if (prob(rng) < config.crossoverRate) {
      offspring = Crossover(parent1, parent2);
    } else {
      offspring = parent1;
    }
    
    offspring.Mutate(config.mutationRate, rng);
    newPopulation.push_back(std::move(offspring));
  }
  
  population = std::move(newPopulation);
}

CircuitGenome EvolutionaryOptimizer::CreateLadderSeed(int stages) const {
  CircuitGenome genome;

  // E24 values to use for ladder: progression through decades
  // Inductors: 220nH, 470nH, 1uH, 2.2uH, etc.
  // Capacitors: 330pF, 680pF, 1.5nF, 3.3nF, etc.
  const double inductorValues[] = {220.0, 470.0, 1000.0, 2200.0, 4700.0};  // nH
  const double capacitorValues[] = {330.0, 680.0, 1500.0, 3300.0, 6800.0}; // pF

  // Create simple ladder network
  int compIdx = 0;
  for (int i = 0; i < stages && compIdx < CircuitGenome::MaxComponents - 1; ++i) {
    int currentNode = (i == 0) ? CircuitGenome::NodeInput :
                      CircuitGenome::FirstInternalNode + i - 1;
    int nextNode = (i == stages - 1) ? CircuitGenome::NodeOutput :
                   CircuitGenome::FirstInternalNode + i;

    // Series inductor with E24 value
    genome.components[compIdx++] = Component(
      ComponentType::Inductor,
      inductorValues[i % 5],
      currentNode, nextNode, true
    );

    // Shunt capacitor to ground with E24 value
    genome.components[compIdx++] = Component(
      ComponentType::Capacitor,
      capacitorValues[i % 5],
      nextNode,
      CircuitGenome::NodeGround, true
    );
  }

  // Deactivate remaining components
  for (int i = compIdx; i < CircuitGenome::MaxComponents; ++i) {
    genome.components[i].active = false;
  }

  return genome;
}