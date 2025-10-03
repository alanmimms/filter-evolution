#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include "genome.h"
#include "evolution.h"
#include "fitness.h"
#include "spice_interface.h"

std::vector<HarmonicSpec> LoadHarmonicData(const std::string& filename) {
  std::vector<HarmonicSpec> specs;
  std::ifstream file(filename);
  
  if (!file) {
    std::cerr << "Failed to open " << filename << "\n";
    return specs;
  }
  
  std::string line;
  std::getline(file, line);  // Skip header
  
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string band;
    HarmonicSpec spec;
    
    std::getline(iss, band, ',');
    spec.bandName = band;
    
    std::string temp;
    std::getline(iss, temp, ',');
    spec.fundamentalFreqMhz = std::stod(temp);
    
    std::getline(iss, temp, ',');
    spec.harmonicOrder = std::stoi(temp);
    
    std::getline(iss, temp, ',');
    spec.harmonicFreqMhz = std::stod(temp);
    
    std::getline(iss, temp, ',');
    spec.levelDbc = std::stod(temp);
    
    specs.push_back(spec);
  }
  
  return specs;
}

void printHelp(const char* programName) {
  std::cout << "Evolutionary Filter Designer for HF EER Transmitter\n";
  std::cout << "Usage: " << programName << " [OPTIONS] [harmonics_file]\n\n";
  std::cout << "Options:\n";
  std::cout << "  -h, --help              Show this help message\n";
  std::cout << "  -p, --population SIZE   Population size (default: 100)\n";
  std::cout << "  -g, --generations N     Number of generations (default: 500)\n";
  std::cout << "  -e, --elite N           Elite count for selection (default: 5)\n";
  std::cout << "  -m, --mutation RATE     Mutation rate 0.0-1.0 (default: 0.15)\n";
  std::cout << "  -c, --crossover RATE    Crossover rate 0.0-1.0 (default: 0.7)\n";
  std::cout << "  -t, --tournament SIZE   Tournament selection size (default: 5)\n";
  std::cout << "  -j, --threads N         Number of parallel threads (default: 8)\n";
  std::cout << "  -o, --output FILE       Output file for best circuit (default: best_filter.cir)\n\n";
  std::cout << "Arguments:\n";
  std::cout << "  harmonics_file          CSV file with harmonic specifications\n";
  std::cout << "                          (default: data/harmonics.csv)\n\n";
  std::cout << "Examples:\n";
  std::cout << "  " << programName << " --generations 1000 --population 200\n";
  std::cout << "  " << programName << " -g 100 -p 50 -m 0.2 my_harmonics.csv\n";
}

bool parseArguments(int argc, char* argv[], EvolutionConfig& config, 
                   std::string& dataFile, std::string& outputFile) {
  // Set defaults
  config.populationSize = 100;
  config.generations = 500;
  config.eliteCount = 5;
  config.mutationRate = 0.15;
  config.crossoverRate = 0.7;
  config.tournamentSize = 5;
  config.numThreads = 8;
  dataFile = "data/harmonics.csv";
  outputFile = "best_filter.cir";
  
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      printHelp(argv[0]);
      return false;
    }
    else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--population") == 0) {
      if (++i >= argc) {
        std::cerr << "Error: --population requires a value\n";
        return false;
      }
      config.populationSize = std::atoi(argv[i]);
      if (config.populationSize <= 0) {
        std::cerr << "Error: Population size must be positive\n";
        return false;
      }
    }
    else if (strcmp(argv[i], "-g") == 0 || strcmp(argv[i], "--generations") == 0) {
      if (++i >= argc) {
        std::cerr << "Error: --generations requires a value\n";
        return false;
      }
      config.generations = std::atoi(argv[i]);
      if (config.generations <= 0) {
        std::cerr << "Error: Generations must be positive\n";
        return false;
      }
    }
    else if (strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--elite") == 0) {
      if (++i >= argc) {
        std::cerr << "Error: --elite requires a value\n";
        return false;
      }
      config.eliteCount = std::atoi(argv[i]);
      if (config.eliteCount < 0) {
        std::cerr << "Error: Elite count cannot be negative\n";
        return false;
      }
    }
    else if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mutation") == 0) {
      if (++i >= argc) {
        std::cerr << "Error: --mutation requires a value\n";
        return false;
      }
      config.mutationRate = std::atof(argv[i]);
      if (config.mutationRate < 0.0 || config.mutationRate > 1.0) {
        std::cerr << "Error: Mutation rate must be between 0.0 and 1.0\n";
        return false;
      }
    }
    else if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--crossover") == 0) {
      if (++i >= argc) {
        std::cerr << "Error: --crossover requires a value\n";
        return false;
      }
      config.crossoverRate = std::atof(argv[i]);
      if (config.crossoverRate < 0.0 || config.crossoverRate > 1.0) {
        std::cerr << "Error: Crossover rate must be between 0.0 and 1.0\n";
        return false;
      }
    }
    else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--tournament") == 0) {
      if (++i >= argc) {
        std::cerr << "Error: --tournament requires a value\n";
        return false;
      }
      config.tournamentSize = std::atoi(argv[i]);
      if (config.tournamentSize <= 0) {
        std::cerr << "Error: Tournament size must be positive\n";
        return false;
      }
    }
    else if (strcmp(argv[i], "-j") == 0 || strcmp(argv[i], "--threads") == 0) {
      if (++i >= argc) {
        std::cerr << "Error: --threads requires a value\n";
        return false;
      }
      config.numThreads = std::atoi(argv[i]);
      if (config.numThreads <= 0) {
        std::cerr << "Error: Number of threads must be positive\n";
        return false;
      }
    }
    else if (strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--output") == 0) {
      if (++i >= argc) {
        std::cerr << "Error: --output requires a filename\n";
        return false;
      }
      outputFile = argv[i];
    }
    else if (argv[i][0] == '-') {
      std::cerr << "Error: Unknown option " << argv[i] << "\n";
      std::cerr << "Use --help for usage information\n";
      return false;
    }
    else {
      // Positional argument - harmonics file
      dataFile = argv[i];
    }
  }
  
  return true;
}

int main(int argc, char* argv[]) {
  EvolutionConfig config;
  std::string dataFile;
  std::string outputFile;
  
  // Parse command line arguments
  if (!parseArguments(argc, argv, config, dataFile, outputFile)) {
    return argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) ? 0 : 1;
  }
  
  std::cout << "Evolutionary Filter Designer for HF EER Transmitter\n";
  std::cout << "====================================================\n\n";
  
  // Display configuration
  std::cout << "Configuration:\n";
  std::cout << "  Data file:      " << dataFile << "\n";
  std::cout << "  Output file:    " << outputFile << "\n";
  std::cout << "  Population:     " << config.populationSize << "\n";
  std::cout << "  Generations:    " << config.generations << "\n";
  std::cout << "  Elite count:    " << config.eliteCount << "\n";
  std::cout << "  Mutation rate:  " << std::fixed << std::setprecision(3) << config.mutationRate << "\n";
  std::cout << "  Crossover rate: " << std::fixed << std::setprecision(3) << config.crossoverRate << "\n";
  std::cout << "  Tournament size:" << config.tournamentSize << "\n";
  std::cout << "  Threads:        " << config.numThreads << "\n\n";
  
  auto harmonics = LoadHarmonicData(dataFile);
  if (harmonics.empty()) {
    std::cerr << "No harmonic data loaded from " << dataFile << ". Exiting.\n";
    return 1;
  }
  
  std::cout << "Loaded " << harmonics.size() << " harmonic specifications\n";
  std::cout << "Covering " << harmonics.size() / 5 << " bands\n\n";

  // Initialize simulator with worker processes and fitness evaluator
  auto evaluator = std::make_shared<FitnessEvaluator>(harmonics);
  SpiceSimulator::GetInstance().SetWorkerCount(config.numThreads);
  SpiceSimulator::GetInstance().SetFitnessEvaluator(evaluator);

  EvolutionaryOptimizer optimizer(config, evaluator);
  CircuitGenome best = optimizer.Optimize();
  
  std::cout << "\n" << std::string(60, '=') << "\n";
  std::cout << "BEST FILTER DESIGN\n";
  std::cout << std::string(60, '=') << "\n\n";
  
  std::cout << "Fitness Score: " << best.fitness << "\n";
  std::cout << "Active Components: " << best.CountActiveComponents() << "\n";
  std::cout << "Active Inductors: " << best.CountActiveInductors() << "\n\n";
  
  std::cout << "SPICE Netlist:\n";
  std::cout << std::string(60, '-') << "\n";
  std::cout << best.ToSpiceNetlist() << "\n";
  
  std::ofstream outFile(outputFile);
  if (!outFile) {
    std::cerr << "Error: Could not write to " << outputFile << "\n";
    return 1;
  }
  outFile << best.ToSpiceNetlist();
  outFile.close();
  
  std::cout << "\nNetlist saved to: " << outputFile << "\n";
  
  return 0;
}