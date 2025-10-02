#include "genome.h"
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>

// Define static constexpr arrays
constexpr double CircuitGenome::E24Values[];
constexpr double CircuitGenome::InductorDecades[];
constexpr double CircuitGenome::CapacitorDecades[];

CircuitGenome::CircuitGenome() : fitness(std::numeric_limits<double>::max()) {
  components.reserve(MaxComponents);
  for (int i = 0; i < MaxComponents; ++i) {
    components.emplace_back();
  }
}

double CircuitGenome::GetE24Value(int valueIndex, int decadeIndex) {
  return E24Values[valueIndex % NumE24Values] *
         (valueIndex < NumE24Values ? InductorDecades[decadeIndex] : CapacitorDecades[decadeIndex]);
}

double CircuitGenome::GetRandomE24Inductor(std::mt19937& rng) {
  std::uniform_int_distribution<> valueDist(0, NumE24Values - 1);
  std::uniform_int_distribution<> decadeDist(0, NumInductorDecades - 1);
  return E24Values[valueDist(rng)] * InductorDecades[decadeDist(rng)];
}

double CircuitGenome::GetRandomE24Capacitor(std::mt19937& rng) {
  std::uniform_int_distribution<> valueDist(0, NumE24Values - 1);
  std::uniform_int_distribution<> decadeDist(0, NumCapacitorDecades - 1);
  return E24Values[valueDist(rng)] * CapacitorDecades[decadeDist(rng)];
}

double CircuitGenome::GetClosestE24Value(double value, bool isInductor) {
  const double* decades = isInductor ? InductorDecades : CapacitorDecades;
  int numDecades = isInductor ? NumInductorDecades : NumCapacitorDecades;

  double bestValue = 0.0;
  double minError = std::numeric_limits<double>::max();

  for (int d = 0; d < numDecades; ++d) {
    for (int v = 0; v < NumE24Values; ++v) {
      double candidate = E24Values[v] * decades[d];
      double error = std::abs(candidate - value);
      if (error < minError) {
        minError = error;
        bestValue = candidate;
      }
    }
  }

  return bestValue;
}

void CircuitGenome::Randomize(std::mt19937& rng) {
  std::uniform_int_distribution<> nodeDist(0, FirstInternalNode + NumInternalNodes - 1);
  std::uniform_int_distribution<> typeDist(0, 1);
  std::uniform_real_distribution<> activeDist(0.0, 1.0);

  for (auto& comp : components) {
    comp.type = typeDist(rng) == 0 ? ComponentType::Inductor : ComponentType::Capacitor;

    // Use E24 standard values
    comp.value = comp.type == ComponentType::Inductor ?
                 GetRandomE24Inductor(rng) : GetRandomE24Capacitor(rng);

    comp.nodeA = nodeDist(rng);
    comp.nodeB = nodeDist(rng);

    // Ensure different nodes
    while (comp.nodeA == comp.nodeB) {
      comp.nodeB = nodeDist(rng);
    }

    // 70% chance of being active initially
    comp.active = activeDist(rng) < 0.7;
  }
}

void CircuitGenome::Mutate(double mutationRate, std::mt19937& rng) {
  std::uniform_real_distribution<> prob(0.0, 1.0);
  std::uniform_int_distribution<> nodeDist(0, FirstInternalNode + NumInternalNodes - 1);
  std::uniform_int_distribution<> valueDist(0, NumE24Values - 1);

  for (auto& comp : components) {
    if (prob(rng) < mutationRate) {
      int mutationType = std::uniform_int_distribution<>(0, 5)(rng);

      switch (mutationType) {
        case 0:  // Change to adjacent E24 value in same decade
        {
          const double* decades = comp.type == ComponentType::Inductor ?
                                  InductorDecades : CapacitorDecades;
          int numDecades = comp.type == ComponentType::Inductor ?
                           NumInductorDecades : NumCapacitorDecades;

          // Find current decade and value index
          int currentDecade = 0;
          double currentBase = comp.value;
          for (int d = numDecades - 1; d >= 0; --d) {
            if (comp.value >= decades[d]) {
              currentDecade = d;
              currentBase = comp.value / decades[d];
              break;
            }
          }

          // Find closest E24 value
          int currentIndex = 0;
          double minDiff = std::abs(E24Values[0] - currentBase);
          for (int i = 1; i < NumE24Values; ++i) {
            double diff = std::abs(E24Values[i] - currentBase);
            if (diff < minDiff) {
              minDiff = diff;
              currentIndex = i;
            }
          }

          // Move to adjacent value
          int newIndex = currentIndex + (prob(rng) < 0.5 ? -1 : 1);
          newIndex = (newIndex + NumE24Values) % NumE24Values;
          comp.value = E24Values[newIndex] * decades[currentDecade];
          break;
        }

        case 1:  // Change decade
        {
          const double* decades = comp.type == ComponentType::Inductor ?
                                  InductorDecades : CapacitorDecades;
          int numDecades = comp.type == ComponentType::Inductor ?
                           NumInductorDecades : NumCapacitorDecades;

          // Find current value base
          double currentBase = 0;
          for (int d = numDecades - 1; d >= 0; --d) {
            if (comp.value >= decades[d]) {
              currentBase = comp.value / decades[d];
              break;
            }
          }

          // Snap to closest E24 value
          int valueIndex = 0;
          double minDiff = std::abs(E24Values[0] - currentBase);
          for (int i = 1; i < NumE24Values; ++i) {
            double diff = std::abs(E24Values[i] - currentBase);
            if (diff < minDiff) {
              minDiff = diff;
              valueIndex = i;
            }
          }

          // Change to random different decade
          std::uniform_int_distribution<> decadeDist(0, numDecades - 1);
          int newDecade = decadeDist(rng);
          comp.value = E24Values[valueIndex] * decades[newDecade];
          break;
        }

        case 2:  // Completely random E24 value
          comp.value = comp.type == ComponentType::Inductor ?
                       GetRandomE24Inductor(rng) : GetRandomE24Capacitor(rng);
          break;

        case 3:  // Type flip
          comp.type = comp.type == ComponentType::Inductor ?
                      ComponentType::Capacitor : ComponentType::Inductor;
          // Get new random E24 value for new type
          comp.value = comp.type == ComponentType::Inductor ?
                       GetRandomE24Inductor(rng) : GetRandomE24Capacitor(rng);
          break;

        case 4:  // Connection mutation
          if (prob(rng) < 0.5) {
            comp.nodeA = nodeDist(rng);
          } else {
            comp.nodeB = nodeDist(rng);
          }
          // Ensure different nodes
          while (comp.nodeA == comp.nodeB) {
            comp.nodeB = nodeDist(rng);
          }
          break;

        case 5:  // Active toggle
          comp.active = !comp.active;
          break;
      }
    }
  }
}

std::string CircuitGenome::ToSpiceNetlist(const std::string& outputFile) const {
  std::ostringstream netlist;

  netlist << "* Evolved Filter Circuit\n";
  netlist << "Vin in 0 AC 1\n";
  netlist << "Rin in input 50\n";

  auto nodeToName = [](int node) -> std::string {
    if (node == NodeInput) return "input";
    if (node == NodeOutput) return "output";
    if (node == NodeGround) return "0";
    return "n" + std::to_string(node);
  };

  int lCount = 0, cCount = 0;
  for (const auto& comp : components) {
    if (!comp.active) continue;

    std::string nodeA = nodeToName(comp.nodeA);
    std::string nodeB = nodeToName(comp.nodeB);

    if (comp.type == ComponentType::Inductor) {
      double valueH = comp.value * 1e-9;
      netlist << "L" << ++lCount << " " << nodeA << " " << nodeB
              << " " << std::scientific << valueH << "\n";
    } else {
      double valueF = comp.value * 1e-12;
      netlist << "C" << ++cCount << " " << nodeA << " " << nodeB
              << " " << std::scientific << valueF << "\n";
    }
  }

  netlist << "Rload output 0 50\n";
  netlist << ".ac dec 50 1e6 150e6\n";

  // For file output (used when saving best circuit)
  if (!outputFile.empty()) {
    netlist << ".control\n";
    netlist << "run\n";
    netlist << "print frequency vdb(output) vp(output) > " << outputFile << "\n";
    netlist << ".endc\n";
  }

  netlist << ".end\n";

  return netlist.str();
}

bool CircuitGenome::HasPath(int from, int to) const {
  std::set<int> visited;
  std::queue<int> toVisit;
  toVisit.push(from);
  visited.insert(from);
  
  while (!toVisit.empty()) {
    int current = toVisit.front();
    toVisit.pop();
    
    if (current == to) return true;
    
    for (const auto& comp : components) {
      if (!comp.active) continue;
      
      int next = -1;
      if (comp.nodeA == current) next = comp.nodeB;
      else if (comp.nodeB == current) next = comp.nodeA;
      
      if (next != -1 && visited.find(next) == visited.end()) {
        visited.insert(next);
        toVisit.push(next);
      }
    }
  }
  
  return false;
}

std::set<int> CircuitGenome::GetConnectedNodes() const {
  std::set<int> nodes;
  for (const auto& comp : components) {
    if (comp.active) {
      nodes.insert(comp.nodeA);
      nodes.insert(comp.nodeB);
    }
  }
  return nodes;
}

bool CircuitGenome::IsValid() const {
  // Must have at least 2 active components
  if (CountActiveComponents() < 2) return false;
  
  // Must have path from input to output
  if (!HasPath(NodeInput, NodeOutput)) return false;
  
  // Output must connect to ground (through load resistor path)
  if (!HasPath(NodeOutput, NodeGround)) return false;
  
  // No isolated nodes (except unused internal nodes)
  auto connectedNodes = GetConnectedNodes();
  if (connectedNodes.find(NodeInput) == connectedNodes.end()) return false;
  if (connectedNodes.find(NodeOutput) == connectedNodes.end()) return false;
  
  return true;
}

int CircuitGenome::CountActiveComponents() const {
  return std::count_if(components.begin(), components.end(),
                       [](const Component& c) { return c.active; });
}

int CircuitGenome::CountActiveInductors() const {
  return std::count_if(components.begin(), components.end(),
                       [](const Component& c) { 
                         return c.active && c.type == ComponentType::Inductor; 
                       });
}