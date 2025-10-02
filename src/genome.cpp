#include "genome.h"
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>

CircuitGenome::CircuitGenome() : fitness(std::numeric_limits<double>::max()) {
  components.reserve(MaxComponents);
  for (int i = 0; i < MaxComponents; ++i) {
    components.emplace_back();
  }
}

void CircuitGenome::Randomize(std::mt19937& rng) {
  std::uniform_real_distribution<> lDist(MinInductorNh, MaxInductorNh);
  std::uniform_real_distribution<> cDist(MinCapacitorPf, MaxCapacitorPf);
  std::uniform_int_distribution<> nodeDist(0, FirstInternalNode + NumInternalNodes - 1);
  std::uniform_int_distribution<> typeDist(0, 1);
  std::uniform_real_distribution<> activeDist(0.0, 1.0);
  
  for (auto& comp : components) {
    comp.type = typeDist(rng) == 0 ? ComponentType::Inductor : ComponentType::Capacitor;
    comp.value = comp.type == ComponentType::Inductor ? lDist(rng) : cDist(rng);
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
  std::uniform_real_distribution<> valueMult(0.5, 2.0);
  std::normal_distribution<> valueAdd(0.0, 0.1);
  std::uniform_int_distribution<> nodeDist(0, FirstInternalNode + NumInternalNodes - 1);
  
  for (auto& comp : components) {
    if (prob(rng) < mutationRate) {
      int mutationType = std::uniform_int_distribution<>(0, 4)(rng);
      
      switch (mutationType) {
        case 0:  // Value mutation - multiplicative
          comp.value *= valueMult(rng);
          if (comp.type == ComponentType::Inductor) {
            comp.value = std::clamp(comp.value, MinInductorNh, MaxInductorNh);
          } else {
            comp.value = std::clamp(comp.value, MinCapacitorPf, MaxCapacitorPf);
          }
          break;
          
        case 1:  // Value mutation - additive
          if (comp.type == ComponentType::Inductor) {
            comp.value += valueAdd(rng) * MaxInductorNh;
            comp.value = std::clamp(comp.value, MinInductorNh, MaxInductorNh);
          } else {
            comp.value += valueAdd(rng) * MaxCapacitorPf;
            comp.value = std::clamp(comp.value, MinCapacitorPf, MaxCapacitorPf);
          }
          break;
          
        case 2:  // Type flip
          comp.type = comp.type == ComponentType::Inductor ? 
                      ComponentType::Capacitor : ComponentType::Inductor;
          // Adjust value to appropriate range
          if (comp.type == ComponentType::Inductor) {
            comp.value = std::uniform_real_distribution<>(MinInductorNh, MaxInductorNh)(rng);
          } else {
            comp.value = std::uniform_real_distribution<>(MinCapacitorPf, MaxCapacitorPf)(rng);
          }
          break;
          
        case 3:  // Connection mutation
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
          
        case 4:  // Active toggle
          comp.active = !comp.active;
          break;
      }
    }
  }
}

std::string CircuitGenome::ToSpiceNetlist() const {
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
  netlist << ".control\n";
  netlist << "run\n";
  netlist << "print frequency vdb(output) vp(output) > output.txt\n";
  netlist << ".endc\n";
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