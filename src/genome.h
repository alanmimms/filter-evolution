#pragma once
#include <vector>
#include <string>
#include <memory>
#include <random>
#include <set>
#include <queue>

enum class ComponentType {
  Inductor,
  Capacitor
};

struct Component {
  bool active;
  ComponentType type;
  double value;  // nH for L, pF for C
  int nodeA;
  int nodeB;
  
  Component() : active(true), type(ComponentType::Inductor), 
                value(100.0), nodeA(0), nodeB(1) {}
  
  Component(ComponentType t, double v, int a, int b, bool act = true)
    : active(act), type(t), value(v), nodeA(a), nodeB(b) {}
};

class CircuitGenome {
public:
  static constexpr int MaxComponents = 20;
  static constexpr int NumInternalNodes = 10;
  static constexpr int NodeInput = 0;
  static constexpr int NodeOutput = 1;
  static constexpr int NodeGround = 2;
  static constexpr int FirstInternalNode = 3;
  
  // Component value ranges
  static constexpr double MinInductorNh = 10.0;
  static constexpr double MaxInductorNh = 5000.0;
  static constexpr double MinCapacitorPf = 10.0;
  static constexpr double MaxCapacitorPf = 5000.0;
  
  std::vector<Component> components;
  double fitness;
  
  CircuitGenome();
  void Randomize(std::mt19937& rng);
  void Mutate(double mutationRate, std::mt19937& rng);
  std::string ToSpiceNetlist(const std::string& outputFile = "output.txt") const;
  bool IsValid() const;
  int CountActiveComponents() const;
  int CountActiveInductors() const;
  
private:
  bool HasPath(int from, int to) const;
  std::set<int> GetConnectedNodes() const;
};