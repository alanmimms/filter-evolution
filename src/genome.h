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

  // E24 series (2% tolerance) base values
  static constexpr double E24Values[] = {
    1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4,
    2.7, 3.0, 3.3, 3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2,
    6.8, 7.5, 8.2, 9.1
  };
  static constexpr int NumE24Values = 24;

  // Decade ranges for inductors (nH) and capacitors (pF)
  // Inductors: 10nH to 500uH (10, 100, 1000, 10000, 100000, 500000 nH decades)
  // Capacitors: 10pF to 100nF (10, 100, 1000, 10000, 100000 pF decades)
  static constexpr double InductorDecades[] = {10.0, 100.0, 1000.0, 10000.0, 100000.0, 500000.0};
  static constexpr double CapacitorDecades[] = {10.0, 100.0, 1000.0, 10000.0, 100000.0};
  static constexpr int NumInductorDecades = 6;
  static constexpr int NumCapacitorDecades = 5;
  static constexpr int NumDecades = 6;  // Max of the two

  std::vector<Component> components;
  double fitness;  // Higher is better

  CircuitGenome();
  void Randomize(std::mt19937& rng);
  void Mutate(double mutationRate, std::mt19937& rng);
  std::string ToSpiceNetlist(const std::string& outputFile = "output.txt") const;
  bool IsValid() const;
  int CountActiveComponents() const;
  int CountActiveInductors() const;

  // Helper to get E24 value
  static double GetE24Value(int valueIndex, int decadeIndex);
  static double GetRandomE24Inductor(std::mt19937& rng);
  static double GetRandomE24Capacitor(std::mt19937& rng);
  static double GetClosestE24Value(double value, bool isInductor);

private:
  bool HasPath(int from, int to) const;
  std::set<int> GetConnectedNodes() const;
};