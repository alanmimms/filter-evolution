#pragma once
#include <vector>
#include <string>

struct FrequencyResponse {
  std::vector<double> frequencies;
  std::vector<double> magnitudeDb;
  std::vector<double> phaseRad;
  
  bool IsEmpty() const { return frequencies.empty(); }
};

class SpiceSimulator {
public:
  SpiceSimulator();
  ~SpiceSimulator() = default;
  
  FrequencyResponse RunAcAnalysis(const std::string& netlist);
  
private:
  bool ParseOutputFile(const std::string& filename, FrequencyResponse& response);
};