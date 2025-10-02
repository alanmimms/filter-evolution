#include "spice_interface.h"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <algorithm>

SpiceSimulator::SpiceSimulator() {
}

FrequencyResponse SpiceSimulator::RunAcAnalysis(const std::string& netlist) {
  FrequencyResponse response;
  
  // Write netlist to temp file
  std::string netlistFile = "temp_circuit.cir";
  std::ofstream out(netlistFile);
  if (!out) {
    std::cerr << "Failed to write netlist file\n";
    return response;
  }
  out << netlist;
  out.close();
  
  // Run ngspice in batch mode (redirect output to suppress noise)
  std::string command = "ngspice -b " + netlistFile + " > /dev/null 2>&1";
  int result = std::system(command.c_str());
  
  if (result != 0) {
    // Silently fail - this is expected for invalid circuits
    return response;
  }
  
  // Parse output
  if (!ParseOutputFile("output.txt", response)) {
    // Silently fail - this is expected for invalid circuits
  }
  
  // Cleanup
  std::filesystem::remove(netlistFile);
  std::filesystem::remove("output.txt");
  std::filesystem::remove("simulation.log");
  
  return response;
}

bool SpiceSimulator::ParseOutputFile(const std::string& filename, 
                                      FrequencyResponse& response) {
  std::ifstream file(filename);
  if (!file) return false;
  
  std::string line;
  bool dataStarted = false;
  
  while (std::getline(file, line)) {
    // Skip header and separator lines
    if (line.find("Index") != std::string::npos || 
        line.find("---") != std::string::npos ||
        line.find("Analysis") != std::string::npos ||
        line.empty() || 
        line[0] == '*') {
      if (line.find("Index") != std::string::npos) {
        dataStarted = true;
      }
      continue;
    }
    
    if (!dataStarted) continue;
    
    // Replace tabs with spaces for easier parsing
    std::replace(line.begin(), line.end(), '\t', ' ');
    
    // Parse: Index frequency magnitude phase
    std::istringstream iss(line);
    int index;
    double freq, mag, phase;
    
    if (iss >> index >> freq >> mag >> phase) {
      response.frequencies.push_back(freq);
      response.magnitudeDb.push_back(mag);
      response.phaseRad.push_back(phase);
    }
  }
  
  return !response.frequencies.empty();
}