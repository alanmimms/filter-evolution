#include "spice_interface.h"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <random>
#include <thread>

SpiceSimulator::SpiceSimulator() {
}

FrequencyResponse SpiceSimulator::RunAcAnalysis(const std::string& netlist) {
  FrequencyResponse response;
  
  // Create unique filenames to avoid conflicts between parallel threads
  static thread_local std::random_device rd;
  static thread_local std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(10000, 99999);
  
  std::string threadId = std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
  std::string uniqueId = threadId + "_" + std::to_string(dis(gen));
  
  std::string netlistFile = "temp_circuit_" + uniqueId + ".cir";
  std::string outputFile = "output_" + uniqueId + ".txt";
  std::string logFile = "simulation_" + uniqueId + ".log";
  
  // Write netlist to temp file
  {
    std::ofstream out(netlistFile);
    if (!out) {
      // Cleanup on failure
      std::filesystem::remove(netlistFile);
      return response;
    }
    out << netlist;
    // File automatically closed when out goes out of scope
  }
  
  // Run ngspice in batch mode (redirect output to suppress noise)
  std::string command = "ngspice -b " + netlistFile + " > /dev/null 2>&1";
  int result = std::system(command.c_str());
  
  if (result != 0) {
    // Cleanup and silently fail - this is expected for invalid circuits
    std::filesystem::remove(netlistFile);
    std::filesystem::remove(outputFile);
    std::filesystem::remove(logFile);
    return response;
  }
  
  // Parse output
  if (!ParseOutputFile(outputFile, response)) {
    // Silently fail - this is expected for invalid circuits
  }
  
  // Always cleanup, even if parsing failed
  std::filesystem::remove(netlistFile);
  std::filesystem::remove(outputFile);
  std::filesystem::remove(logFile);
  
  return response;
}

bool SpiceSimulator::ParseOutputFile(const std::string& filename, 
                                      FrequencyResponse& response) {
  {
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
    // File automatically closed when file goes out of scope
  }
  
  return !response.frequencies.empty();
}