#include "spice_interface.h"
#include "genome.h"
#include <ngspice/sharedspice.h>
#include <iostream>
#include <cstring>
#include <sstream>
#include <algorithm>

// Callback implementations (static functions)
int SpiceSimulator::SendCharCallback(char* message, int id, void* user_data) {
  (void)message; (void)id; (void)user_data;
  // Suppress output
  return 0;
}

int SpiceSimulator::SendStatCallback(char* message, int id, void* user_data) {
  (void)message; (void)id; (void)user_data;  // Suppress warnings
  return 0;
}

int SpiceSimulator::ControlledExitCallback(int status, NG_BOOL immediate, NG_BOOL quit, int id, void* user_data) {
  (void)immediate; (void)id; (void)user_data;  // Suppress warnings
  if (!quit) {
    std::cerr << "NGSpice error exit with status " << status << std::endl;
  }
  return 0;
}

int SpiceSimulator::SendDataCallback(pvecvaluesall data, int count, int id, void* user_data) {
  (void)data; (void)count; (void)id; (void)user_data;  // Suppress warnings
  return 0;
}

int SpiceSimulator::SendInitDataCallback(pvecinfoall data, int id, void* user_data) {
  (void)data; (void)id; (void)user_data;  // Suppress warnings
  return 0;
}

int SpiceSimulator::BGThreadRunningCallback(NG_BOOL running, int id, void* user_data) {
  (void)running; (void)id; (void)user_data;  // Suppress warnings
  return 0;
}

// Singleton instance
SpiceSimulator& SpiceSimulator::GetInstance() {
  static SpiceSimulator instance;
  return instance;
}

SpiceSimulator::SpiceSimulator()
  : shutdown_(false), ngspiceInitialized_(false) {
  // Start the worker thread
  workerThread_ = std::thread(&SpiceSimulator::WorkerThread, this);
}

SpiceSimulator::~SpiceSimulator() {
  // Signal shutdown
  {
    std::lock_guard<std::mutex> lock(queueMutex_);
    shutdown_ = true;
  }
  queueCV_.notify_all();

  // Wait for worker thread to finish
  if (workerThread_.joinable()) {
    workerThread_.join();
  }
}

std::future<SimulationResult> SpiceSimulator::RunAcAnalysis(const CircuitGenome& genome) {
  // Generate netlist (no output file needed for in-memory)
  std::string netlist = genome.ToSpiceNetlist("");

  // Create job with promise/future
  SimulationJob job(std::move(netlist));
  auto future = job.result.get_future();

  // Add to queue
  {
    std::lock_guard<std::mutex> lock(queueMutex_);
    jobQueue_.push(std::move(job));
  }

  // Notify worker
  queueCV_.notify_one();

  return future;
}

void SpiceSimulator::WorkerThread() {
  // Initialize ngspice once in this thread
  InitializeNgspice();

  while (true) {
    SimulationJob job("");

    // Wait for a job or shutdown signal
    {
      std::unique_lock<std::mutex> lock(queueMutex_);
      queueCV_.wait(lock, [this] {
        return shutdown_ || !jobQueue_.empty();
      });

      if (shutdown_ && jobQueue_.empty()) {
        break;
      }

      if (!jobQueue_.empty()) {
        job = std::move(jobQueue_.front());
        jobQueue_.pop();
      } else {
        continue;
      }
    }

    // Process the job
    SimulationResult result = RunSimulation(job.netlist);

    // Send result back via promise
    job.result.set_value(std::move(result));
  }
}

void SpiceSimulator::InitializeNgspice() {
  int result = ngSpice_Init(
    SendCharCallback,
    SendStatCallback,
    ControlledExitCallback,
    SendDataCallback,
    SendInitDataCallback,
    BGThreadRunningCallback,
    nullptr
  );

  if (result != 0) {
    std::cerr << "Failed to initialize NGSpice library" << std::endl;
    ngspiceInitialized_ = false;
  } else {
    ngspiceInitialized_ = true;
  }
}

SimulationResult SpiceSimulator::RunSimulation(const std::string& netlist) {
  SimulationResult result;

  if (!ngspiceInitialized_) {
    result.failureType = SimulationFailureType::InternalError;
    result.errorMessage = "NGSpice not initialized";
    return result;
  }

  // Convert netlist string to array of char* for ngSpice_Circ
  std::vector<char*> circuitLines;
  std::istringstream stream(netlist);
  std::string line;
  std::vector<std::string> lineStorage;

  while (std::getline(stream, line)) {
    if (!line.empty()) {
      lineStorage.push_back(line);
    }
  }

  for (auto& stored : lineStorage) {
    circuitLines.push_back(const_cast<char*>(stored.c_str()));
  }
  circuitLines.push_back(nullptr);

  // Load circuit
  int circ_result = ngSpice_Circ(circuitLines.data());

  if (circ_result != 0) {
    result.failureType = SimulationFailureType::InvalidCircuit;
    result.errorMessage = "Failed to parse circuit";
    return result;
  }

  // Run AC analysis - need to use bg_run for background execution
  int ac_cmd_result = ngSpice_Command(const_cast<char*>("ac dec 50 1e6 150e6"));
  if (ac_cmd_result != 0) {
    result.failureType = SimulationFailureType::SimulationError;
    result.errorMessage = "AC analysis setup failed";
    ngSpice_Command(const_cast<char*>("destroy all"));
    return result;
  }

  // Actually run the simulation
  int run_result = ngSpice_Command(const_cast<char*>("run"));
  if (run_result != 0) {
    result.failureType = SimulationFailureType::SimulationError;
    result.errorMessage = "Simulation run failed";
    ngSpice_Command(const_cast<char*>("destroy all"));
    return result;
  }

  // Wait a tiny bit for simulation to finish (it should be fast)
  // Note: In production, we should use callbacks, but for now this works
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Extract results
  if (!ExtractFrequencyResponse(result.response)) {
    result.failureType = SimulationFailureType::SimulationError;
    result.errorMessage = "Failed to extract simulation results";
    ngSpice_Command(const_cast<char*>("destroy all"));
    return result;
  }

  result.failureType = SimulationFailureType::Success;

  // Clean up for next simulation
  ngSpice_Command(const_cast<char*>("destroy all"));

  return result;
}

bool SpiceSimulator::ExtractFrequencyResponse(FrequencyResponse& response) {
  // Get current plot name
  char* plotname = ngSpice_CurPlot();
  if (!plotname) {
    return false;
  }

  // Construct vector names with plot prefix
  std::string freqVecName = std::string(plotname) + ".frequency";
  std::string outVecName = std::string(plotname) + ".output";

  // Get the frequency vector
  pvector_info freqVec = ngGet_Vec_Info(const_cast<char*>(freqVecName.c_str()));
  if (!freqVec || freqVec->v_length == 0) {
    return false;
  }

  // Get the output voltage vector (complex)
  pvector_info outVec = ngGet_Vec_Info(const_cast<char*>(outVecName.c_str()));
  if (!outVec || outVec->v_length == 0) {
    return false;
  }

  // Check if we have complex data for output
  if (!outVec->v_compdata) {
    return false;
  }

  // Check if we have the right number of points
  if (freqVec->v_length != outVec->v_length) {
    return false;
  }

  int numPoints = freqVec->v_length;
  response.frequencies.reserve(numPoints);
  response.magnitudeDb.reserve(numPoints);
  response.phaseRad.reserve(numPoints);

  // Extract data
  for (int i = 0; i < numPoints; i++) {
    // Frequency can be in v_realdata OR v_compdata (ngspice uses compdata for AC freq)
    double freq;
    if (freqVec->v_realdata) {
      freq = freqVec->v_realdata[i];
    } else if (freqVec->v_compdata) {
      freq = freqVec->v_compdata[i].cx_real;  // Take real part
    } else {
      return false;
    }
    response.frequencies.push_back(freq);

    double real = outVec->v_compdata[i].cx_real;
    double imag = outVec->v_compdata[i].cx_imag;

    // Convert to magnitude (dB) and phase (radians)
    double magnitude = std::sqrt(real * real + imag * imag);
    double magnitudeDb = 20.0 * std::log10(magnitude + 1e-20); // Avoid log(0)
    double phaseRad = std::atan2(imag, real);

    response.magnitudeDb.push_back(magnitudeDb);
    response.phaseRad.push_back(phaseRad);
  }

  return !response.frequencies.empty();
}
