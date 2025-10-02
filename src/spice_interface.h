#pragma once
#include <vector>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <future>
#include <memory>
#include <atomic>

// Include ngspice types for callbacks
#include <ngspice/sharedspice.h>

// Forward declaration
class CircuitGenome;

struct FrequencyResponse {
  std::vector<double> frequencies;
  std::vector<double> magnitudeDb;
  std::vector<double> phaseRad;

  bool IsEmpty() const { return frequencies.empty(); }
};

// Enum to categorize simulation failures
enum class SimulationFailureType {
  Success,
  InvalidCircuit,        // Non-viable circuit (disconnected, etc.) - penalize fitness
  SimulationError,       // NGSpice couldn't converge this particular circuit - penalize fitness
  InternalError          // Bug in our code or NGSpice crash - should investigate
};

struct SimulationResult {
  FrequencyResponse response;
  SimulationFailureType failureType;
  std::string errorMessage;

  SimulationResult() : failureType(SimulationFailureType::Success) {}
  bool IsSuccess() const { return failureType == SimulationFailureType::Success; }
};

// Internal job structure for the queue
struct SimulationJob {
  std::string netlist;
  std::promise<SimulationResult> result;

  SimulationJob(std::string nl) : netlist(std::move(nl)) {}

  // Make movable but not copyable (because of promise)
  SimulationJob(SimulationJob&&) = default;
  SimulationJob& operator=(SimulationJob&&) = default;
  SimulationJob(const SimulationJob&) = delete;
  SimulationJob& operator=(const SimulationJob&) = delete;
};

class SpiceSimulator {
public:
  SpiceSimulator();
  ~SpiceSimulator();

  // Main API: submit a simulation and get a future result
  std::future<SimulationResult> RunAcAnalysis(const CircuitGenome& genome);

  // Get singleton instance
  static SpiceSimulator& GetInstance();

private:
  // Worker thread that runs all ngspice simulations sequentially
  void WorkerThread();

  // Initialize ngspice library (called once in worker thread)
  void InitializeNgspice();

  // Run a single simulation using ngspice library
  SimulationResult RunSimulation(const std::string& netlist);

  // Parse ngspice vector data into FrequencyResponse
  bool ExtractFrequencyResponse(FrequencyResponse& response);

  // Callbacks for ngspice (static members) - need to match ngspice API exactly
  static int SendCharCallback(char* message, int id, void* user_data);
  static int SendStatCallback(char* message, int id, void* user_data);
  static int ControlledExitCallback(int status, NG_BOOL immediate, NG_BOOL quit, int id, void* user_data);
  static int SendDataCallback(pvecvaluesall data, int count, int id, void* user_data);
  static int SendInitDataCallback(pvecinfoall data, int id, void* user_data);
  static int BGThreadRunningCallback(NG_BOOL running, int id, void* user_data);

  // Thread-safe job queue
  std::queue<SimulationJob> jobQueue_;
  std::mutex queueMutex_;
  std::condition_variable queueCV_;

  // Worker thread
  std::thread workerThread_;
  std::atomic<bool> shutdown_;

  // NGSpice initialization flag
  bool ngspiceInitialized_;

  // Capture output for debugging
  std::mutex outputMutex_;
  std::string lastOutput_;
};