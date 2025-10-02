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
#include <sys/types.h>

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

// Worker process information
struct WorkerProcess {
  pid_t pid;
  int socket_fd;
  bool busy;

  WorkerProcess() : pid(-1), socket_fd(-1), busy(false) {}
};

class SpiceSimulator {
public:
  SpiceSimulator();
  ~SpiceSimulator();

  // Main API: submit a simulation and get a future result
  std::future<SimulationResult> RunAcAnalysis(const CircuitGenome& genome);

  // Get singleton instance
  static SpiceSimulator& GetInstance();

  // Set number of worker processes (call before first use)
  void SetWorkerCount(int numWorkers);

private:
  // Spawn worker processes
  void SpawnWorkers(int numWorkers);

  // Shutdown all workers
  void ShutdownWorkers();

  // Orchestrator thread that distributes work to processes
  void OrchestratorThread();

  // Find available worker (round-robin)
  int GetAvailableWorker();

  // Send job to worker process and get result
  SimulationResult SendToWorker(int workerIdx, const std::string& netlist);

  // Worker process entry point (runs in child process)
  static void WorkerProcessMain(int socket_fd);

  // Worker process simulation loop
  static void WorkerSimulationLoop(int socket_fd);

  // IPC: Send/receive data over socket
  static bool SendString(int fd, const std::string& str);
  static bool ReceiveString(int fd, std::string& str);
  static bool SendResult(int fd, const SimulationResult& result);
  static bool ReceiveResult(int fd, SimulationResult& result);

  // Worker process: Initialize ngspice library
  static void InitializeNgspice();

  // Worker process: Run a single simulation
  static SimulationResult RunSimulation(const std::string& netlist);

  // Worker process: Extract frequency response
  static bool ExtractFrequencyResponse(FrequencyResponse& response);

  // Callbacks for ngspice (static members)
  static int SendCharCallback(char* message, int id, void* user_data);
  static int SendStatCallback(char* message, int id, void* user_data);
  static int ControlledExitCallback(int status, NG_BOOL immediate, NG_BOOL quit, int id, void* user_data);
  static int SendDataCallback(pvecvaluesall data, int count, int id, void* user_data);
  static int SendInitDataCallback(pvecinfoall data, int id, void* user_data);
  static int BGThreadRunningCallback(NG_BOOL running, int id, void* user_data);

  // Worker processes
  std::vector<WorkerProcess> workers_;
  std::mutex workerMutex_;
  int nextWorkerIdx_;

  // Job queue for orchestrator
  std::queue<SimulationJob> jobQueue_;
  std::mutex queueMutex_;
  std::condition_variable queueCV_;

  // Orchestrator thread
  std::thread orchestratorThread_;
  std::atomic<bool> shutdown_;
};