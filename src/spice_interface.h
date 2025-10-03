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

// Need full definition for struct members
#include "genome.h"

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


// Configuration for evolution (needed by workers)
struct EvolutionConfig {
  int populationSize;
  int generations;
  int eliteCount;
  double mutationRate;
  double crossoverRate;
  int tournamentSize;
  int numThreads;
};

// Result from worker containing genome, fitness, and simulation data
struct EvaluationResult {
  CircuitGenome genome;
  double fitness;
  SimulationResult simResult;

  EvaluationResult() : fitness(-1e9) {}
};

// Internal job structure for the queue
struct SimulationJob {
  CircuitGenome genome;
  std::promise<EvaluationResult> result;
  bool generateOffspring;  // If true, worker creates genome via selection/crossover/mutation

  SimulationJob(CircuitGenome g, bool genOffspring = false)
    : genome(std::move(g)), generateOffspring(genOffspring) {}

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
  int stderr_fd;  // Pipe to capture stderr from worker process
  bool busy;

  WorkerProcess() : pid(-1), socket_fd(-1), stderr_fd(-1), busy(false) {}
};

class SpiceSimulator {
public:
  SpiceSimulator();
  ~SpiceSimulator();

  // Main API: submit a genome for evaluation and get back genome with fitness
  std::future<EvaluationResult> EvaluateGenome(CircuitGenome genome);

  // Get singleton instance
  static SpiceSimulator& GetInstance();

  // Set number of worker processes (call before first use)
  void SetWorkerCount(int numWorkers);

  // Set fitness evaluator (needed by worker threads)
  void SetFitnessEvaluator(std::shared_ptr<class FitnessEvaluator> evaluator);

  // Set evolution configuration (needed by worker threads)
  void SetEvolutionConfig(const EvolutionConfig& config);

  // Update shared population (called by orchestrator after sorting)
  void UpdatePopulation(const std::vector<CircuitGenome>& pop);

  // Request offspring from workers (workers do selection/crossover/mutation/simulation)
  std::vector<std::future<EvaluationResult>> GenerateOffspring(int count);

private:
  // Evolution operations used by worker threads
  CircuitGenome TournamentSelect(std::mt19937& rng);
  CircuitGenome Crossover(const CircuitGenome& parent1, const CircuitGenome& parent2, std::mt19937& rng);

private:
  // Spawn worker processes
  void SpawnWorkers(int numWorkers);

  // Shutdown all workers
  void ShutdownWorkers();

  // Worker threads that handle communication with worker processes
  void WorkerThread(int workerIdx);

  // Stderr monitor thread for filtering worker process errors
  void StderrMonitorThread(int workerIdx);

  // Filter and handle stderr message from worker process
  static void FilterStderrMessage(const std::string& message);

  // Send job to worker process and get result (blocking)
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
  std::atomic<int> nextWorkerIdx_;

  // Per-worker job queues (one queue per worker thread)
  std::vector<std::unique_ptr<std::queue<SimulationJob>>> perWorkerQueues_;
  std::vector<std::unique_ptr<std::mutex>> perWorkerMutexes_;
  std::vector<std::unique_ptr<std::condition_variable>> perWorkerCVs_;

  // Worker threads (one per worker process)
  std::vector<std::thread> workerThreads_;

  // Stderr monitor threads (one per worker process)
  std::vector<std::thread> stderrMonitorThreads_;

  std::atomic<bool> shutdown_;

  // Fitness evaluator shared by all worker threads
  std::shared_ptr<class FitnessEvaluator> fitnessEvaluator_;

  // Evolution configuration
  EvolutionConfig evolutionConfig_;
  bool evolutionConfigSet_;

  // Shared population (read-only for workers)
  std::vector<CircuitGenome> sharedPopulation_;
  std::mutex populationMutex_;
};