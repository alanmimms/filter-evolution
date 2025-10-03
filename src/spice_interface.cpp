#include "spice_interface.h"
#include "genome.h"
#include "fitness.h"
#include <ngspice/sharedspice.h>
#include <iostream>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

// Thread-local flag to track if current simulation has errors
static thread_local bool g_simulationHasError = false;

// Callback implementations (static functions)
int SpiceSimulator::SendCharCallback(char* message, int id, void* user_data) {
  (void)id; (void)user_data;

  if (!message) return 0;

  // Check for critical errors that indicate invalid circuits
  if (strstr(message, "singular matrix") ||
      strstr(message, "Pivot") ||
      strstr(message, "gmin stepping failed") ||
      strstr(message, "source stepping failed") ||
      strstr(message, "Transient op failed") ||
      strstr(message, "operating point") ||
      strstr(message, "AC operating point failed")) {
    g_simulationHasError = true;
    return 0;  // Suppress these known errors
  }

  // Check for warnings we want to suppress
  if (strstr(message, "has no value, DC 0 assumed") ||
      strstr(message, "No compatibility mode selected")) {
    return 0;  // Suppress these benign messages
  }

  // If we get here, it's an unexpected message - print it so we can add handling
  if (strstr(message, "Error") || strstr(message, "error") ||
      strstr(message, "Warning") || strstr(message, "warning") ||
      strstr(message, "Failed") || strstr(message, "failed")) {
    std::cerr << "NGSpice message: " << message;
    if (message[strlen(message)-1] != '\n') {
      std::cerr << '\n';
    }
  }

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
  : nextWorkerIdx_(0), shutdown_(false), evolutionConfigSet_(false) {
  // Don't spawn workers yet - wait for SetWorkerCount()
}

SpiceSimulator::~SpiceSimulator() {
  ShutdownWorkers();
}

void SpiceSimulator::SetWorkerCount(int numWorkers) {
  if (!workers_.empty()) {
    return;  // Already initialized
  }

  if (numWorkers <= 0) {
    numWorkers = std::thread::hardware_concurrency();
    if (numWorkers == 0) numWorkers = 4;
  }

  SpawnWorkers(numWorkers);

  // Initialize per-worker queues and synchronization primitives
  perWorkerQueues_.reserve(numWorkers);
  perWorkerMutexes_.reserve(numWorkers);
  perWorkerCVs_.reserve(numWorkers);
  for (int i = 0; i < numWorkers; i++) {
    perWorkerQueues_.push_back(std::make_unique<std::queue<SimulationJob>>());
    perWorkerMutexes_.push_back(std::make_unique<std::mutex>());
    perWorkerCVs_.push_back(std::make_unique<std::condition_variable>());
  }

  // Start one worker thread per worker process
  workerThreads_.reserve(numWorkers);
  stderrMonitorThreads_.reserve(numWorkers);
  for (int i = 0; i < numWorkers; i++) {
    workerThreads_.emplace_back(&SpiceSimulator::WorkerThread, this, i);
    stderrMonitorThreads_.emplace_back(&SpiceSimulator::StderrMonitorThread, this, i);
  }
}

void SpiceSimulator::SpawnWorkers(int numWorkers) {
  workers_.resize(numWorkers);

  // Flush all output buffers before forking to avoid duplicated output
  std::cout.flush();
  std::cerr.flush();
  fflush(stdout);
  fflush(stderr);

  for (int i = 0; i < numWorkers; i++) {
    // Create socketpair for IPC
    int sockets[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) < 0) {
      std::cerr << "Failed to create socketpair for worker " << i << std::endl;
      continue;
    }

    // Create pipe for capturing stderr
    int stderr_pipe[2];
    if (pipe(stderr_pipe) < 0) {
      std::cerr << "Failed to create stderr pipe for worker " << i << std::endl;
      close(sockets[0]);
      close(sockets[1]);
      continue;
    }

    pid_t pid = fork();
    if (pid < 0) {
      std::cerr << "Failed to fork worker " << i << std::endl;
      close(sockets[0]);
      close(sockets[1]);
      close(stderr_pipe[0]);
      close(stderr_pipe[1]);
      continue;
    }

    if (pid == 0) {
      // Child process
      close(sockets[0]);  // Close parent end
      close(stderr_pipe[0]);  // Close read end of stderr pipe

      // Redirect stderr to pipe
      dup2(stderr_pipe[1], STDERR_FILENO);
      close(stderr_pipe[1]);

      // Close all other worker sockets inherited from parent
      for (int j = 0; j < i; j++) {
        if (workers_[j].socket_fd >= 0) {
          close(workers_[j].socket_fd);
        }
        if (workers_[j].stderr_fd >= 0) {
          close(workers_[j].stderr_fd);
        }
      }

      WorkerProcessMain(sockets[1]);
      _exit(0);  // Worker should never return
    }

    // Parent process
    close(sockets[1]);  // Close child end
    close(stderr_pipe[1]);  // Close write end of stderr pipe
    workers_[i].pid = pid;
    workers_[i].socket_fd = sockets[0];
    workers_[i].stderr_fd = stderr_pipe[0];
    workers_[i].busy = false;
  }

  std::cout << "Spawned " << numWorkers << " worker processes for parallel simulation\n";
}

void SpiceSimulator::ShutdownWorkers() {
  shutdown_ = true;

  // Notify all per-worker queues
  for (auto& cv : perWorkerCVs_) {
    cv->notify_all();
  }

  // Wait for all worker threads to finish
  for (auto& thread : workerThreads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  workerThreads_.clear();

  // Shutdown worker processes
  for (auto& worker : workers_) {
    if (worker.pid > 0) {
      // Send empty string to signal shutdown
      SendString(worker.socket_fd, "");
      close(worker.socket_fd);

      // Close stderr pipe (will cause monitor thread to exit)
      if (worker.stderr_fd >= 0) {
        close(worker.stderr_fd);
      }

      // Wait for worker process to exit
      int status;
      waitpid(worker.pid, &status, 0);
    }
  }

  // Wait for stderr monitor threads to finish
  for (auto& thread : stderrMonitorThreads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  stderrMonitorThreads_.clear();

  workers_.clear();
}

void SpiceSimulator::SetFitnessEvaluator(std::shared_ptr<FitnessEvaluator> evaluator) {
  fitnessEvaluator_ = evaluator;
}

std::future<EvaluationResult> SpiceSimulator::EvaluateGenome(CircuitGenome genome) {
  // Create job with genome
  SimulationJob job(std::move(genome));
  auto future = job.result.get_future();

  // Round-robin assign to worker
  int workerIdx = nextWorkerIdx_.fetch_add(1, std::memory_order_relaxed) % workers_.size();

  // Add to this worker's queue
  {
    std::lock_guard<std::mutex> lock(*perWorkerMutexes_[workerIdx]);
    perWorkerQueues_[workerIdx]->push(std::move(job));
  }

  // Notify this specific worker
  perWorkerCVs_[workerIdx]->notify_one();

  return future;
}

void SpiceSimulator::WorkerThread(int workerIdx) {
  if (workerIdx < 0 || workerIdx >= (int)workers_.size()) {
    return;
  }

  // Thread-local RNG for evolution operations
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_real_distribution<> prob(0.0, 1.0);

  // Infinite loop: wait for job, process it, return result, repeat
  while (!shutdown_) {
    CircuitGenome emptyGenome;
    SimulationJob job(std::move(emptyGenome));

    // Wait for a job from this worker's queue
    {
      std::unique_lock<std::mutex> lock(*perWorkerMutexes_[workerIdx]);
      perWorkerCVs_[workerIdx]->wait(lock, [this, workerIdx] {
        return shutdown_ || !perWorkerQueues_[workerIdx]->empty();
      });

      if (shutdown_) {
        break;
      }

      if (!perWorkerQueues_[workerIdx]->empty()) {
        job = std::move(perWorkerQueues_[workerIdx]->front());
        perWorkerQueues_[workerIdx]->pop();
      } else {
        continue;
      }
    }

    // If this is a GenerateOffspring request, worker creates the genome
    // via selection, crossover, mutation
    if (job.generateOffspring && evolutionConfigSet_) {
      // Tournament selection to pick parents
      CircuitGenome parent1 = TournamentSelect(rng);
      CircuitGenome parent2 = TournamentSelect(rng);

      // Crossover
      CircuitGenome offspring;
      if (prob(rng) < evolutionConfig_.crossoverRate) {
        offspring = Crossover(parent1, parent2, rng);
      } else {
        offspring = parent1;
      }

      // Mutation
      offspring.Mutate(evolutionConfig_.mutationRate, rng);

      // Now use this offspring for simulation
      job.genome = std::move(offspring);
    }

    // Generate netlist from genome
    std::string netlist = job.genome.ToSpiceNetlist("");

    // Run simulation via worker process
    SimulationResult simResult = SendToWorker(workerIdx, netlist);

    // Compute fitness from simulation result
    EvaluationResult evalResult;
    evalResult.genome = std::move(job.genome);
    evalResult.simResult = simResult;  // Store simulation result for debugging/printing
    if (fitnessEvaluator_) {
      evalResult.fitness = fitnessEvaluator_->ComputeFitness(evalResult.genome, simResult);
    } else {
      evalResult.fitness = -1e9;  // No evaluator
    }

    // Return result via promise
    job.result.set_value(std::move(evalResult));
  }
}

SimulationResult SpiceSimulator::SendToWorker(int workerIdx, const std::string& netlist) {
  SimulationResult result;

  if (workerIdx < 0 || workerIdx >= (int)workers_.size()) {
    result.failureType = SimulationFailureType::InternalError;
    result.errorMessage = "Invalid worker index";
    return result;
  }

  WorkerProcess& worker = workers_[workerIdx];

  // Send netlist to worker process
  if (!SendString(worker.socket_fd, netlist)) {
    result.failureType = SimulationFailureType::InternalError;
    result.errorMessage = "Failed to send netlist to worker";
    return result;
  }

  // Receive result from worker process
  if (!ReceiveResult(worker.socket_fd, result)) {
    result.failureType = SimulationFailureType::InternalError;
    result.errorMessage = "Failed to receive result from worker";
    return result;
  }

  return result;
}

// IPC functions
bool SpiceSimulator::SendString(int fd, const std::string& str) {
  uint32_t len = str.length();
  if (write(fd, &len, sizeof(len)) != sizeof(len)) return false;
  if (len > 0 && write(fd, str.data(), len) != (ssize_t)len) return false;
  return true;
}

bool SpiceSimulator::ReceiveString(int fd, std::string& str) {
  uint32_t len;
  if (read(fd, &len, sizeof(len)) != sizeof(len)) return false;
  if (len == 0) {
    str.clear();
    return true;
  }
  if (len > 10000000) return false;  // Sanity check

  str.resize(len);
  if (read(fd, &str[0], len) != (ssize_t)len) return false;
  return true;
}

bool SpiceSimulator::SendResult(int fd, const SimulationResult& result) {
  // Send failure type
  uint32_t failType = static_cast<uint32_t>(result.failureType);
  if (write(fd, &failType, sizeof(failType)) != sizeof(failType)) return false;

  // Send error message
  if (!SendString(fd, result.errorMessage)) return false;

  // Send frequency response
  uint32_t numPoints = result.response.frequencies.size();
  if (write(fd, &numPoints, sizeof(numPoints)) != sizeof(numPoints)) return false;

  if (numPoints > 0) {
    if (write(fd, result.response.frequencies.data(), numPoints * sizeof(double)) != (ssize_t)(numPoints * sizeof(double))) return false;
    if (write(fd, result.response.magnitudeDb.data(), numPoints * sizeof(double)) != (ssize_t)(numPoints * sizeof(double))) return false;
    if (write(fd, result.response.phaseRad.data(), numPoints * sizeof(double)) != (ssize_t)(numPoints * sizeof(double))) return false;
  }

  return true;
}

bool SpiceSimulator::ReceiveResult(int fd, SimulationResult& result) {
  // Receive failure type
  uint32_t failType;
  if (read(fd, &failType, sizeof(failType)) != sizeof(failType)) return false;
  result.failureType = static_cast<SimulationFailureType>(failType);

  // Receive error message
  if (!ReceiveString(fd, result.errorMessage)) return false;

  // Receive frequency response
  uint32_t numPoints;
  if (read(fd, &numPoints, sizeof(numPoints)) != sizeof(numPoints)) return false;

  if (numPoints > 0) {
    result.response.frequencies.resize(numPoints);
    result.response.magnitudeDb.resize(numPoints);
    result.response.phaseRad.resize(numPoints);

    if (read(fd, result.response.frequencies.data(), numPoints * sizeof(double)) != (ssize_t)(numPoints * sizeof(double))) return false;
    if (read(fd, result.response.magnitudeDb.data(), numPoints * sizeof(double)) != (ssize_t)(numPoints * sizeof(double))) return false;
    if (read(fd, result.response.phaseRad.data(), numPoints * sizeof(double)) != (ssize_t)(numPoints * sizeof(double))) return false;
  }

  return true;
}

void SpiceSimulator::StderrMonitorThread(int workerIdx) {
  if (workerIdx < 0 || workerIdx >= (int)workers_.size()) {
    return;
  }

  int stderr_fd = workers_[workerIdx].stderr_fd;
  if (stderr_fd < 0) return;

  char buffer[4096];
  std::string line;

  while (!shutdown_) {
    ssize_t n = read(stderr_fd, buffer, sizeof(buffer) - 1);
    if (n <= 0) {
      // Pipe closed or error
      break;
    }

    buffer[n] = '\0';

    // Process buffer line by line
    for (ssize_t i = 0; i < n; i++) {
      if (buffer[i] == '\n') {
        if (!line.empty()) {
          FilterStderrMessage(line);
          line.clear();
        }
      } else {
        line += buffer[i];
      }
    }
  }

  // Process any remaining partial line
  if (!line.empty()) {
    FilterStderrMessage(line);
  }
}

void SpiceSimulator::FilterStderrMessage(const std::string& message) {
  // Filter out known spam messages
  if (message.find("Pivot for step") != std::string::npos ||
      message.find("spfactor.c") != std::string::npos ||
      message.find("singular matrix") != std::string::npos ||
      message.find("gmin stepping") != std::string::npos ||
      message.find("source stepping") != std::string::npos ||
      message.find("Transient op failed") != std::string::npos ||
      message.find("operating point") != std::string::npos ||
      message.find("has no value, DC 0 assumed") != std::string::npos ||
      message.find("No compatibility mode") != std::string::npos) {
    // Suppress these known messages
    return;
  }

  // If we get here, it's an unexpected message - print it
  if (message.find("error") != std::string::npos ||
      message.find("Error") != std::string::npos ||
      message.find("warning") != std::string::npos ||
      message.find("Warning") != std::string::npos ||
      message.find("failed") != std::string::npos ||
      message.find("Failed") != std::string::npos) {
    std::cerr << "NGSpice stderr: " << message << std::endl;
  }
}

// Worker process entry point
void SpiceSimulator::WorkerProcessMain(int socket_fd) {
  // stderr is already redirected to pipe by parent process

  // Initialize ngspice in worker process
  InitializeNgspice();

  // Run simulation loop
  WorkerSimulationLoop(socket_fd);

  close(socket_fd);
}

void SpiceSimulator::WorkerSimulationLoop(int socket_fd) {
  while (true) {
    // Receive netlist
    std::string netlist;
    if (!ReceiveString(socket_fd, netlist)) {
      break;  // Connection closed or error
    }

    if (netlist.empty()) {
      break;  // Shutdown signal
    }

    // Run simulation
    SimulationResult result = RunSimulation(netlist);

    // Send result back
    if (!SendResult(socket_fd, result)) {
      break;  // Connection closed or error
    }
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
    std::cerr << "Worker: Failed to initialize NGSpice library" << std::endl;
  }
}

SimulationResult SpiceSimulator::RunSimulation(const std::string& netlist) {
  SimulationResult result;

  // Reset error flag for this simulation
  g_simulationHasError = false;

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

  if (circ_result != 0 || g_simulationHasError) {
    result.failureType = SimulationFailureType::InvalidCircuit;
    result.errorMessage = g_simulationHasError ? "Circuit has singular matrix" : "Failed to parse circuit";
    ngSpice_Command(const_cast<char*>("destroy all"));
    return result;
  }

  // Run the simulation - the .ac command is already in the netlist
  int run_result = ngSpice_Command(const_cast<char*>("run"));
  if (run_result != 0 || g_simulationHasError) {
    result.failureType = SimulationFailureType::SimulationError;
    result.errorMessage = g_simulationHasError ? "Simulation convergence failed" : "Simulation run failed";
    ngSpice_Command(const_cast<char*>("destroy all"));
    return result;
  }

  // Wait for simulation to complete
  // NGSpice runs asynchronously, so we need to wait
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Check for errors one more time
  if (g_simulationHasError) {
    result.failureType = SimulationFailureType::SimulationError;
    result.errorMessage = "Simulation had convergence errors";
    ngSpice_Command(const_cast<char*>("destroy all"));
    return result;
  }

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

  // CRITICAL BUG FIX: ngGet_Vec_Info() returns pointers to internal buffers
  // that get invalidated on the NEXT call to ngGet_Vec_Info()!
  // We must copy the data immediately after each call.

  // Get frequency vector and copy data immediately
  pvector_info freqVec = ngGet_Vec_Info(const_cast<char*>(freqVecName.c_str()));
  if (!freqVec || freqVec->v_length == 0) {
    return false;
  }

  int numPoints = freqVec->v_length;
  response.frequencies.reserve(numPoints);

  // Copy frequency data NOW before calling ngGet_Vec_Info again
  for (int i = 0; i < numPoints; i++) {
    double freq;
    if (freqVec->v_realdata) {
      freq = freqVec->v_realdata[i];
    } else if (freqVec->v_compdata) {
      freq = freqVec->v_compdata[i].cx_real;
    } else {
      return false;
    }
    response.frequencies.push_back(freq);
  }

  // Now get output vector (this invalidates freqVec pointer but we already copied the data!)
  pvector_info outVec = ngGet_Vec_Info(const_cast<char*>(outVecName.c_str()));
  if (!outVec || outVec->v_length == 0 || !outVec->v_compdata) {
    return false;
  }

  if (outVec->v_length != numPoints) {
    return false;
  }

  response.magnitudeDb.reserve(numPoints);
  response.phaseRad.reserve(numPoints);

  // Copy output data NOW
  for (int i = 0; i < numPoints; i++) {
    double real = outVec->v_compdata[i].cx_real;
    double imag = outVec->v_compdata[i].cx_imag;

    // Convert to magnitude (dB) and phase (radians)
    double magnitude = std::sqrt(real * real + imag * imag);
    double magnitudeDb = 20.0 * std::log10(magnitude + 1e-20);
    double phaseRad = std::atan2(imag, real);

    response.magnitudeDb.push_back(magnitudeDb);
    response.phaseRad.push_back(phaseRad);
  }

  return !response.frequencies.empty();
}

// Set evolution configuration
void SpiceSimulator::SetEvolutionConfig(const EvolutionConfig& config) {
  evolutionConfig_ = config;
  evolutionConfigSet_ = true;
}

// Update shared population (called by orchestrator after sorting)
void SpiceSimulator::UpdatePopulation(const std::vector<CircuitGenome>& pop) {
  std::lock_guard<std::mutex> lock(populationMutex_);
  sharedPopulation_ = pop;
}

// Tournament selection (used by worker threads)
CircuitGenome SpiceSimulator::TournamentSelect(std::mt19937& rng) {
  std::lock_guard<std::mutex> lock(populationMutex_);

  std::uniform_int_distribution<> dist(0, sharedPopulation_.size() - 1);

  int bestIdx = dist(rng);
  double bestFitness = sharedPopulation_[bestIdx].fitness;

  for (int i = 1; i < evolutionConfig_.tournamentSize; ++i) {
    int idx = dist(rng);
    if (sharedPopulation_[idx].fitness > bestFitness) {
      bestIdx = idx;
      bestFitness = sharedPopulation_[idx].fitness;
    }
  }

  return sharedPopulation_[bestIdx];
}

// Crossover (used by worker threads)
CircuitGenome SpiceSimulator::Crossover(const CircuitGenome& parent1,
                                        const CircuitGenome& parent2,
                                        std::mt19937& rng) {
  CircuitGenome offspring;
  std::uniform_real_distribution<> prob(0.0, 1.0);

  for (size_t i = 0; i < parent1.components.size(); ++i) {
    offspring.components[i] = prob(rng) < 0.5 ?
                              parent1.components[i] : parent2.components[i];
  }

  return offspring;
}

// Generate offspring (workers do selection/crossover/mutation/simulation)
std::vector<std::future<EvaluationResult>> SpiceSimulator::GenerateOffspring(int count) {
  std::vector<std::future<EvaluationResult>> futures;
  futures.reserve(count);

  for (int i = 0; i < count; ++i) {
    // Create a promise/future pair
    std::promise<EvaluationResult> promise;
    auto future = promise.get_future();
    futures.push_back(std::move(future));

    // Round-robin to worker queues
    int workerIdx = nextWorkerIdx_.fetch_add(1) % perWorkerQueues_.size();

    // Create job with empty genome and generateOffspring flag set
    CircuitGenome emptyGenome;
    SimulationJob job(std::move(emptyGenome), true);  // true = generate offspring
    job.result = std::move(promise);

    // Add to worker's queue
    {
      std::lock_guard<std::mutex> lock(*perWorkerMutexes_[workerIdx]);
      perWorkerQueues_[workerIdx]->push(std::move(job));
    }
    perWorkerCVs_[workerIdx]->notify_one();
  }

  return futures;
}
