#include "spice_interface.h"
#include "genome.h"
#include <ngspice/sharedspice.h>
#include <iostream>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>

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
  : nextWorkerIdx_(0), shutdown_(false) {
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

  // Start orchestrator thread
  orchestratorThread_ = std::thread(&SpiceSimulator::OrchestratorThread, this);
}

void SpiceSimulator::SpawnWorkers(int numWorkers) {
  workers_.resize(numWorkers);

  for (int i = 0; i < numWorkers; i++) {
    // Create socketpair for IPC
    int sockets[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) < 0) {
      std::cerr << "Failed to create socketpair for worker " << i << std::endl;
      continue;
    }

    pid_t pid = fork();
    if (pid < 0) {
      std::cerr << "Failed to fork worker " << i << std::endl;
      close(sockets[0]);
      close(sockets[1]);
      continue;
    }

    if (pid == 0) {
      // Child process
      close(sockets[0]);  // Close parent end

      // Close all other worker sockets inherited from parent
      for (int j = 0; j < i; j++) {
        if (workers_[j].socket_fd >= 0) {
          close(workers_[j].socket_fd);
        }
      }

      WorkerProcessMain(sockets[1]);
      _exit(0);  // Worker should never return
    }

    // Parent process
    close(sockets[1]);  // Close child end
    workers_[i].pid = pid;
    workers_[i].socket_fd = sockets[0];
    workers_[i].busy = false;
  }

  std::cout << "Spawned " << numWorkers << " worker processes for parallel simulation\n";
}

void SpiceSimulator::ShutdownWorkers() {
  shutdown_ = true;
  queueCV_.notify_all();

  // Wait for orchestrator to finish
  if (orchestratorThread_.joinable()) {
    orchestratorThread_.join();
  }

  // Shutdown workers
  for (auto& worker : workers_) {
    if (worker.pid > 0) {
      // Send empty string to signal shutdown
      SendString(worker.socket_fd, "");
      close(worker.socket_fd);

      // Wait for worker to exit
      int status;
      waitpid(worker.pid, &status, 0);
    }
  }

  workers_.clear();
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

void SpiceSimulator::OrchestratorThread() {
  while (!shutdown_) {
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

    // Get available worker
    int workerIdx = GetAvailableWorker();

    // Send to worker and get result
    SimulationResult result = SendToWorker(workerIdx, job.netlist);

    // Send result back via promise
    job.result.set_value(std::move(result));
  }
}

int SpiceSimulator::GetAvailableWorker() {
  // Simple round-robin assignment
  std::lock_guard<std::mutex> lock(workerMutex_);
  int worker = nextWorkerIdx_;
  nextWorkerIdx_ = (nextWorkerIdx_ + 1) % workers_.size();
  return worker;
}

SimulationResult SpiceSimulator::SendToWorker(int workerIdx, const std::string& netlist) {
  SimulationResult result;

  if (workerIdx < 0 || workerIdx >= (int)workers_.size()) {
    result.failureType = SimulationFailureType::InternalError;
    result.errorMessage = "Invalid worker index";
    return result;
  }

  WorkerProcess& worker = workers_[workerIdx];

  // Send netlist
  if (!SendString(worker.socket_fd, netlist)) {
    result.failureType = SimulationFailureType::InternalError;
    result.errorMessage = "Failed to send netlist to worker";
    return result;
  }

  // Receive result
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

// Worker process entry point
void SpiceSimulator::WorkerProcessMain(int socket_fd) {
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
