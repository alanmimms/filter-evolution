# Evolutionary Filter Designer for HF EER Transmitter

A C++20 application that uses evolutionary algorithms to design optimal Low-Pass Filters (LPFs) for high-performance HF transmitters using Envelope Elimination and Restoration (EER) architecture.

## Overview

This tool evolves **both topology and component values** to create filters that achieve superior harmonic suppression compared to classical filter designs. It's specifically optimized for Class-E Power Amplifier outputs with known harmonic characteristics.

### Key Features

- **Topology Evolution**: Discovers optimal circuit structures, not just component values
- **Application-Specific Optimization**: Optimizes against actual PA harmonic spectrum
- **Parallel Evaluation**: Uses C++20 async features for fast fitness evaluation
- **Flexible Genome**: Represents arbitrary LC networks with up to 20 components
- **NGSpice Integration**: Industrial-grade SPICE simulation for accuracy

## Architecture

### Transmitter Context

The target system is a 50-watt, software-defined HF transmitter with:
- Direct-conversion architecture
- Polar modulation (phase + envelope separation)
- Class-E PA driven by phase-modulated square wave
- Fast-tracking buck-boost envelope converter
- Coverage: 160m through 10m bands

### Design Challenge

Class-E switching amplifiers are highly efficient but generate strong harmonics:
- 2nd harmonic: -19.5 to -22.5 dBc (input to filter)
- FCC requirement: -43 dBc spurious emissions
- Design target: -50 dBc or better output purity
- Required filter performance: ≥45 dB stopband attenuation

## Prerequisites

### System Requirements

- C++20 compatible compiler (GCC 10+, Clang 10+, MSVC 2019+)
- CMake 3.20 or later
- NGSpice circuit simulator
- pthread support (for parallel evaluation)

### Installing Dependencies

#### Ubuntu/Debian
```bash
sudo apt update
sudo apt install build-essential cmake ngspice
```

#### macOS
```bash
brew install cmake ngspice
```

#### Arch Linux
```bash
sudo pacman -S cmake ngspice
```

## Building

### Project Structure

```
evolutionary-filter/
├── src/
│   ├── main.cpp              # Entry point and data loading
│   ├── genome.h/cpp          # Circuit representation
│   ├── evolution.h/cpp       # Evolutionary algorithm
│   ├── fitness.h/cpp         # Fitness evaluation
│   └── spice_interface.h/cpp # NGSpice integration
├── data/
│   └── harmonics.csv         # PA harmonic spectrum data
├── CMakeLists.txt
└── README.md
```

### Compilation

```bash
# Create build directory
mkdir build && cd build

# Configure (Release build for speed)
cmake -DCMAKE_BUILD_TYPE=Release ..

# Compile
make -j$(nproc)

# Or for debug build with symbols
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
```

## Usage

### Input Data Format

Create `harmonics.csv` with your PA's harmonic spectrum:

```csv
LPF_Band,Fundamental_Freq_MHz,Harmonic_Order,Harmonic_Freq_MHz,Level_dBc
LPF_1_160m,2.0,1,2.0,0.0
LPF_1_160m,2.0,2,4.0,-19.5
LPF_1_160m,2.0,3,6.0,-24.5
...
```

### Running the Optimizer

```bash
# Use default harmonics.csv in current directory
./filter_evolver

# Or specify custom data file
./filter_evolver path/to/custom_harmonics.csv
```

### Output

The program outputs:
1. **Console Progress**: Generation-by-generation fitness evolution
2. **Best Filter Stats**: Component count, fitness score
3. **SPICE Netlist**: Complete circuit in `best_filter.cir`

Example output:
```
Evolutionary Filter Designer for HF EER Transmitter
====================================================

Loaded 40 harmonic specifications
Covering 8 bands

Initializing population...
Starting evolution for 500 generations
============================================================
Gen    0 | Best:      247.35 | Avg:      892.47
Gen   10 | Best:      183.21 | Avg:      654.32
Gen   20 | Best:      124.89 | Avg:      421.65
...
Gen  490 | Best:       12.43 | Avg:       45.67
Gen  500 | Best:        8.92 | Avg:       38.21
============================================================
Evolution complete!
Best fitness: 8.92
Active components: 12
Active inductors: 5

BEST FILTER DESIGN
============================================================

SPICE Netlist:
------------------------------------------------------------
* Evolved Filter Circuit
Vin in 0 AC 1
Rin in input 50
L1 input n3 2.450000e-07
C1 n3 0 4.720000e-10
L2 n3 n4 1.830000e-07
...
Rload output 0 50
.ac dec 50 1e6 150e6
.end

Netlist saved to: best_filter.cir
```

## Algorithm Details

### Genome Representation

Each circuit genome contains:
- Up to 20 components (L or C)
- Component values (10-5000 nH for L, 10-5000 pF for C)
- Node connectivity (input, output, ground, 10 internal nodes)
- Active/inactive flags

### Fitness Function

Penalizes:
1. **Passband loss** at fundamental frequencies (heavy weight)
2. **Insufficient stopband attenuation** at harmonics (very heavy, weighted by harmonic order)
3. **Inductor count** (small penalty - L components are expensive)

```
fitness = Σ(passband_errors × 100) 
        + Σ(stopband_shortfalls × 50 / harmonic_order)
        + inductor_count × 10
```

### Evolutionary Operators

**Mutation Types** (15% rate):
- Value mutation (multiplicative or additive)
- Component type flip (L ↔ C)
- Connection rewiring
- Active/inactive toggle

**Crossover** (70% rate):
- Uniform crossover (each gene 50% from either parent)

**Selection**:
- Tournament selection (size 5)
- Elitism (top 5 preserved)

### Population Initialization

- 3-5 seeded topologies (ladder networks of varying order)
- Remainder randomly generated
- Typical population: 100 individuals

### Parallelization

Uses C++20 `std::async` to evaluate multiple genomes simultaneously. On an 8-core CPU, achieves ~8x speedup over serial evaluation.

## Configuration

Edit `EvolutionConfig` in `main.cpp`:

```cpp
EvolutionConfig config;
config.populationSize = 100;   // Individuals per generation
config.generations = 500;      // Number of generations
config.eliteCount = 5;         // Top N preserved each gen
config.mutationRate = 0.15;    // 15% per component per gen
config.crossoverRate = 0.7;    // 70% offspring from crossover
config.tournamentSize = 5;     // Tournament selection pool
```

## Validation & Testing

### Verify Evolved Filter

```bash
# Run the saved netlist in NGSpice
ngspice best_filter.cir

# In NGSpice prompt:
run
plot vdb(output)
print vdb(output) > response.txt
quit
```

### Check Harmonic Suppression

Parse `response.txt` and verify:
- Passband insertion loss < 0.5 dB
- 2nd harmonic suppression > 45 dB
- All harmonics meet -50 dBc target

## Customization

### Different Frequency Bands

Modify `harmonics.csv` with your specific bands and harmonic measurements.

### Tighter Specifications

Adjust in `fitness.cpp`:

```cpp
static constexpr double TargetOutputDbc = -60.0;  // More stringent
static constexpr double MaxPassbandLossDb = 0.2;  // Lower insertion loss
```

### Component Value Ranges

Modify in `genome.h`:

```cpp
static constexpr double MinInductorNh = 5.0;
static constexpr double MaxInductorNh = 10000.0;
static constexpr double MinCapacitorPf = 5.0;
static constexpr double MaxCapacitorPf = 10000.0;
```

### More Complex Topologies

Increase in `genome.h`:

```cpp
static constexpr int MaxComponents = 30;      // Allow bigger circuits
static constexpr int NumInternalNodes = 15;   // More routing flexibility
```

## Performance Tips

1. **Initial Generations Are Slow**: First ~20 generations explore diverse topologies, many invalid
2. **CPU Cores Matter**: Linear scaling up to ~8-16 cores
3. **Long Runs Recommended**: 500-1000 generations for best results
4. **Multiple Runs**: Try 3-5 independent runs, pick best result

Typical runtime on modern 8-core CPU:
- 100 generations: ~10 minutes
- 500 generations: ~45 minutes
- 1000 generations: ~90 minutes

## Troubleshooting

### NGSpice Not Found

```
Error: ngspice command not found
```

Install ngspice or add to PATH.

### Compilation Errors

Ensure C++20 support:
```bash
g++ --version  # Should be 10.0 or later
```

### Simulation Failures

Check temp files:
```bash
cat temp_circuit.cir
cat simulation.log
```

### Poor Convergence

Try:
- Larger population (200-300)
- More generations (1000+)
- Higher mutation rate (0.20-0.25)
- Different random seed (rerun)

## Theory of Operation

This tool implements **Evolutionary Circuit Synthesis** as described in Ted Dunning's QEX article "Understanding and Using Circuit Optimization". Key insights:

1. **Classical designs optimize generic specs** (ripple, rolloff rate)
2. **This tool optimizes YOUR specific specs** (suppress 4.0 MHz by exactly 30.5 dB while passing 2.0 MHz perfectly)
3. **Topology evolution discovers structures** that classical tables can't provide
4. **Application-specific fitness** creates filters impossible to design by hand

The result: Filters that dramatically outperform textbook designs for your exact use case.

## Future Enhancements

Potential improvements:
- [ ] Schematic visualization (export to KiCad/gEDA)
- [ ] Multi-objective optimization (Pareto front)
- [ ] Constraint-based component values (E12/E24 series)
- [ ] PCB parasitic modeling
- [ ] Temperature stability analysis
- [ ] Automated BOM generation with vendor links

## References

- Ted Dunning, "Understanding and Using Circuit Optimization", QEX Sep/Oct 2025
- NGSpice User Manual: http://ngspice.sourceforge.net/docs.html
- Class-E PA Design: Nathan Sokal, "Class E - A New Class of High-Efficiency Tuned Single-Ended Switching Power Amplifiers"

## License

This code is provided as-is for educational and research purposes. Modify freely for your projects.

## Author

Created for HF EER transmitter development. Based on evolutionary circuit synthesis principles.

## Contributing

Improvements welcome! Areas of interest:
- Faster SPICE integration (libngspice direct API)
- GPU acceleration for parallel evaluation
- Better seeding strategies
- Component tolerance analysis
- Real-world validation data
