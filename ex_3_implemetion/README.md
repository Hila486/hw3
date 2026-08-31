# Assignment 3 - Drone Mapper (Multi-threaded Concurrent Simulator)

**Submitters**:
- **Shalev Cohen**, ID: 207610130
- **Hila Regev Shabath**, ID: 215664087

---

## Overview

Assignment 3 expands the autonomous drone mapping system into a multi-threaded, concurrent simulator capable of evaluating mapping algorithms and mission controllers compiled as independent dynamic shared libraries (`.so` files).

The system supports two execution modes:
1. **Comparative Mode (`-comparative`)**: Runs multiple `MissionControl` shared libraries against a single `Algorithm` shared library.
2. **Competitive Mode (`-competition`)**: Runs multiple `Algorithm` shared libraries against a single `MissionControl` shared library.

---

## Project Structure & Namespaces

```text
ex3_207610130_215664087/
│
├── common/                  <-- Provided course interface headers (unmodified)
│   └── include/Common/      <-- IMappingAlgorithm.h, IMissionControl.h, etc.
│
├── UserCommon/              <-- Common implementation files (namespace user_common_207610130_215664087)
│   ├── include/UserCommon/  <-- Map3DImpl.h, MockGPS.h, MockLidar.h, MockMovement.h, DroneControlImpl.h, etc.
│   └── src/                 <-- Component source implementations
│
├── Algorithm/               <-- Mapping Algorithm Shared Library Target
│   ├── CMakeLists.txt       <-- Builds Algorithm_207610130_215664087.so (independent build)
│   ├── include/Algorithm/   <-- MappingAlgorithmImpl_207610130_215664087.h (namespace algorithm_207610130_215664087)
│   └── src/                 <-- MappingAlgorithmImpl_207610130_215664087.cpp + REGISTER_MAPPING_ALGORITHM(...)
│
├── MissionControl/          <-- Mission Control Shared Library Target
│   ├── CMakeLists.txt       <-- Builds MissionControl_207610130_215664087.so (independent build)
│   ├── include/MissionControl/ <-- MissionControlImpl_207610130_215664087.h (namespace mission_control_207610130_215664087)
│   └── src/                 <-- MissionControlImpl_207610130_215664087.cpp + REGISTER_MISSION_CONTROL(...)
│
├── Simulator/               <-- Executable Simulation Project Target
│   ├── CMakeLists.txt       <-- Builds simulator_207610130_215664087 executable (independent build)
│   ├── include/Simulator/   <-- Registrar.h, DlLoader.h, ArgumentParser.h, SimulationEngine.h, ResultExporter.h
│   └── src/                 <-- Simulator entry point and task dispatcher
│
├── CMakeLists.txt           <-- Root CMake file building all 3 projects
├── README.md                <-- Project documentation
└── students.txt             <-- Submitter names and IDs
```

---

## Dynamic Registration & Loading Mechanism

- Shared libraries (`.so`) invoke static auto-registration macros (`REGISTER_MAPPING_ALGORITHM` / `REGISTER_MISSION_CONTROL`) at global scope.
- When `dlopen()` loads a `.so` library on the main thread, its global static constructors execute immediately and register a factory lambda with the `Registrar` singleton inside the Simulator.
- The `DlLoader` retrieves the factory from the `Registrar` and creates objects on demand.
- **Strict Lifetime Ordering**: All instantiated algorithm/mission control objects and captured factory `std::function` objects are explicitly reset/destroyed *before* `dlclose()` unmaps the shared library.

---

## Multithreading Model

- Controlled by the `num_threads` command-line argument:
  - `num_threads` omitted or `num_threads=1`: Execution runs synchronously on the main thread.
  - `num_threads >= 2`: Spawns `<num_threads>` worker threads in addition to the main thread. The main thread joins worker completion.
- **Fine-Grained SimulationJob Queue**: Work is divided at the individual simulation run level (`simulation × mission × drone × lidar` Cartesian specs) across all tested plugins.
- Worker threads pull from an atomic job index counter, ensuring full thread utilization even when testing a single plugin across multiple scenarios.
- The simulator never spawns idle worker threads if the total number of jobs is smaller than `num_threads`.

---

## Error Handling & Logging

- **Error Continuation**: If a single simulation run encounters an error, it is recorded with `score = -1.0` and `status = "error"`, and the simulator proceeds to execute all subsequent runs.
- **Immediate Error Logging**: Errors are logged immediately to `error_log.txt` under a thread-safe mutex and flushed.
- **Verbose Mode (`-verbose`)**: Creates dedicated `_verbose.log` files with step-by-step telemetry on disk for each mission run.

---

## Building and Running

### 1. Building Entire Project from Root
```bash
cmake --preset default
cmake --build --preset default
```

### 2. Building Each Module Independently

#### Mapping Algorithm:
```bash
cmake -S Algorithm -B build_algo
cmake --build build_algo
```

#### Mission Control:
```bash
cmake -S MissionControl -B build_mc
cmake --build build_mc
```

#### Simulator:
```bash
cmake -S Simulator -B build_sim
cmake --build build_sim
```

---

### Running the Simulator

#### Comparative Mode:
```bash
./build/default/Simulator/simulator_207610130_215664087 \
  -comparative \
  simulation=inputs/sim_compose.yaml \
  mission_control_folder=./build/default/MissionControl \
  algorithm=./build/default/Algorithm/Algorithm_207610130_215664087.so \
  num_threads=4 \
  -verbose
```

#### Competitive Mode:
```bash
./build/default/Simulator/simulator_207610130_215664087 \
  -competition \
  simulation=inputs/sim_compose.yaml \
  mission_control=./build/default/MissionControl/MissionControl_207610130_215664087.so \
  algorithms_folder=./build/default/Algorithm \
  num_threads=4 \
  -verbose
```
