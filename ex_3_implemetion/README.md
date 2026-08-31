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
ex_3_skeleton-main/
│
├── common/                  <-- Provided course interface headers (unmodified)
│   └── include/Common/      <-- IMappingAlgorithm.h, IMissionControl.h, etc.
│
├── UserCommon/              <-- Common implementation files (namespace user_common_207610130_215664087)
│   ├── include/UserCommon/  <-- Map3DImpl.h, MockGPS.h, MockLidar.h, MockMovement.h, DroneControlImpl.h, etc.
│   └── src/                 <-- Component source implementations
│
├── Algorithm/               <-- Mapping Algorithm Shared Library Target
│   ├── CMakeLists.txt       <-- Builds Algorithm_207610130_215664087.so
│   ├── include/Algorithm/   <-- MappingAlgorithmImpl_207610130_215664087.h (namespace algorithm_207610130_215664087)
│   └── src/                 <-- MappingAlgorithmImpl_207610130_215664087.cpp + REGISTER_MAPPING_ALGORITHM(...)
│
├── MissionControl/          <-- Mission Control Shared Library Target
│   ├── CMakeLists.txt       <-- Builds MissionControl_207610130_215664087.so
│   ├── include/MissionControl/ <-- MissionControlImpl_207610130_215664087.h (namespace mission_control_207610130_215664087)
│   └── src/                 <-- MissionControlImpl_207610130_215664087.cpp + REGISTER_MISSION_CONTROL(...)
│
├── Simulator/               <-- Executable Simulation Project Target
│   ├── CMakeLists.txt       <-- Builds simulator_207610130_215664087 executable
│   ├── include/Simulator/   <-- Registrar.h, DlLoader.h, ArgumentParser.h, SimulationEngine.h, ResultExporter.h
│   └── src/                 <-- Simulator entry point and task dispatcher
│
├── CMakeLists.txt           <-- Root CMake file building all 3 projects
├── README.md
└── students.txt
```

---

## Dynamic Registration & Loading Mechanism

- Shared libraries (`.so`) invoke static auto-registration macros (`REGISTER_MAPPING_ALGORITHM` / `REGISTER_MISSION_CONTROL`) at global scope.
- When `dlopen()` loads a `.so` library, its global static constructors execute immediately and register a factory lambda with the `Registrar` singleton inside the Simulator.
- The `DlLoader` retrieves the factory from the `Registrar` and creates objects on demand.
- Before `dlclose()` is invoked, all instantiated objects associated with the `.so` library are destroyed.

---

## Multithreading Model

- Controlled by the `num_threads` command-line argument:
  - `num_threads` omitted or `num_threads=1`: Execution runs synchronously on the main thread.
  - `num_threads >= 2`: Spawns `<num_threads>` worker threads in addition to the main thread. The main thread delegates tasks to a concurrent work queue and performs a blocking wait (`join()`) for worker completion.
- The simulator never spawns idle worker threads if the total number of tasks is smaller than `num_threads`.

---

## Building and Running

### Build Instructions
```bash
cmake --preset default
cmake --build --preset default
```

### Running Comparative Mode
```bash
./build/Simulator/simulator_207610130_215664087 \
  -comparative \
  simulation=inputs/simulation_compositions.yaml \
  mission_control_folder=./build/MissionControl \
  algorithm=./build/Algorithm/Algorithm_207610130_215664087.so \
  num_threads=2 \
  -verbose
```

### Running Competitive Mode
```bash
./build/Simulator/simulator_207610130_215664087 \
  -competition \
  simulation=inputs/simulation_compositions.yaml \
  mission_control=./build/MissionControl/MissionControl_207610130_215664087.so \
  algorithms_folder=./build/Algorithm \
  num_threads=2 \
  -verbose
```
