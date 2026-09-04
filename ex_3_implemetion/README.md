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

## Project Structure & Architectural Separation

In strict alignment with the course's *Structuring the project* guidelines:

```text
ex3_207610130_215664087/
│
├── common/                  <-- Provided course interface headers (unmodified)
│   ├── CMakeLists.txt
│   └── include/Common/      <-- IMappingAlgorithm.h, IMissionControl.h, IMap3D.h, Types.h, etc.
│
├── Algorithm/               <-- Mapping Algorithm Shared Library Target (namespace algorithm_207610130_215664087)
│   ├── CMakeLists.txt       <-- Independent build for Algorithm_207610130_215664087.so
│   ├── include/Algorithm/   <-- MappingAlgorithmImpl_207610130_215664087.h
│   └── src/                 <-- MappingAlgorithmImpl_207610130_215664087.cpp + REGISTER_MAPPING_ALGORITHM(...)
│
├── MissionControl/          <-- Mission Control Shared Library Target (namespace mission_control_207610130_215664087)
│   ├── CMakeLists.txt       <-- Independent build for MissionControl_207610130_215664087.so
│   ├── common_mission_control/ <-- IDroneControl.h interface
│   ├── include/MissionControl/ <-- MissionControlImpl_..., DroneControlImpl.h, ScanResultToVoxels.h
│   └── src/                 <-- MissionControlImpl_...cpp, DroneControlImpl.cpp, ScanResultToVoxels.cpp
│
├── Simulator/               <-- Simulation Executable Target (namespace simulator_207610130_215664087)
│   ├── CMakeLists.txt       <-- Independent build for simulator_207610130_215664087 executable
│   ├── common_simulator/   <-- ISimulation.h, ISimulationRun.h, SimulationTypes.h
│   ├── include/Simulator/   <-- SimulationEngine.h, Registrar.h, DlLoader.h, ResultExporter.h,
│   │                            ConfigParser.h, Map3DImpl.h, MapsComparison.h, MockGPS.h,
│   │                            MockLidar.h, MockMovement.h, NpyMapIO.h, ArgumentParser.h
│   └── src/                 <-- main.cpp, SimulationEngine.cpp, Registrar.cpp, DlLoader.cpp,
│                                ResultExporter.cpp, ConfigParser.cpp, Map3DImpl.cpp,
│                                MapsComparison.cpp, MockGPS.cpp, MockLidar.cpp,
│                                MockMovement.cpp, NpyMapIO.cpp, ArgumentParser.cpp
│
├── UserCommon/              <-- Shared types and utility headers (namespace user_common_207610130_215664087)
│   └── include/UserCommon/  <-- CommonDefines.h, AngleUtils.h, GeometryUtils.h
│
├── CMakeLists.txt           <-- Root CMake build orchestrator
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
  - `num_threads` omitted or `num_threads=1`: Execution runs synchronously on the main thread (1 thread).
  - `num_threads >= 2`: Spawns `<num_threads>` worker threads in addition to the main thread ($N+1 \ge 3$ threads).
  - If `jobs.size() < 2`, runs synchronously on the main thread. Total running threads is **never 2**.
- **Fine-Grained SimulationJob Queue**: Work is divided at the individual simulation run level (`simulation × mission × drone × lidar` Cartesian specs) across all tested plugins.
- Worker threads pull from an atomic job index counter, ensuring full thread utilization without locking during run execution.

---

## Physical Simulation & Swept Collision Detection

- **Full Building Hidden Map**: Physical boundaries of the ground-truth hidden map are calculated from the full dimensions of the input NumPy array ($\text{dimensions} \times \text{resolution} - \text{offset}$), enabling the hidden world to cover the entire building while `output_map` is bounded by `mission_bounds`.
- **Mandatory Swept Volume Collision Checking**: In `MockMovement`, trajectory advancement and vertical elevation step through the path in sub-voxel increments, testing both the center voxel and the drone's spherical radius against the hidden map to detect any physical wall collisions.

---

## Error Handling & Reports

- **Error Continuation**: If a single simulation run encounters an error, it is recorded with `score = -1.0` and `status = "error"`, and the simulator proceeds to execute all subsequent runs.
- **Immediate Error Logging**: Errors are logged immediately to `error_log.txt` under a thread-safe mutex and flushed.
- **Verbose Mode (`-verbose`)**: Creates dedicated `_verbose.log` files with step-by-step telemetry on disk for each mission run.
- **Assignment-2 `score_report` Schema**: Per-SO reports emit the exact hierarchical `score_report:` YAML schema with summary statistics and simulation/mission/run trees.

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

---

### Running Standalone Maps Comparison

```bash
./build/default/Simulator/maps_comparison <origin_map.npy> <target_map.npy> [comparison_config=<config.yaml>]
```
- Prints the comparison percentage score (`0` to `100`) to `stdout` upon success.
- Prints `-1` to `stdout` and error diagnostics to `stderr` upon failure.

---

## Output Folder Structure

Each simulation run generates a unique timestamped output directory inside the tested plugin directory:
- **Comparative Mode**: `<mission_control_folder>/comparative_results_<timestamp>/`
- **Competitive Mode**: `<algorithms_folder>/competition_<timestamp>/`

Containing:
- `error_log.txt`: Always created; records any simulation or load errors immediately with mutex synchronization.
- `comparative_simulation_report.yaml` / `competitive_simulation_report.yaml`: Aggregate summary report.
- `<plugin_name>_simulation_report.yaml`: Per-plugin detailed report adhering to the Assignment-2 YAML schema.
- `output_map_<plugin>_run_<index>.npy`: 3D voxel grid output maps generated by the simulation.
- `output_map_<plugin>_run_<index>_verbose.log`: Detailed step-by-step telemetry logs (when `-verbose` is specified).

