# JS-ME-ControlSimulation

This repository contains the simulation tools, processing scripts, and analysis developed for my Master's thesis on online parameter estimation and control allocation for UAV systems. The project uses a Flamingo octocopter platform running PX4 v1.8 as the primary test vehicle.

## Project Sections

### 1. Parameter Sensitivity Analysis
Monte Carlo and Sobol-based sensitivity analysis to identify which UAV physical parameters most significantly affect estimator and control performance. Includes polynomial chaos expansion (PCE) and multivariate linear regression (MLR) analysis.

### 2. Estimator Simulation (RLS & UKF)
Simulation-based comparison of two online parameter estimators — Recursive Least Squares (RLS) and Unscented Kalman Filter (UKF) — run against synthetic flight data. Includes batch and parallelised runners for parameter sweeps.

### 3. Real World Data Processing
Pipeline for converting PX4 `.ulg` flight logs to MATLAB format, replaying the RLS and UKF estimators on real flight data, and comparing estimator performance against motion capture ground truth.

### 4. Mixer Calculation & Testing
Generation of updated control effectiveness mixers from estimated UAV parameters, validated through batch closed-loop simulation to assess the impact of re-identified parameters on control allocation performance.

## Project Structure
```
├── Parameter Sensitivity/     – Sensitivity analysis scripts (Monte Carlo, Sobol, MLR, PCE)
├── RLS/                       – Recursive Least Squares estimator
├── UKF/                       – Unscented Kalman Filter estimator
├── DataProcessing/            – Pre/post processing for simulation data
├── EXP_Processing/            – Real world flight log processing and replay
├── MixerTesting/              – Mixer generation, validation, and batch simulation
├── Flamingo Data/             – Raw flight logs and motion capture data
├── ConvertedUlogs/            – Processed .ulg logs converted to .mat
├── ParameterSet/              – Sampled parameter sets for batch runs
├── archive/                   – Legacy Simulink PX4 flight simulation (DTRG submodule)
└── parameters.m               – Global UAV parameter definitions
```

## Documentation

Detailed documentation of each section, script-level descriptions, and workflow diagrams are provided in the accompanying thesis PDF. A structured breakdown of the codebase is also available in the project Obsidian vault notes.

## Requirements

- MATLAB R2024b or later
- Simulink
- Control System Toolbox
- Optimization Toolbox
- UQLab (for PCE / Sobol sensitivity analysis)

## Running the Code

Open MATLAB and navigate to the project root. Each section has a dedicated base script as its entry point:

- **Sensitivity Analysis:** `Parameter Sensitivity/monteCarloSim.m`
- **RLS Simulation:** `RLS/RLSBase.m`
- **UKF Simulation:** `UKF/UKFBase.m`
- **Real World Processing:** `EXP_Processing/ulog_to_mat.m` → `EXP_Processing/RLSBaseEXP.m` / `UKFBaseEXP.m`
- **Mixer Testing:** `EffectivenessMixer.m` → `MixerTesting/MixerValidationBase.m`

## License

This project is for academic use only. Contact the author for other uses.

---

For questions or collaboration, reach out via GitHub or email.
