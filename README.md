# JS-ME-ControlSimulation

This repository contains the simulation tools and results developed for my Master's thesis. The project is organized into two main components:

## Parameter Estimation

This section includes scripts and functions used to estimate physical parameters from experimental data. These estimates feed into the control models to ensure accurate system representation.

Key features:
- Data preprocessing and filtering
- Model fitting for dynamic system parameters
- Visualizations of estimation accuracy and residuals

## Control Allocation

This section focuses on allocating control inputs based on the estimated parameters to achieve desired system behavior under constraints.

Key features:
- Control allocation algorithms (e.g. pseudo-inverse, optimization-based)
- Constraint handling (e.g. actuator limits)
- Simulation of closed-loop control behavior

## Project Structure

- `ParameterEstimation/` – Scripts and functions for parameter identification  
- `ControlAllocation/` – Control allocation and simulation tools  
- `data/` – Raw and processed experimental data  
- `results/` – Generated figures and logs  
- `README.md` – Project overview  

## Requirements

- MATLAB R2023a or later
- Control System Toolbox
- Optimization Toolbox (for advanced control allocation)

## Running Simulations

Open MATLAB, navigate to the project folder, and open the corresponding `.mlx` or `.m` files under `ParameterEstimation` or `ControlAllocation`.

## License

This project is for academic use only. Contact the author for other uses.

---

For questions or collaboration, reach out via GitHub or email.
