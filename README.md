# Autonomous Drone Interception via EDMDc-Based Model Predictive Control

This repository contains the official Python implementation for the paper **"Autonomous Drone Interception via EDMDc-Based Model Predictive Control"**. 

It provides a complete framework for simulating nonlinear quadcopter dynamics, identifying a lifted linear predictor using Extended Dynamic Mode Decomposition with control (EDMDc), and embedding that model into a real-time Model Predictive Control (MPC) quadratic program (QP) to track complex 3D trajectories and intercept moving targets.

## Features
* **Nonlinear Quadcopter Simulation:** High-fidelity 12-state rigid-body dynamics with a cascaded inner-loop attitude PID controller.
* **Data-Driven Modeling (EDMDc):** Lifts the physical 10-state space into a 27-dimensional observable space (capturing trigonometric, cross-coupling, and energy terms) via Tikhonov-regularized least squares.
* **Real-Time MPC:** Formulates the Koopman-based predictive controller as a sparse Quadratic Program solved via OSQP at sub-millisecond speeds.
* **Comparative Baselines:** Includes rigorous comparisons against a hover-linearized MPC baseline and a reactive PID controller under identical actuator constraints and tracking horizons.

## Requirements & Installation

This project requires **Python 3.8+**. 

Install the required dependencies using pip:

    pip install numpy scipy matplotlib scikit-learn osqp

*Note: osqp is used as the core QP solver for the MPC frameworks, scikit-learn handles the StandardScaler for state/input normalization prior to lifting, and scipy is heavily used for solve_ivp integration and sparse matrix operations.*

## Repository Structure

* `quadcopter.py` & `Simulation.py`: Defines the physical plant, rigid-body dynamics, and trajectory generators (Helix, Figure-8, Lissajous, Waypoints, PRBS).
* `Cascaded_Controllers.py` & `Closed_loop.py`: Inner-loop attitude PID controllers and closed-loop integration steps.
* `parallel_sim.py` & `mix_traj.py`: Scripts for generating randomized closed-loop training data via multiprocessing and packaging it into combined datasets.
* `EDMDc_training.py`: Standardizes data, computes the 27-dimensional observable lifting, and identifies the EDMDc A and B matrices with rolling-horizon regularization.
* `edmdc_mpc.py`: The core OSQP-based Model Predictive Control class formulation.
* `final_comparison.py`: Compares PID, Linear MPC, and EDMDc MPC on held-out tracking trajectories.
* `Intercept_comparison.py`: Simulates active interception of evasive 3D targets using the trained predictive controllers.

## Usage: How to Run

To fully reproduce the results from the paper, run the following sequence of commands in your terminal:

1. Generate the training data across all trajectory families
    python parallel_sim.py

2. Combine the datasets into a single master training file
    python mix_traj.py

3. Train the EDMDc model and save the matrices
    python EDMDc_training.py

4. Run the trajectory tracking comparisons (100Hz and 10Hz)
    python final_comparison.py --dt 0.01
    python final_comparison.py --dt 0.1

5. Run the drone interception scenarios
    python Intercept_comparison.py
