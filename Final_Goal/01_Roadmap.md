# Development Roadmap

This document outlines the phased execution plan to achieve the [Product Vision](00_Product_Vision.md).

## Phase 1: Foundation (Current) ✅
*   **Goal**: robust simulation environment and basic control.
*   [x] 3D Physics Environment (MuJoCo).
*   [x] Basic Linear MPC.
*   [x] Directory Architecture Refactor (Production Grade).
*   [ ] **Next**: Migration to Hydra Configuration System.

## Phase 2: The "Universal" Upgrade
*   **Goal**: Decouple the software from specific hardware layouts.
*   [ ] **Configuration System**: Implement Hydra/OmegaConf.
*   [ ] **Dynamic Dynamics**: Procedurally generate B-Matrices ($\tau = r \times F$) from YAML config.
*   [ ] **Vehicle Integrator**: Update Visualizer and Sim to spawn thrusters based on file definition.

## Phase 3: Robustness & Fault Tolerance
*   **Goal**: Ensure mission success despite hardware failures.
*   [ ] **Adaptive Control**: Implement Real-time B-Matrix updates.
*   [ ] **Chaos Monkey**: Build a random fault injector for Monte Carlo testing.
*   [ ] **FDIR**: Fault Detection, Isolation, and Recovery logic layer.

## Phase 4: Autonomy (The Product)
*   **Goal**: "One-click" inspection of any object.
*   [ ] **Mesh Analysis**: Raycasting engine to determine surface normals and visibility.
*   [ ] **Coverage Planner**: Traveling Salesperson Problem (TSP) solver for viewpoint ordering.
*   [ ] **Trajectory Optimization**: Smooth graph-search (A*) between viewpoints.

## Phase 5: Commercialization
*   **Goal**: Sellable, flight-ready assets.
*   [ ] **C++ Export**: Pipeline to generate `acados` C code from Python MPC definition.
*   [ ] **HIL Bridge**: Serial/UDP interface for Hardware-in-the-Loop testing.
*   [ ] **Binary Obfuscation**: IP protection strategy.
