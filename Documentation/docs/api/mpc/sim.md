---
title: "sim"
slug: "/api/mpc/sim/"
---

Simulate a `jMPC` MPC Controller in a given `jSIM` Simulation Environment.

## Syntax

> `jSIMobj = sim(jMPCobj,jSIMobj)`

> `jSIMobj = sim(jMPCobj,jSIMobj,mode)`

## Description

`jSIMobj = sim(jMPCobj,jSIMobj,mode)` applies the MPC Controller in `jMPCobj` to the [Simulation Environment](../simulation-options/index.md) in `jSIMobj`, using the given `mode`. There are currently four modes available for simulation:

|  |  |
| --- | --- |
| Mode	Description |  |
| `'MEX'` | Default High speed C implementation of the jMPC engine and QP solver. Useful for multiple MPC simulation studies. |
| `'Matlab'` | Simulation mode uses an m-file implementation of the jMPC engine. Best for debugging and algorithm development. |
| `'Simulink'` | Uses the Discrete MPC C S Function within Simulink as the jMPC engine. |
| `'MPCToolbox'` | Converts the `jMPC` object to a MATLAB MPC Toolbox object and simulates it using the [MPC Toolbox®](https://au.mathworks.com/help/mpc/index.html). Useful for comparison studies of `jMPC` results against a commercial product. |

From all simulation modes the returned `jSIM` object will be tagged as a results object, and will contain the simulation results. 

## Functional Overview

The function performs 6 steps at every sample of the simulation:

|  |  |  |
| --- | --- | --- |
| Step | Operation | Description |
| 1 | State Estimator Update | The current model states are updated with the state estimation gain, **Kest**. This infers a current step rather than a predictor step state estimator. |
| 2 | Solve MPC Control Law | As described in [mpcsolve](./mpcsolve.md), the MPC Control law is computed here and **u** solved for. |
| 3 | Saturate Inputs | The calculated rate of change of input, and absolute input are saturated to the limits of the constraints (if they are not already within these limits). |
| 4 | Simulate Model | The model is simulated over one time step. |
| 5 | Apply Input to Plant | The input is applied to the plant for one time step. |
| 6 | Save Sample Results & Repeat | The results of the above are saved and the simulation repeats until specified. |
