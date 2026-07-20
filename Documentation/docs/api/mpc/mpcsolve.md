---
title: "mpcsolve"
slug: "/api/mpc/mpcsolve/"
---

Solve the MPC Control Law for the optimal control input.

## Syntax

> `[delu,iter,t] = mpcsolve(jMPCobj,del_xm,u,old_u,setp,mdist,k,tstart)`

## Description

`[delu,iter,t] = mpcsolve(jMPCobj,del_xm,u,old_u,setp,mdist,k,tstart)` uses current and previous model states and plant inputs, together with the Quadratic Program stored within the `jMPC` object, to solve for the next optimal control input increment, `delu`. This function requires the jMPC object, `jMPCobj`, the current state increment, `del_xm`, previous plant input, `u`, previous solution vector, `old_u`, the current setpoint, `setp`, the current measured disturbance, `mdist`, the current simulation time step, `k`, and the CPU time at which the function starts. The function returns a solution vector, `del_u`, the number of iterations required by the QP Solver, and a vector of the times taken by the various components of the function.

Note this function is not intended to be called directly by the user.

## Functional Overview

The function performs 4 steps at every sample of the simulation:

|  |  |  |
| --- | --- | --- |
| Step | Operation | Description |
| 1 | Update QP Right Hand Side | In the current toolbox implementation, both **H** & **A** remain constant, while **f** & **b** have constant and dynamic parts. The dynamic parts are a function of the current state, setpoint, and previous input, and thus must be updated at every sample. |
| 2 | Check Global Minimum | It is possible the global minimum (`H\f`) is also the constrained minimum, thus to save computation time and increase efficiency, the global minimum is checked against each constraint, and if it passes, will skip the QP solver. |
| 3 | Shift previous solution vector | If the global minimum is not also the constrained minimum, then the previous solution vector, `old_u`, is shifted one step toward the future and used to warm start the QP Solver. |
| 4 | Solve QP for new solution | The QP Solver selected is then used to solve for a new solution vector. |
