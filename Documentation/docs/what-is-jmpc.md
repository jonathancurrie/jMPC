---
title: "What Is jMPC?"
slug: "/what-is-jmpc/"
---

jMPC is a MATLAB toolbox which is a collection of m-files, Simulink models and compiled C-MEX functions for use in simulating and controlling dynamic systems using Model Predictive Control. Key functionality offered by this toolbox include:

- Easily build linear MPC controllers.
- Simulating controllers within linear and nonlinear environments.
- Embed the resulting MPC controller with autocode generation in C.
- Applying linear inequality constraints to inputs and outputs.
- Various Quadratic Programming (QP) solvers are provided to solve the quadratic cost function.
- Implement advanced functionality such as state estimation, blocking and soft constraints.
- View real-time MPC control using the supplied Graphical User Interface (GUI).
- Connect the Simulink MPC block to real world dynamic systems using an A/D and D/A.
- Demonstrate MPC using 3D animations of real systems.

This work is the result of my research into embedded MPC during my time at AUT University during my Ph.D. in 2009-2012 (i.e. a long time ago!).

## Further Reading

My [thesis](https://openrepository.aut.ac.nz/items/bb124ef8-830c-4446-80ec-5e5fc51f0bb0) also contains a huge amount of information on the development of jMPC, its motivation and some interesting case studies. However for most users, jump to the [Documentation & Examples](./examples/index.md) page!
