---
title: "Nonlinear Models"
slug: "/api/nonlinear/"
---

One of the advanced features of the toolbox is the ability to use a nonlinear model as the plant during simulation. This enables the user to create a more accurate simulation, and thus a more realistic validation of controller tuning. In order to use a nonlinear model, the user must create a `jNL` object with the necessary properties. Documentation of the `jNL` class and methods of it are listed below.

|  |  |
| --- | --- |
| [`jNL`](./jnl.md) | Create a Nonlinear Model |
| [`sim`](./sim.md) | Simulate the model over one sample |
| [`odesim`](./odesim.md) | Simulate the model over a fixed time period with a constant input |
| [`linearize`](./linearize.md) | Linearize a nonlinear model about an operating point |
| [`SetSolver`](./set-solver.md) | Manually set the ODE solver used for simulating the model |
