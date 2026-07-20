---
title: "Simulation Options"
slug: "/api/simulation-options/"
---

Due to the `jMPC` object containing just the controller specifications, the `jSIM` object is used to specify the simulation options. This provides added flexibility such that one can change the simulation environment without rebuilding the controller, or vice versa. The `jSIM` object can also be tagged as a results object, enabling it to contain simulation results for plotting.

|  |  |
| --- | --- |
| [`jSIM`](./jsim.md) | Create a MPC simulation environment `jSIM` object. |
| [`removebias`](./removebias.md) | Remove linearization biases from results. |
