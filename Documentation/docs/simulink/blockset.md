---
title: "Simulink Block Set"
slug: "/simulink/blockset/"
---

The toolbox comes with a small library of Simulink blocks for use in Simulink simulations.

## Discrete MPC Controller

![block mpc](/img/jmpc/block_mpc.png)

The Discrete MPC block receives the current plant output (`yp`), setpoint (`setp`) and measured disturbance (`mdist`), and at each sampling instant (set by the controller object), will calculate the optimal plant input (`u`) in order to control the plant to the setpoint, such that the constraints are not violated, provided this is feasible. Also provided by the block are the input increments (`delu`), model output (`ym`), model states (`xm`) and solver statistics for simulation analysis.

Importantly, this block is a Simulink S Function written in C to accelerate computations, and has been written to avoid high memory overhead associated with the large number of variables. This block forms the foundation of jMPC implementations we have run on real equipment.

The Discrete MPC Controller block only requires one input parameter, a `jMPC` object containing the controller specification which has been built offline. From this object, the required properties and values will be automatically propagated through the underlying blocks and variables. The same `jMPC` object created for MATLAB simulations can also be used within this block without modification. Consult the help section on the [MPC Controllers](../api/mpc/index.md) for more details on creating a `jMPC` object.

## Discrete LTI Plant

![block lti](/img/jmpc/block_lti.png)

This block was designed to allow the toolbox to function without the Control Systems Toolbox. It implements the very basic academic model of a state space system, with its only advantage being that it is setup to accept a `jSS` object directly. The input to the system is designated (`u`), while the plant output and states and designated `y` and `x`, respectively.

As with the Discrete MPC blocks, this block only requires a single input parameter, being the `jSS` object containing the model specification. Consult the help section on the [State Space Models](../api/state-space/index.md) for more details on creating this object.

## Continuous ODE Plant

![block ode](/img/jmpc/block_ode.png)

This block uses the Simulink integrator to numerically integrate a system of Ordinary Differential Equations (ODEs) supplied by the user within a `jNL` object. The input to the system is designated (`u`), while the plant output and states and designated `y` and `x`, respectively.

As with the Discrete MPC blocks, this block only requires a single input parameter, being the `jNL` object containing the model specification. Consult the help section on the [Nonlinear Models](../api/nonlinear/index.md) for more details on creating this object.
