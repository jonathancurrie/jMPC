---
title: "sim"
slug: "/api/nonlinear/sim/"
---

Simulate a `jNL` model over one sample step.

## Syntax

> `[x,y] = sim(jNLobj,x0,u,Ts)`

## Description

`[x,y] = sim(jNLobj,x0,u,Ts)` simulates the `jNL` object over the time horizon of `0` to `Ts` using a selected ODE solver. By default sim will use `ode45`, however as described below the user can customize this. `x0` is passed to the ODE solver as the initial states for integration, while the input `u` is passed as a default parameter to the nonlinear function.

## Numerical Integration Considerations

If your particular ODE system is more difficult (i.e. stiff, badly scaled) you may wish to try using a different ODE solver. If the `jNL` object has been linearized, you can specify the ODE solver to use for future simulations as part of the `linearize` arguments, or you can specify it using the [SetSolver method](./set-solver.md) of the `jNL` class.
