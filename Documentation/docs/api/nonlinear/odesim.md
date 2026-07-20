---
title: "odesim"
slug: "/api/nonlinear/odesim/"
---

Simulate a `jNL` model over a specified time period using a constant input.

## Syntax

> `[t,y] = odesim(jNLobj,x0,u,T)`

> `[t,y] = odesim(jNLobj,x0,u,T,odeopts)`

## Description

`[t,y] = odesim(jNLobj,x0,u,T)` simulates the `jNL` object over the time horizon of `0` to `T` using a selected ODE solver, and will return a vector of outputs (`y`) at specified times (`t`) as per a default ODE evaluation. By default `odesim` will use `ode45`, however as described below the user can customize this. `x0` is passed to the ODE solver as the initial states for integration, while the constant input `u` is passed as a default parameter to the nonlinear function.

## Numerical Integration Considerations

If your particular ODE system is more difficult (i.e. stiff, badly scaled) you may wish to try using a different ODE solver. If the `jNL` object has been linearized, you can specify the ODE solver to use for future simulations as part of the `linearize` arguments, or you can specify it using the [SetSolver method](./set-solver.md) of the `jNL` class.
