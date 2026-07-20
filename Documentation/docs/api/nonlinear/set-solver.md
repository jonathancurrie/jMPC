---
title: "SetSolver"
slug: "/api/nonlinear/set-solver/"
---

Set the ODE solver used for `jNL` model simulations.

## Syntax

> `SetSolver(jNLobj,odeSolver)`

## Description

`SetSolver(jNLobj,odeSolver)` specifies the ODE solver to be used for future simulations of the `jNL` object. Valid options are `'ode45'`, `'ode23s'`, or `'ode15s'`.
