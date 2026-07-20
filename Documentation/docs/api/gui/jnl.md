---
title: "Using jNL Models"
slug: "/api/gui/jnl/"
---

In order to more accurately simulate a given process, you can use a `jNL` object as the Plant. This allows you to use nonlinear (or linear) Ordinary Differential Equations (ODEs) to represent the Plant Dynamics, while retaining a simpler, linear state space model as the internal controller model. See the [help section](../nonlinear/index.md) on `jNL` objects on how to create the Plant, and the limit of the associated object.

Note you must still supply a linear time invariant (lti) model for the controller, as the `jMPC` object does not currently support nonlinear models.

![gui j NL](/img/jmpc/gui_jNL.png)

Due to increased computation of the simulation (an ODE solver), on older machines you may experience a noticeable performance slow down in the GUI.
