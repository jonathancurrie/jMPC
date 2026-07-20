---
title: "SetLinearization"
slug: "/api/state-space/set-linearization/"
---

Manually set the linearization points of the model. This function is useful if you know your model was derived about a non-zero linearization point and want to include the linearization biases in the controller (which you should!). This function is not required if you use the [`jNL.linearize`](../nonlinear/linearize.md) method from this toolbox.

## Syntax

> `jSSobj = SetLinearization(jSSobj,u_op,x_op)`

## Description

`jSSobj = SetLinearization(jSSobj,u_op,x_op)` sets the linearization points of the model to `u_op` (inputs) and `x_op` (states).
