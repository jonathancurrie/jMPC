---
title: "augment"
slug: "/api/state-space/augment/"
---

Augment the model's output to the state matrix.

## Syntax

> `augjSSobj = augment(jSSobj)`

## Description

This method is used to embed an integrator within the supplied model, effectively augmenting the output *y(k)* to the state vector *x(k)*. Given the standard state space linear equation:

![eq ss](/img/jmpc/eq_ss.png)

and following the steps in [1, p4], by applying a difference operation on both sides, leads to the following state space model:

![eq aug](/img/jmpc/eq_aug.png)

which is the augmented form of the model. Note in this implementation the state space matrix **D** is omitted for simplicity.

Based on the above equation, the optimization variable is now the increment of the input (*Δu*) versus the original absolute input (*u*). This also has an effect on the state vector, which also becomes the state increment (*Δxm*). This step is required for the jMPC controller, as the Quadratic Programming problem is formed in relation to the input increment.

For those interested in this step further, they are referred to read [2, p66] and [3, p49] for details on deriving the above expression.

<small>References: [1] Wang, L., Model Predictive Control System Design and Implementation Using MATLAB, Advances in Industrial Control, 2009; [2] Rossiter, J. A., Model Based Predictive Control, A Practical Approach, CRC Press, 2004; [3] Maciejowski, J. M., Predictive Control with Constraints, Prentice Hall, 2002.</small>
