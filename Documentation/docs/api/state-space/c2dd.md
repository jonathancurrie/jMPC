---
title: "c2dd"
slug: "/api/state-space/c2dd/"
---

Converts the continuous `jSS` model to discrete with explicit computational dead time added. This function can be used to model deadtime as a result of computational delay in a real (implemented) MPC controller where the control calculation is not "instantaneous". 

## Syntax

> `jSSobj = c2dd(jSSobj,Ts,tau)`

## Description

`jSSobj = c2dd(jSSobj,Ts,tau)` converts the supplied continuous `jSS` object to discrete using the sampling time `Ts`. This function also models computational dead time which is specified via `tau`. Note tau must be less than or equal to `Ts`. This function is based on the function presented in [1].

<small>Reference: [1] Maciejowski, J. M., Predictive Control with Constraints, Prentice Hall, 2002</small>
