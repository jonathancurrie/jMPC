---
title: "removebias"
slug: "/api/simulation-options/removebias/"
---

Remove linearization biases from simulation results.

## Syntax

> `jSIMobj = removebias(jSIMobj,lin,index,opts)`

## Description

`jSIMobj = removebias(jSIMobj,lin,index,opts)` takes the `jSIM object` containing the results in the field `plotvec`, and returns the input, output, states and set point such that they all reflect true values, in a new `jSIM` object. This method is called automatically when you run a jMPC simulation, with the fields `lin`, `index` and `opts` filled by the `jMPC` object. For manual use the fields required for each are as follows:

|  |  |
| --- | --- |
| `lin.u_op` | Input operating points |
| `lin.x_op` | State operating points |
| `lin.y_op` | Output operating points |
| `index.iq_out` | Indices of controlled outputs |
| `opts.LinSim` | True if a linear plant is being used in a simulation with linearization biases, otherwise this is false. |

## Working with Operating Points

When an MPC simulation is run with a linearized model the resulting jSIM object containing the results will have operating point biases on the recorded inputs, outputs and states. This may or may not be intended, depending on your preference. Linearized models operate in deviation state space, where the model values returned are at deviations from the desired operating point, not the actual true states or output values.
