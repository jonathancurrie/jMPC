---
title: "jMPCset"
slug: "/api/mpc/jmpcset/"
---

Create or edit a `jMPC` options structure.

## Syntax

> `options = jMPCset('param1',value1,'param2',value2,...)`

> `jMPCset`

> `options = jMPCset`

> `options = jMPCset(oldopts,'param1',value1,...)`

## Description

The function `jMPCset` creates an options structure that you can pass to the `jMPC` constructor for advanced setup options for your MPC controller. This function will automatically fill in defaults for fields required to run an MPC simulation. The remainder of the fields will be set to `[]`.

`options = jMPCset('param1',value1,'param2',value2,...)` creates an MPC options structure called `options`, in which the specified parameters (`param`) have the specified values (`value`).

`jMPCset` with no input or output arguments displays a complete list of parameters with their valid values.

`options = jMPCset` with no input arguments creates a default options structure.

`options = jMPCset(oldopts,'param1',value1,...)` updates the existing options structure, `oldopts`, with the parameters and values specified in `param` and `value`.

## Options

For a complete list of solver options use

```matlab
>> jMPCset
```

## Example

Create an options structure to use setpoint look ahead with jMPC's `quad_wright` as the QP solver:

```matlab
options = jMPCset('Solver','quad_wright','LookAhead',1);
```
