---
title: "linearize"
slug: "/api/nonlinear/linearize/"
---

Linearizes a `jNL` object about an operating point.

## Syntax

> `[ltimodel,x_op] = linearize(jNLobj,u_op)`

> `[ltimodel,x_op] = linearize(jNLobj,u_op,x_op)`

> `[ltimodel,x_op] = linearize(jNLobj,u_op,x_op,odeSolver)`

## Description

`[ltiModel,x_op] = linearize(jNLobj,u_op)` linearizes the supplied `jNL` nonlinear model, using the supplied input operating point (`u_op`) to solve for the steady state of the system (`x_op`). See Steady State Considerations below for details on solving for the steady state, and Numerical Jacobian Considerations below for details on the linearization process. Note the returned linear model is a Control Systems Toolbox LTI State Space Model - chosen to enable the user to verify the linearized system using the analysis tools within this toolbox.

`[ltiModel,x_op] = linearize(jNLobj,u_op,x_op)` linearizes the supplied `jNL` nonlinear model, using the supplied input and state operating point. This method is preferred over the above function call, as this avoids the costly derivation of a steady state operating point which may not exist.

`[ltiModel,x_op] = linearize(jNLobj,u_op,[],odeSolver)` linearizes the supplied `jNL` using the selected ODE solver. Valid options are `ode45`, `ode23s` and `ode15s`. Note the operating state need not be supplied as typical use selecting the ODE solver infers we do not know the operating state. In most cases this should be left empty, as above.

An important point to note is that the linearized operating points must be included as biases inside the MPC controller, and thus `u_op`, `x_op` and `y_op` are all saved in the `UserData` property of the returned `ltiModel`. In R2009a I have noticed that if you perform any function on the returned `ltiModel` (e.g. `c2d`, `d2d`), `UserData` is erased! This will cause the MPC to function incorrectly, so please take note of this limitation - manually copying `UserData` if required.

## Solving for a Steady State

The new method used from version 3.10 and above uses a more robust method to determine the state operating point (if `x_o`p is not supplied). The algorithm uses a combination of tighter steady state tolerances and expanded to solution times to attempt to solve for a steady state using the user selected ODE solver (default is `ode45`). If it fails it will attempt to use `fsolve` from the last point of the ODE solver. If it still fails, then it will relax the tolerances and repeat. Note the algorithm is not 100% robust and you may find you are better specifying an operating point.

You should only use this function with a generous amount of caution.

## Numerical Jacobian Considerations

The core job of this function is to solve for the Jacobians of the nonlinear system, in other words, the gradient matrices. The gradient can be solved for analytically (pen and paper - or using the Symbolic Toolbox), or it can be approximated by finite difference. MATLAB includes a default numerical Jacobian solver (`numjac`) which has been utilized by `linearize` to obtain the state space **A**,**B**,**C**,**D** matrices. However, it is a well known problem that finite difference methods can fail, especially in the presence of non differentiable elements (discontinuities, hysteresis, etc), thus take care to verify the linearized solution is correct.

As above you should only use this function with a generous amount of caution.

## Robust Linearization

When you have a Simulink Model of your nonlinear system, it is recommended you use `trim` to solve for the steady state operating point, and `linmod2` to solve for the linearized system. Both of these functions are supplied with Simulink and use a more robust algorithm than the functions supplied with this toolbox.

## Example

Starting with the `jNL` object created in the [jNL Documentation](./jnl.md), we wish to linearize the model about `u = 0.5V` for both motors:

```matlab
%Linearize Plant
Model = linearize(Plant,[0.5 0.5]);  
```

By default the function will display the results in the command window:

```matlab
------------------------------------------------
Linearizing Model: nl_heli

Solving for Steady State about u = [0.5 0.5] ...
Solved using ode45
xss: 0 0 0 0 0 0

Numerically Constructing Jacobians....
Solving for A...Done
Solving for B...Done
------------------------------------------------
```

When we convert the Model to a jSS object:

```matlab
%Build jSS Object
Model = jSS(Model)
```

We note the class automatically detects the model is linearized:

```matlab
-- jMPC State Space Object --
States: 6
Inputs: 2
Outputs: 3
Continuous Model
Linearized about u: [0.5 0.5], x: [0 0 0 0 0 0]
```

Which will then propagate the necessary changes through when building the controller.
