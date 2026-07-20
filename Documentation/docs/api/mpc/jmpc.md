---
title: "jMPC"
slug: "/api/mpc/jmpc/"
---

Create a jMPC Model Predictive Controller object.

## Syntax

> `jMPCobj = jMPC(Model,Np,Nc,uwt,ywt)`

> `jMPCobj = jMPC(Model,Np,Nc,uwt,ywt,con)`

> `jMPCobj = jMPC(Model,Np,Nc,uwt,ywt,con,Kest)`

> `jMPCobj = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)`

## Description

`jMPCobj = jMPC(Model,Np,Nc,uwt,ywt)` creates an MPC controller `jMPC` object, using the supplied `Model`, Prediction Horizon, `Np`, and Control Horizon, `Nc`. Optionally `Nc` can also be a vector of blocking moves at which the input trajectory is calculated. If blocking is used, the sum of the elements must equal the Prediction Horizon. `Model` must be a discrete `jSS` State Space Model. `Np` and `Nc` are integers where `Np` ≥ `Nc`. You must also specify the objective function weighting vectors, `uwt` for weighting on changes of the input (**R** Matrix), and `ywt` for output weighting (**Q** Matrix). These five input arguments are the minimum required by this toolbox to create an unconstrained MPC controller.

`jMPCobj = jMPC(Model,...,ywt,con)` allows the user to specify constraints on the inputs, input increments and outputs, in the following structure form:  

> `con.u = [umin umax delumax];`

> `con.y = [ymin ymax];`

Where each row specifies the respective input / output. `delumax` specifies both the maximum and minimum rate of change of the input, `u`. To only constrain some values, replace the unconstrained values with `inf`. Note however, `delumax` must be finite if the controller is constrained. Optionally you may also specify a slack term in order use 'soft constraints' on the output constraints. This allows the QP problem to remain feasible even when the output constraints may be broken. Soft constraints cannot be applied to the input constraints as these are typically hard constraints. The magnitude of the slack term infers the penalty associated when the output breaks a constraint:

> `con.slack = 1e3;`

Smaller terms penalize the output less and thus the constraints is more likely to be broken. Avoid slack values > 50e3 for numerical stability of the QP problem. Optionally you may also specify slack weights for each output:

> `con.slack = [1e3 1e3 Inf];`

Where for the above example outputs 1 and 2 are soft constrained using the respective slack weights, and output 3 is hard constrained (identified by using `Inf`). You must specify a slack weight for each output using this method.

`jMPCobj = jMPC(Model,...,con,Kest)` specifies the state estimator gain, `Kest`. When omitted, it is assumed there is no state estimation (no observer).

`jMPCobj = jMPC(Model,...,Kest,opts)` allows the user to specify advanced options for creating the controller. See the section on [jMPCset](./jmpcset.md) for more details on this parameter.

## Example

Build an MPC Controller for a Linearized Cessna Model [1] with the following Continuous State Space Model:

We wish to constrain the input pitch angle to ±15° and rate of change to ±30°, and for outputs, the pitch angle to ±20° and the maximum rate of altitude change to 30m/s  

```matlab
%Plant - Linearized Continuous Time Cessna Model
A = [-1.2822   0     0.98      0;     
     0         0     1         0;     
     -5.4293   0     -1.8366   0;     
     -128.2    128.2 0         0];
 
B = [-0.3;     0;     -17;     0];
 
C = [0         1      0        0;     
     0         0      0        1;    
     -128.2    128.2  0        0];

D = zeros(3,1);

%Build jSS object
Plant = jSS(A,B,C,D);
%Discretize model
Ts = 0.5;
Plant = c2d(Plant,Ts);

%-- MPC Setup --%
%MPC Model
Model = Plant; %no model-plant mismatch
%Horizons
Np = 10;
Nc = 5;

%Constraints
con.u = [-0.2618 0.2618 0.5236];  %limit u to +- 15deg and rate to 30
con.y = [-0.35   0.35             %limit pitch angle to +- 20deg         
         -inf    inf;         
         -inf    30];             %limit altitude rate to +30deg/s

%Weighting
uwt = 1;
ywt = [1 1 1]';

%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC Object--%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest);
```

<small>Reference: [1] Maciejowski, J. M., Predictive Control with Constraints, Prentice Hall, 2002.</small>
