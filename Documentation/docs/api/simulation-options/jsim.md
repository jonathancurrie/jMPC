---
title: "jSIM"
slug: "/api/simulation-options/jsim/"
---

Create a Simulation Environment for an MPC Simulation.

## Syntax

> `jSIMobj = jSIM(jMPCobj,Plant,T,setp)`

> `jSIMobj = jSIM(jMPCobj,Plant,T,setp,umdist)`

> `jSIMobj = jSIM(jMPCobj,Plant,T,setp,umdist,mdist)`

> `jSIMobj = jSIM(jMPCobj,Plant,T,setp,umdist,mdist,noise)`

## Description

`jSIMobj = jSIM(jMPCobj,Plant,T,setp)` creates a `jSIM` object which is used by `jMPC/sim` for running a simulation of your controller in a given simulation environment. In order to verify sizes & settings between the controller and simulation, the `jMPC` object must be built first, and passed to this function. The variable `Plant` can be a `jSS` or a `jNL` object, which represents the 'real' plant that you wish to control. This does not have to be the same as the controller model, but if it is a state space model, it must be sampled at the same rate. `T` is the length of the simulation (in samples) and `setp` is a matrix containing the setpoint at every sample specified by the length of `T`. Each column of `setp` is an individual output, while the rows are each sample. Note for constant setpoints it is possible to just enter the setpoint at sample 1 (as a row vector).

`jSIMobj = jSIM(jMPCobj,Plant,T,setp,umdist)` allows the user to specify a matrix of unmeasured input disturbances, as detailed in the disturbances section below. `umdist` follows the same conventions as `setp`, where each column is an input, and each row is a sample. Unmeasured disturbances are always specified as a *delta* value, such that they are added to the respective input / output.

`jSIMobj = jSIM(jMPCobj,Plant,T,setp,umdist,mdist)` allows the user to specify a matrix of measured input disturbances, as detailed in the section below. `mdist` follows the same conventions as above. Measured disturbances are always specified as an *absolute* value, such that they are used as the value applied directly to the plant.

`jSIMobj = jSIM(jMPCobj,Plant,T,setp,umdist,mdist,noise)` allows the user to specify a matrix of unmeasured output disturbances (measurement noise), as detailed in the section below. `noise` follows the same conventions as above. Unmeasured disturbances are always specified as a *delta* value, such that they are added to the respective input / output.

## Adding Disturbances

You can currently add three types of disturbances to the simulation, unmeasured inputs and outputs, and measured inputs. Measured disturbances are modelled as part of the MPC problem, while unmeasured disturbances are not and thus must be corrected by a state estimator or embedded integrator.

## Example

Create a simulation scenario for a SISO plant with a time varying setpoint, as well as unmeasured input and output disturbances.

```matlab
%Plant
Gs = tf(2,[0.7 0.2 1]);
Ts = 0.1;
%Convert to Discrete
Gd = c2d(Gs,Ts);
%Create a jSS Object
Plant = jSS(Gd);

%Simulation Duration
T = 200;
%Setpoint
setp = ones(T,1);
setp(75:150) = 0.5;
%Unmeasured Disturbances
umdist = zeros(T,1);
umdist(60:120) = -0.3;
noise = rand(T,1)/50;

%Create MPC object here...

%Create simulation options
simopts = jSIM(MPCobj,Plant,T,setp,umdist,[],noise);
```
