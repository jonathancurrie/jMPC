---
title: "GUI Auto Setup Functionality"
slug: "/api/gui/auto-setup/"
---

To speed up users familiar with the `jMPC object` and using scripts to run the simulation, this button allows the GUI to check the workspace to find setup options with default variable names:

- `Np`
- * Prediction Horizon (integer)

- `Nc`
- * Control Horizon (integer)

- `uwt`
- * Input tuning weights (vector)

- `ywt`
- * Output tuning weights (vector)

- `con`
- * Constraint structure in the form:

> `con.u` is `[Umin Umax Delumax]` (matrix)
> `con.y` is `[Ymin Ymax]` (matrix)

- `setp`
- * Setpoints (matrix)
- * Note only the first row of this is used (first setpoint)

- `Ts`
- * Sampling time of the controller (scalar)

All variables must be defined exactly as above. Note you must have a model loaded first before you can use this feature. An example m-file is presented below:

```matlab
%Plant
G1p = tf({[1 -1],[1 2]},{[1 1],[1 4 5]});
%Model
G1m = tf({[1.2 -1.1],[1.5 2.2]},{[1.4 1.7],[1.2 4.4 5.1]});

Np  = 10;
Nc = 5;

uwt = [0.1 0.2]';	%[in1 in2]
ywt = [0.1]';	%[out1]
con.u = [-5 5 2.5;
         -5 5 2.5];	%[umin umax del_umax]
con.y = [-5 5];	%[ymin ymax]
setp = 1;	%[setp1]
Ts = 0.1;
```
