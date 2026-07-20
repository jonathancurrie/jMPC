---
title: "jSS"
slug: "/api/state-space/jss/"
---

Create a State Space Model object.

## Syntax

> `jSSobj = jSS(LTIobj)`

> `jSSobj = jSS(A,B,C,D)`

> `jSSobj = jSS(A,B,C,D,Ts)`

> `jSSobj = jSS(A,B,C,D,Ts,x0)`

> `jSSobj = jSS(A,B,C,D,Ts,x0,u_op,x_op)`

## Description

`jSSobj = jSS(LTIobj)` creates a `jSS` object from a MATLAB Control Systems Toolbox Linear Time Invariant Object (`LTIobj`). This can be a State Space (`ss`), Transfer Function (`tf`) or Zero-Pole-Gain (`zpk`) object. For the later two objects, they will be first converted to State Space format using the Control function `ss`, and then all three will have the respective model matrices extracted and saved into the class.

`jSSobj = jSS(A,B,C,D)` creates a `jSS` object without using the Control Systems Toolbox. The user supplies the State Space matrices directly to the function.

`jSSobj = jSS(A,B,C,D,Ts)` specifies a sample time of the model. If no sample time is supplied, the model is assumed continuous (Ts = 0).

`jSSobj = jSS(A,B,C,D,Ts,x0)` specifies the initial states of the model as a column vector. When omitted all states are assumed to be zero.

`jSSobj = jSS(A,B,C,D,Ts,x0,u_op,x_op)` allows the user to specify the model has been linearized about an input operating point (`u_op`) and state operating point (`x_op`), and thus now uses deviation states.

## Models with Dead Time

`lti` Models supplied with dead time (`InputDelay`, `IODelay`, etc) are not directly compatible with the `jSS` class. However the class is written to automatically convert the dead time in `lti` models to internal state delays, using the Control function `delay2z`. An issue that arises is when the user does not supply a sample time, or supplies a continuous model. In order to avoid using a continuous approximation (e.g. `pade`), which will change the dynamic behaviour of the model, the dead time properties are saved until the model is converted to discrete. As the jMPC controller is a discrete implementation of MPC, all models must discrete, and `c2d` will be called automatically, ensuring the final conversion of dead time into states (poles at zero).

## Examples

Converting a MATLAB `lti` object to a `jSS` object:

```matlab
%Model
Gs = tf(2,[0.7 0.2 1]);
Ts = 0.1;
%Convert to Discrete
Gd = c2d(Gs,Ts);
%Create a jSS Object
Model = jSS(Gd);  
```

Creating a `jSS` Object Directly:

```matlab
%Model
A = [0.6000 -0.4000 0.2000;      
     1.0000 0.4000 0.4000;     
     0.8000 1.0000 0.4000];
B = [0.4  0.3;       
       0   -1;       
       0  0.1]/2;
C =[1.0000 -2.2000 1.1200;    
    0       1      1    ];
D = zeros(2,2);
Ts = 0.1;

%Create Object
Model = jSS(A,B,C,D,Ts);
%Set Initial States
Model.x0 = [0.3 0.1 0]';     
```

Also see the [Quad Tank Case Study](../../case-studies/quad-tank.md) for use of this class.
