---
title: "Overloaded Methods"
slug: "/api/state-space/overload/"
---

The following methods are overloaded in the `jSS` class, although all require the [Control Systems Toolbox](https://au.mathworks.com/products/control.html).

## `ss`

`ltiobj = ss(jSSobj)` converts the `jSS` model to a MATLAB `lti` State Space Model.

## `c2d`

`jSSobj = c2d(jSSobj,Ts)` converts the continuous `jSS` model to discrete, using the sampling time `Ts`. If the model has been flagged to contain dead time (see [Models with Dead Time](./jss.md)), then this will be converted to state delay(s) at this point.

## `d2d`

`jSSobj = d2d(jSSobj,Ts)` resamples the discrete `jSS` model to a new sampling period, using the sampling time `Ts`. Note this function will not work if the model has dead time which has been converted to state delays, in which case the correct sampling period must be used by `c2d`.

## `d2c`

`djSSobj = d2c(jSSobj)` converts the discrete `jSS` model to continuous.

## `step`

`step(jSSobj)` performs a step response simulation and plots the result.

`[y,t] = step(jSSobj)` performs a step response simulation and saves the result in the output vectors.

## `impulse`

`impulse(jSSobj)` performs a impulse response simulation and plots the result.

`[y,t] = impulse(jSSobj)` performs a impulse response simulation and saves the result in the output vectors.

## `dlqe`

`K = dlqe(jSSobj)` uses default Gain (**G**) & Covariance Matrices (**Q**,**R**) (all Identity) to calculate the steady state discrete time observer gain, `K`, from the supplied state space model.

`K = dlqe(jSSobj,Q,R)` identical to above but allows the user to specify the `Q` and `R` matrices manually.

## `ssdata`

`ssdata(jSSobj)` displays the **A**, **B**, **C** and **D** state space matrices together with the sampling time and initial states in the command window.

`[A,B,C,D,Ts,x0] = ssdata(jSSobj)` returns the above data in the left hand variables.
