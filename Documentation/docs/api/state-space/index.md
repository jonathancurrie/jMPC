---
title: "State Space Models"
slug: "/api/state-space/"
---

The `jSS` object is an alternative to the [Control Systems Toolbox](https://au.mathworks.com/products/control.html) `lti` object and allows use of the jMPC Toolbox without the Control Systems Toolbox installed. The `jSS` class stores the model in the basic state space form. It is however highly recommended that the Control Systems Toolbox is installed in order to fully utilize this toolbox. Documentation of the `jSS` class and methods of it are listed below.

|  |  |
| --- | --- |
| [`jSS`](./jss.md) | Create a State Space (SS) Model |
| [`sim`](./sim.md) | Simulate the model over one sample |
| [`SetMeasuredDist`](./set-measured-dist.md) | Set one or more inputs as measured disturbances |
| [`SetUnmeasuredOut`](./set-unmeasured-out.md) | Set one or more outputs as unmeasured outputs |
| [`SetLinearization`](./set-linearization.md) | Manually set the linearization points of the model |
| [`ss`](./overload.md) | Convert a `jSS` object to a Matlab State Space `lti` object |
| [`c2d`](./overload.md) | Convert a continuous model to discrete |
| [`c2dd`](./c2dd.md) | Convert a continuous model to discrete with computational dead time |
| [`d2d`](./overload.md) | Resample a discrete model |
| [`d2c`](./overload.md) | Convert a discrete model to continuous |
| [`step`](./overload.md) | Perform a step test on the model |
| [`impulse`](./overload.md) | Perform an impulse test on the model |
| [`dqle`](./overload.md) | Design a discrete observer based on the model |
| [`augment`](./augment.md) | Augment the model's outputs to the state matrix |
| [`ssdata`](./overload.md) | Extract the state space matrices from the object |
