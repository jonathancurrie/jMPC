---
title: "Changing GUI Options"
slug: "/api/gui/options/"
---

To accommodate different user preferences, the following options are available from the options menu:

## Auto Scaling

Auto scaling refers to the ability of the scrolling plots to resize in real time. When this option is enabled, and the process is at steady state, the response windows will begin to rescale, such that window will zoom back to the response lines. This is an advantage when a process has become unstable, and you wish to zoom back to the detail of the plot.

Performing multiple small step changes will encourage the plot to rescale to smaller sizes. If you wish to keep the plot size constant (i.e. it will get bigger but not smaller), then disable this option. This option is enabled by default.

## Auto Calculate Ts

Ts is the sampling time of the controller, and must be chosen with consideration of the speed of the model. To aid amateur users from selecting sample times too fast or slow for a given process, this option will perform a step test of the model, and extract the sample time and length of simulation. It will then use these values for choosing the Controller Sampling Period (Ts) and Window Length.

Disable this option to manually select these values. This option is enabled by default.

Note: You must have the Control Systems Toolbox® to use this option.

## Data Logging

If you wish to log the data displayed by the GUI, then enable this option. This can be useful for later analysis, as you can view the entire simulation, not just the window size.

Note that not just the controller inputs and plant outputs are returned, but also the model output, plant & model states, and the rate of change of the inputs.

With this option enabled, three objects are automatically created in the base workspace:

- `plotvec`
- * This is a structure containing all the elements listed above, such that it can be accessed by a user's plotting function easily

- `simresult`
- * This is a `jSIM` object, which contains the same information as `plotvec`, but within one of the properties of the object. The advantage of this is the plot command is overloaded for this object, so plotting MPC responses is simple.

- `MPC1`
- * This is a `jMPC` object, used for plotting the results, above.  

To plot the response using the `jSIM` object, use the following command:

```matlab
>> plot(MPC1,simresult)
```

Which is described in more detail [here](../mpc/plot.md).

## Demo Mode

A visual feature only demo mode allows the GUI to automatically vary the setpoints every 100 samples, such that it provides a live demonstration. The algorithm will randomly select an output to modify, and randomly choose a new setpoint. The new setpoint will be on a normal distribution, with a mean of 1, and a standard deviation of 2. The algorithm does not respect any constraints on the system.
