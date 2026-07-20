---
title: "Loading and Saving GUI Setup"
slug: "/api/gui/load-save/"
---

In order to be able to save & load an MPC configuration, three methods are provided:

## Import from / Export to Workspace

When you wish to use a setup locally, i.e. within the MATLAB workspace only, this option allows the use of `jGUI` objects, which are specific to this tool. They contain all options specified by the GUI, with the exception of simulation data (which is only read during an active simulation).

Both of these options are available via the respective File -> Load / File -> Save menus, and each will launch a separate window for loading or saving.

### Load From Workspace

This window is almost identical to the Load Example window, with the exception that this window scans the base MATLAB workspace for jGUI objects, rather than loading a pre-saved example.

![gui load work](/img/jmpc/gui_load_work.png)

### Save To Workspace

Use this method to save the current settings into a `jGUI` object, which will be saved to the base MATLAB workspace. Not you must have a working setup for this to work, as this option will rebuild a `jMPC` object from the current settings to save.

![gui save work](/img/jmpc/gui_save_work.png)

## Load From / Save To File

When you wish to use a permanent setup, or permanently save your MPC configuration, use the Load From / Save To File option to do so. Both of these options will open a standard file Window, and allow the user to specify a file name and location. Files will be saved as a .mat file, and thus only these can be loaded.

Note as above, only a working setup can be saved.

## Export MPC Controller

To use a controller created within the GUI, i.e. within Simulink or a standard jMPC simulation, you can export just the jMPC controller using this feature. A standard Save To Workspace window will be presented to choose a variable name for the controller.
