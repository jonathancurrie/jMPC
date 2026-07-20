---
title: "GetMPCToolboxObj"
slug: "/api/mpc/get-mpc-toolbox-obj/"
---

Convert the current `jMPC` object into a Matlab MPC Toolbox® object.

## Syntax

> `MPCobj = GetMPCToolboxObj(jMPCobj,jSIMobj)`

## Description

`MPCobj = GetMPCToolboxObj(jMPCobj,jSIMobj)` creates a new MATLAB [MPC Toolbox®](https://au.mathworks.com/help/mpc/index.html) object and configures it using the settings saved in the current `jMPC` and `jSIM` objects. Note you can also simulate an MPC Toolbox controller directly using the sim command with the `'MPCToolbox'` mode, or use the MPC Toolbox standalone.

This method is only in Beta and does not support all functionality provided by either the jMPC Toolbox or the MATLAB MPC Toolbox.
