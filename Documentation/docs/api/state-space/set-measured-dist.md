---
title: "SetMeasuredDist"
slug: "/api/state-space/set-measured-dist/"
---

Identify inputs which are measured disturbances of the system. Measured disturbance inputs are inputs which cannot be controlled by e.g. a jMPC Controller, but are able to be measured (e.g. an incoming temperature) and thus included in the dynamics of the system.

## Syntax

> `jSSobj = SetMeasuredDist(jSSobj,index)`

## Description

`jSSobj = SetMeasuredDist(jSSobj,index)` sets the system inputs of the supplied `jSS` object to measured disturbances, as specified via the vector of indices in `index`.

## Example
See the [CSTR Case Study](../../case-studies/cstr.md).
