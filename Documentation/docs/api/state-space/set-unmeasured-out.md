---
title: "SetUnmeasuredOut"
slug: "/api/state-space/set-unmeasured-out/"
---

Identify outputs which are unmeasured as part of the system. Unmeasured outputs allow a jMPC controller to be built where the controller knows it will not receive measurements (via **y<sub>p</sub>**) of the plant output (because it can't be measured), but it can be modelled ("predicted") and thus included in the controller itself. This allows constraints to be placed on this output, even though it can't be measured.

## Syntax

> `jSSobj = SetUnmeasuredOut(jSSobj,index)`

## Description

`jSSobj = SetUnmeasuredOut(jSSobj,index)` sets the system outputs of the supplied `jSS` object to unmeasured, as specified via the vector of indices in `index`.

## Example
See the [Servomechanism Case Study](../../case-studies/servo.md).
