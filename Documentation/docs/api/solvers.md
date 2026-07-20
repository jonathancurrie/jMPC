---
title: "Quadratic Programming Solvers"
slug: "/api/solvers/"
---

As detailed in the [Quadratic Programming Overview](../concepts/quadratic-programming.md) jMPC Toolbox comes with several Quadratic Programming (QP) solvers suited for the MPC problems the toolbox was designed for (small problems, short horizons). These are summarized below.

|  |  |  |  |
| --- | --- | --- | --- |
| **Function** | **Implementation** | **Precision** | **Reference** |
| `quad_wright` | MATLAB m-code | Double | [1] |
| `squad_wright` | MATLAB m-code | Single | [1] |
| `mquad_wright` | MATLAB MEX | Double | [1] |
| `msquad_wright` | MATLAB MEX | Single | [1] |
| `quad_mehrotra` | MATLAB m-code | Double | [2] |
| `squad_mehrotra` | MATLAB m-code | Single | [2] |
| `mquad_mehrotra` | MATLAB MEX | Double | [2] |
| `msquad_mehrotra` | MATLAB MEX | Single | [2] |

The default implementation and algorithm used by jMPC is the `mquad_wright` function, which is generally fastest and supports standard MATLAB precision (double). For detailed information on the solvers, including their algorithms and implementation, consult Chapter 3 in my [thesis](https://openrepository.aut.ac.nz/items/bb124ef8-830c-4446-80ec-5e5fc51f0bb0). For those interested, both algorithms above use Infeasible Interior Point (IIP) methods for solving the constrained optimization problem.

Algorithm References:
- [1] S. J. Wright, "Applying New Optimization Algorithms to Model Predictive Control," in Chemical Process Control-V, CACHE, AIChE Symposium, 1997, pp. 147-155
- [2] Object Orientated Software for Quadratic Programming by E. Gertz and S. Wright. University of Wisconsin-Madison.
