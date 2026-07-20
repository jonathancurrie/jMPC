---
title: "Embedded MPC Overview"
slug: "/embedded/"
---

One of the advanced functions of the jMPC Toolbox is an auto-coding framework specifically developed for generating high-speed embeddable C-code MPC controllers from a MATLAB `jMPC` object. The framework allows a complete MPC controller, QP solver, and associated linear algebra functions to be generated in ANSI C in as little as 50ms, allowing rapid-prototyping and quick re-tuning of a real MPC controller. In addition, the framework can automatically verify the generated code using auto-generated test benches, thus ensuring the code generated is bit accurate when compared to an equivalent computer implementation.

The resulting auto-coded MPC and QP functions are a combination of hand-coded, hand-optimized functions, together with auto-coded optimizations where pre-processing can be done to simplify the online computations by MATLAB. The resulting controllers and solvers have been tested on a variety of embedded hardware with a range of problem sizes and control systems and performed excellently. 

The pages below summarize the functionality provided by the toolbox.

- [Code Generation](./code-gen.md)
- [Memory Estimation](./mem-est.md)
- [Code Verification](./code-verify.md)
- [Processor In The Loop (PIL) Verification](./pil.md)

Additionally Chapter 4 of my [thesis](https://openrepository.aut.ac.nz/items/bb124ef8-830c-4446-80ec-5e5fc51f0bb0) contains a lot more detail than the above pages on the autocoding and embedded algorithm, as well PIL verification techniques.
