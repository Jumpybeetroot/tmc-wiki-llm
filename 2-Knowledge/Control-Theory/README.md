# Control Theory

**Type:** knowledge
**Summary:** Motor control theory — microstepping, current chopping, PID control, S-curve acceleration, back-EMF, and related concepts.
**Tags:** #control-theory #motion #reference
**Status:** active
**Updated:** 2026-04-08

---

This directory contains knowledge articles on stepper motor control theory, algorithms, and motion planning concepts.

## Pages

- [[PID-Tuning-Methods]] — Overview of classical and modern PID tuning methods (ZN, Cohen-Coon, Lambda/IMC, relay autotune, Tyreus-Luyben, direct synthesis).
- [[SIMC-PID-Tuning]] — Skogestad's SIMC method for PI/PID tuning. Model-based, single tuning parameter τc, analytically derived from FOPDT.
- [[Sliding-Mode-Control]] — Sliding mode control theory and application to linear/rotary motors. Robust nonlinear control with chattering mitigation.
- [[Feedforward-Compensators]] — Feedforward compensator design for motion control. Velocity/acceleration feedforward, IMC-based design, combined FB+FF architecture.
- [[SMC-PMSM-Survey]] — Systematic review of SMC for PMSM speed control (2020–2025). Six-family taxonomy, 66 cataloged papers, computational cost analysis, research gaps.
- [[k4671-Feedforward]] — Dedicated feedforward implementation in k4671 firmware. Three-term software FF, saturation nonlinearity, PidTorqueFluxOffset injection.
