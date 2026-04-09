# SMC-Based PMSM Speed Control — Survey & Taxonomy (2020–2025)

**Type:** knowledge
**Summary:** Systematic review and taxonomy of sliding mode control methods for PMSM speed regulation (2020–2025), covering 66 cataloged studies across six SMC families with comparative analysis of robustness, chattering, computation, and research gaps. Part I of a two-part series (Part II is a companion simulation study).
**Tags:** #control-theory #smc #pmsm #survey #advanced
**Status:** draft
**Updated:** 2026-04-09
**Source:** Badamasi, Mubarak & Ali Nasir, "SMC-based PMSM Speed Control: A Comprehensive Review and Taxonomy (2020–2025)," arXiv:2510.18420v1, 2025. KFUPM, Saudi Arabia.
**Related:** [[Sliding-Mode-Control]], [[PID-Tuning-Methods]], [[Feedforward-Compensators]], [[SMC-PI-Cascade]], [[GESO-SMC-RL-Feed]]

---

## Overview

This page summarizes a comprehensive survey of sliding mode control (SMC) techniques applied specifically to **permanent magnet synchronous motor (PMSM) speed control**, covering the period 2020–2025. The survey follows PRISMA methodology: 292 Scopus records screened → 138 retained → 66 cataloged in detail.

The paper complements the existing [[Sliding-Mode-Control]] page (based on Yu et al. 2023), which covers PMLSM *position* control with deep theory. This survey focuses on rotary PMSM *speed* control, provides a broader taxonomic classification, analyzes computational requirements, and identifies research gaps.

**Key finding:** No universal "best" SMC algorithm exists — the choice is application-dependent. Composite controllers (observer + SMC variant + intelligent tuning) consistently outperform standalone approaches. Super-twisting SMC and adaptive SMC offer the best robustness-to-smoothness ratio for most practical PMSM drives.

---

## PMSM System Model

Standard d-q frame model used across all surveyed works:

**Voltage equations:**
```
u_d = R_s·i_d + L_d·(di_d/dt) − ω_e·L_q·i_q
u_q = R_s·i_q + L_q·(di_q/dt) + ω_e·L_d·i_d + ω_e·ψ_f
```

**Electromagnetic torque** (SPMSM, L_d = L_q):
```
T_e = (3/2)·p_n·ψ_f·i_q
```

**Mechanical dynamics:**
```
J·dω_r/dt = T_e − B·ω_r − T_L
```

Where: J = inertia, B = viscous friction, T_L = load torque, p_n = pole pairs, ψ_f = PM flux linkage.

Control objectives: speed regulation under load disturbance, parameter uncertainty rejection, chattering suppression, fast transient response.

---

## SMC Taxonomy

The survey classifies all 66 studies into six primary SMC families, with cross-cutting categories for observers, intelligent methods, and optimization.

### 1. Conventional SMC (CSMC)

Linear sliding surface s = c·e + ė with discontinuous sign-based switching.

**Strengths:** Simple design, low computation, strong theoretical foundation.
**Weakness:** Chattering — the fundamental tradeoff.

**2020–2025 enhancements:**
- Modified reaching laws (exponential, power-rate) to reduce chattering
- Extended state observer (ESO) integration for disturbance estimation
- Disturbance observers: DOB, enhanced sliding-mode disturbance observer (ESMDO)
- Barrier-function SMC — bounds switching gain without disturbance upper bound knowledge
- MPC-SMC hybrids — model predictive control for current loop, SMC for speed loop
- Fuzzy SMC — fuzzy logic tunes switching gain adaptively
- Adaptive reaching laws tied to state error magnitude

**Computational cost:** Lowest of all SMC families.

### 2. Terminal SMC (TSMC)

Nonlinear sliding surface guaranteeing **finite-time convergence** to equilibrium.

**Variants cataloged:**
- **NTSMC** — nonsingular terminal SMC, avoids the singularity problem of basic TSMC
- **NFTSMC** — nonsingular fast terminal SMC, combines fast + nonsingular properties
- **Fractional-order complementary TSMC + NN** — fractional calculus surface with neural network disturbance compensation
- **Fuzzy GFTSMC + Luenberger observer** — global fast terminal SMC with fuzzy gain tuning
- **Event-triggered TSMC + GA-optimized ESO** — reduces computational load via event-triggered updates
- **Fault-tolerant NTSMC** — maintains performance under sensor/actuator faults
- **Discrete-time TSMC** — direct DSP implementation without continuous-time approximation
- **SNTSMC** — smooth nonsingular terminal SMC
- **Backstepping NTSMC + bounded-time DOB** — recursive design with finite-time disturbance estimation

**Computational cost:** Moderate — nonlinear surface evaluation adds overhead vs. CSMC.

### 3. Integral SMC (ISMC)

Integral term in sliding surface **eliminates the reaching phase** — trajectory starts on the sliding manifold from t = 0. Ensures invariance from the beginning.

**Combinations cataloged:**
- ISMC + ESO — extended state observer estimates total disturbance
- ISMC + high-order observer (HOO)
- ISMC + fuzzy sliding-mode observer (SMO)
- ISMC + boundary layer — continuous approximation for chattering reduction
- Second-order ISMC (SOISMC)
- Fractional-order ISMC extensions
- PSO-tuned ISMC — particle swarm optimization for offline parameter selection
- H∞ + ISMC hybrid — combines H∞ robustness guarantee with SMC invariance

**Computational cost:** Low to moderate — integral term is cheap but observer adds cost.

### 4. Higher-Order SMC (HOSMC)

Enforces s = ṡ = ··· = s^(r-1) = 0, producing a control signal with r-1 continuous derivatives. The discontinuity is pushed to the r-th derivative, effectively eliminating chattering from the control output.

#### 4.1 Super-Twisting Algorithm (STA)

The most widely used HOSMC variant. Continuous control output with only internal state derivative discontinuous.

```
u   = −α·|s|^(1/2)·sgn(s) + v
v̇   = −β·sgn(s)
```

**STA variants cataloged:**
- FSTA-SMC — fast super-twisting with accelerated convergence
- STA + DOB — super-twisting with disturbance observer
- PSO-optimized STA — gains tuned offline by particle swarm optimization
- SVR observer + STA — support vector regression observer for disturbance estimation
- Composite HOSTSM + DBO — higher-order super-twisting with dung beetle optimizer
- Hybrid STA + BAS — beetle antennae search for online gain adaptation

#### 4.2 Adaptive HOSMC
- Lyapunov-based adaptive laws for gain adjustment
- Generalized STA (GSTA) + sinusoidal saturation function for smooth switching

#### 4.3 Observer-Based HOSMC
- Enhanced fast STA sliding-mode observer — enables sensorless PMSM control
- High-order sliding-mode state observer (HOSTO) for velocity/disturbance estimation
- Full-order adaptive SMC + ESO

#### 4.4 Second-Order SMC (SOSMC)
- NN compensator for model uncertainty
- Singular perturbation theory approach (separates fast/slow dynamics)
- DOB-enhanced SOSMC
- MPC dual-loop with SOSMC speed controller

#### 4.5 Hybrid HOSMC
- MPC + STA — predictive current control with super-twisting speed loop
- Deadbeat predictive current + STA — fastest current response combined with robust speed loop

#### 4.6 Fault-Tolerant HOSMC
- High-resistance fault compensation
- Global fast terminal HOSMC
- NFTSMC + HOSMC + exponential reaching law (ERL)

**Computational cost:** Variable — STA itself is moderate; observers and optimization add overhead.

### 5. Fractional-Order SMC (FOSMC)

Uses Caputo fractional derivatives in sliding surface design. The fractional operator provides a "memory" effect — the surface dynamics depend on the full trajectory history, not just current state. This enables finer transient shaping.

**Variants cataloged:**
- FOSMC-PID hybrid — fractional surface combined with PID structure
- RL-optimized FOSMC — reinforcement learning for fractional order and gain tuning
- Super-twisting FOSMC — STA reaching law with fractional surface
- Fuzzy-exponential convergence FOSMC — fuzzy logic accelerates convergence
- Multi-motor synchronization FOSMC — coordinated speed control of coupled PMSMs

**Strengths:** Best transient response shaping of all SMC families; additional tuning freedom from fractional order parameter.
**Weakness:** Extremely high computational cost — fractional derivative approximation (Grünwald-Letnikov or Oustaloup) requires storing trajectory history and performing convolution sums. Impractical for resource-constrained embedded targets without approximation.

**Computational cost:** Highest — prohibitive for low-cost MCUs.

### 6. Adaptive SMC (ASMC)

Online gain adjustment removes the need for a priori knowledge of disturbance upper bounds. The adaptive law increases switching gain until reachability is satisfied, then stabilizes.

**Variants cataloged:**
- Adaptive TSMC — terminal surface with adaptive switching
- Adaptive NTSMC — nonsingular terminal with adaptive gain
- Observer-assisted ASMC with DOB, reduced-order PI observer, ESO
- Event-triggered ASMC — reduces computation by updating control only when a triggering condition is met

**Strengths:** No conservative gain overestimation; self-tuning under varying conditions.
**Weakness:** Potential for slow adaptation if learning rate is conservative; gain drift risk without barrier-function constraints.

**Computational cost:** Moderate — adaptive law evaluation is light; observer adds bulk.

---

## Cross-Cutting Techniques

### Observer Integration

Over **90% of surveyed methods** incorporate some form of disturbance/state observer:

| Observer Type | Abbreviation | Usage |
|---|---|---|
| Extended State Observer | ESO | Most common; estimates total disturbance as augmented state |
| Disturbance Observer | DOB | Estimates lumped disturbance from measurable states |
| Sliding-Mode Observer | SMO | Uses SMC structure for robust state estimation |
| Finite-Time Disturbance Observer | FTDO | Converges in bounded time |
| Nonlinear Disturbance Observer | NDO | Handles nonlinear disturbance dynamics |
| High-Order Sliding-Mode Observer | HOSTO | Multi-derivative estimation |
| Support Vector Regression Observer | SVR | ML-based disturbance estimation |

The observer reduces the required switching gain from K > D_max to K > |D − D̂| (estimation error), directly reducing chattering.

### Intelligent Methods

- **Fuzzy logic:** Tunes switching gain based on sliding variable magnitude and rate; ~15% of studies
- **Neural networks:** RBFNN, ELM, recurrent NN approximate lumped disturbances online; replace conservative switching terms
- **Reinforcement learning:** Emerging — used for FOSMC parameter optimization; <5% of studies

AI-augmented SMCs report **30–40% improvement** in tracking accuracy and chattering reduction over non-intelligent counterparts.

### Optimization-Based Tuning

**~22% of studies** integrate offline metaheuristic optimization:

| Algorithm | Usage |
|---|---|
| PSO (Particle Swarm Optimization) | Most common — sliding surface and reaching law parameters |
| GA (Genetic Algorithm) | ESO gains, controller parameters |
| HHO (Harris Hawks Optimization) | Multi-objective SMC tuning |
| BAS (Beetle Antennae Search) | STA gain optimization |
| DBO (Dung Beetle Optimizer) | HOSTSM parameter tuning |
| GWO (Grey Wolf Optimizer) | Multi-parameter SMC tuning |

---

## Comparative Analysis

### Performance–Complexity Tradeoff (from Table 2)

| SMC Family | Chattering | Convergence | Robustness | Computation | Best For |
|---|---|---|---|---|---|
| **CSMC** | High (mitigated by boundary layer) | Asymptotic | Good (matched disturbances) | Lowest | Simple drives, prototyping |
| **TSMC** | Moderate | Finite-time | Strong | Moderate | Precision tracking, time-critical |
| **ISMC** | Low (no reaching phase) | Asymptotic | Invariant from t=0 | Low-moderate | Disturbance-sensitive applications |
| **HOSMC** | Very low (STA: continuous u) | Finite-time | Strong | Variable | Production drives needing smooth torque |
| **FOSMC** | Low | Fractional-order | Strong + memory effect | Very high | Research; high-performance with compute budget |
| **ASMC** | Low (adaptive gain) | Varies | Self-tuning | Moderate | Variable-load applications |

### Research Gap Assessment (from Section 6.5)

1. **Limited experimental validation** — fewer than 15% of surveyed papers include real hardware experiments; most are simulation-only (MATLAB/Simulink)
2. **Energy efficiency neglected** — almost no studies consider power consumption or efficiency of the SMC algorithm itself
3. **Unmatched disturbance handling** — most methods assume matched disturbances only; mismatched disturbances (entering through different channels than control input) are underaddressed
4. **No standardized benchmarks** — no agreed-upon test scenarios, motor parameters, or performance metrics for comparing SMC methods

### Future Directions Identified

1. **Intelligent self-learning SMC** — RL-based or meta-learning approaches that adapt structure, not just gains
2. **Fractional-adaptive hybrids** — combining FOSMC's transient shaping with ASMC's self-tuning (computationally challenging)
3. **Energy-aware and fault-tolerant SMC** — joint optimization of control performance and power consumption; graceful degradation under faults
4. **Hardware-in-the-loop (HIL) validation** — bridge the simulation-to-hardware gap
5. **Benchmark standardization** — community-accepted test scenarios for fair comparison
6. **Cross-domain integration** — SMC for multi-physics systems (thermal + electrical + mechanical)

---

## Research Evolution (2020–2025)

Three dominant research axes identified:

1. **Order augmentation** — CSMC → HOSMC (higher-order surfaces push discontinuity deeper) → FOSMC (fractional calculus adds memory)
2. **Structural augmentation** — standalone SMC → observer-enhanced → MPC-hybrid → fault-tolerant
3. **Intelligence infusion** — fixed gains → optimization-tuned → fuzzy/NN-augmented → RL-adapted

The trajectory moves from pure theory toward practical, intelligent, fault-resilient controllers. **68% of 2020–2025 papers incorporate observers; 22% integrate optimization or ML.**

---

## Relevance to Stepper Motor Control

This survey's PMSM speed control focus maps directly to FOC-driven stepper motors:

- **PMSM ≈ stepper under FOC:** The d-q frame model and torque equation T_e = (3/2)p_n·ψ_f·i_q apply identically to stepper motors driven by [[TMC4671-LA]] or firmware FOC (e.g., [[k4671]])
- **Speed loop SMC:** The survey's speed controllers sit in the same architectural position as the PID velocity loop in the TMC4671 cascade — SMC could replace or augment the PID speed regulator
- **STA recommended for embedded:** The super-twisting algorithm produces continuous control output (no chattering in torque command), moderate computation, and strong robustness — well-suited for MCU implementation
- **Observer + SMC practical pattern:** The 90%+ observer usage rate confirms that standalone SMC is insufficient in practice; the [[k4671]] firmware's software feedforward serves a similar role to disturbance observers
- **Computational constraints matter:** FOSMC's high cost makes it impractical for typical stepper driver MCUs; CSMC, ISMC, and STA are the most implementable families
- **Experimental validation gap:** The <15% experimental rate in the literature means published SMC results may not transfer to real stepper hardware without significant tuning — caution warranted when adopting published algorithms
- **Cogging torque as matched disturbance:** Stepper cogging torque (especially at low speed) maps to the lumped disturbance D(t) in the PMSM model; DOB-based SMC should handle it effectively

### Comparison with Yu et al. 2023 (Existing [[Sliding-Mode-Control]] Page)

| Aspect | Yu et al. 2023 | Badamasi et al. 2025 |
|---|---|---|
| Motor type | PMLSM (linear) | PMSM (rotary) |
| Control objective | Position tracking | Speed regulation |
| Coverage period | Historical through 2022 | 2020–2025 (systematic PRISMA) |
| Depth | Deep theory + implementation details | Broad taxonomy + research trends |
| Variants covered | Same families | Same families + FOSMC, fault-tolerant |
| Unique value | Practical design procedure, hardware examples | Computational cost analysis, research gaps, 66-paper catalog |

Both sources are complementary. The existing page provides the theory and practical design guidance; this survey provides the research landscape and trend analysis.

---

## Key References

Selected from 220+ citations in the survey:

- **Utkin (1977):** Original variable-structure control with sliding modes — foundational work
- **Levant (1993, 2003):** Higher-order sliding modes, super-twisting algorithm — HOSMC foundation
- **Podlubny (1999), Monje et al. (2010):** Fractional-order calculus and control — FOSMC mathematical basis
- **Badamasi & Nasir (2025):** This survey — arXiv:2510.18420v1
- **Yu et al. (2023):** PMLSM position control SMC review — *Actuators* 12(1), 31 — basis of [[Sliding-Mode-Control]]
