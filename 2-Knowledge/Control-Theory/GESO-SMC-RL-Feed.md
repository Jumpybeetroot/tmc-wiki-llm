# GESO-Based Sliding Mode Control with Reinforcement Learning for Servo Feed Systems

**Type:** knowledge
**Summary:** Wang et al. 2024 — SMC with generalized extended state observer (GESO, n+2 order) for matched + mismatched disturbance estimation, plus DCDDPG reinforcement learning for observer gain tuning. Applied to a two-axis differential micro-feed system (TDMS) that avoids low-speed crawling. Experimental: 2.1 μm avg tracking error vs PID+FC 6.8 μm (69% improvement); friction observation error reduced 82.6%.
**Tags:** #control-theory #sliding-mode #feed-drives #disturbance-observer #reinforcement-learning #friction
**Status:** draft
**Updated:** 2026-04-09
**Source:** Wang, A.; Feng, X.; Liu, H.; Yao, M. "Design of sliding mode controller for servo feed system based on generalized extended state observer with reinforcement learning." *Scientific Reports* 14, 24233, 2024. DOI: 10.1038/s41598-024-75598-5. Shandong University, NSFC Grant 51875325.
**Related:** [[Sliding-Mode-Control]], [[SMC-PMSM-Survey]], [[SMC-PI-Cascade]], [[PID-Tuning-Methods]]

---

## Overview

High-precision servo feed systems suffer from nonlinear friction (Stribeck effect at low speed), system uncertainty, and external disturbances. This paper addresses all three through a composite controller:

1. **Two-axis differential micro-feed system (TDMS)** — mechanical solution to low-speed crawling
2. **Generalized extended state observer (GESO)** — n+2 order observer for matched + mismatched disturbances
3. **Integral sliding mode controller** — robust control with Lyapunov-proven stability
4. **DCDDPG reinforcement learning** — automatic tuning of critical observer gains
5. **Quadratic optimal control** — reduces manual tuning of controller parameters

The key novelty is the combination: GESO handles both disturbance channels (where traditional ESO handles only one), and RL automates the most sensitive tuning step.

## Two-Axis Differential Micro-Feed System (TDMS)

### Mechanism

Traditional ball screw feeds drive the screw with a motor while the nut translates with the table. At low speeds, the motor enters the Stribeck friction region, causing crawling.

TDMS adds a second servo motor that independently drives the nut via synchronous belt (1:1 ratio). Both shafts rotate in the same direction at speeds well above the crawling threshold. The table velocity equals the speed **difference** between screw and nut shafts:

```
v_table = v_screw - v_nut
```

**Example:** For 3 mm/s table feed at sinusoidal profile:
- Screw shaft: v_rs = 5 + 3·sin(0.4πt) mm/s (always ≥ 2 mm/s)
- Nut shaft: v_rn = -5 mm/s (constant, well above crawling speed)
- Table: v = 3·sin(0.4πt) mm/s (includes zero-crossing, but neither shaft crawls)

### Advantage

Both drive shafts operate above the Stribeck velocity, so nonlinear friction exists only at the workbench guide rail (where it is much smaller than at the drive shafts). This transforms a nonlinear friction problem into a nearly linear one.

## Plant Model — Flexible Two-Mass Drive Model (FTMDM)

The system is modeled as a flexible two-mass system (not a rigid body), accounting for shaft compliance:

```
Screw drive shaft (Eqs. 2-3):
  m_s1·ẍ_s1 = u_s - b_s1·ẋ_s1 - C_s·(ẋ_s1 - ẋ_s2) - k_s·(x_s1 - x_s2) + f_s1
  m_s2·ẍ_s2 = C_s·(ẋ_s1 - ẋ_s2) + k_s·(x_s1 - x_s2) - b_s2·ẋ_s2 + f_s2
```

Where:
- m_s1, m_s2 — equivalent masses of rotating and linear displacement elements
- k_s, C_s — equivalent stiffness and damping of the screw drive shaft
- b_s1, b_s2 — viscous damping coefficients
- f_s1 — **matched disturbance** (in control channel): motor nonlinear friction, parameter uncertainty
- f_s2 — **mismatched disturbance** (not in control channel): guide rail friction, external forces

State vector: x_s = [x_s1, x_s2, ẋ_s1, ẋ_s2]ᵀ, with state-space form ẋ = Ax + bu + Dh.

The nut drive shaft has an identical model structure with different parameters.

### Experimental Parameters

| Parameter | Screw shaft | Nut shaft | Unit |
|---|---|---|---|
| Rotating element mass m₁ | 0.00048 | 0.0032 | V·s²/rad |
| Linear element mass m₂ | 0.00191 | 0.00687 | V·s²/rad |
| Equivalent stiffness K | 3120 | 5653 | V/rad |
| Equivalent damping C | 0.018 | 0.033 | V·s/rad |
| Motor viscous damping b₁ | 0.00342 | 0.0045 | V·s/rad |
| Nut/guide rail damping b₂ | 0.0298 | — | V·s/rad |

## Generalized Extended State Observer (GESO)

### Design Motivation

Traditional extended state observers (ESO) add one augmented state to estimate lumped disturbance. This works for systems where all disturbances enter through the control channel (**matched**). In FTMDM, disturbances enter through **two channels** — f_s1 (matched, motor side) and f_s2 (mismatched, guide rail side). A standard ESO cannot estimate both simultaneously.

### GESO Structure

GESO extends the state vector by **two** augmented states (n+2 order instead of n+1):

```
Extended state: x_se = [x_s1, x_s2, ẋ_s1, ẋ_s2, f_s1, f_s2]ᵀ
```

Observer form:
```
ż = A_e·z + b·u + L·(y - ŷ)
ŷ = C_e·z
```

Where z = [z₁, z₂, z₃, z₄, z₅, z₆]ᵀ are the observed values, and disturbances are estimated through z₅ (matched) and z₆ (mismatched).

Gain matrix L is 6×2 (12 parameters total):
```
L = [l₁₁ l₁₂; l₂₁ l₂₂; l₃₁ l₃₂; l₄₁ l₄₂; l₅₁ l₅₂; l₆₁ l₆₂]ᵀ
```

### Convergence Proof (Lemma 1)

**Claim:** If L is chosen such that A_o = A_e - L·C_e is Hurwitz, then the observation error e_o is bounded for any bounded f_s derivative.

**Proof sketch:** Define Lyapunov function V₁ = eᵀ·J·e where J satisfies A_oᵀ·J + J·A_o = -Q (Q positive definite). Taking V̇₁ and analyzing cases:

- When ‖e_o‖² > 2‖J·d‖², then V̇₁ < 0 (error decreasing)
- When ‖e_o‖² < 2‖J·d‖², then V̇₁ > 0 (error bounded by 2‖J·d‖²)

In practice, the sampling frequency is much higher than disturbance bandwidth, so ḟ_s ≈ 0 within each control period, giving d = 0 and asymptotic stability.

## Sliding Mode Controller

### Integral Sliding Surface

```
σ = K·e_s + P·∫e_s dt
```

Where:
- e_s = x_r - x_s (tracking error vector)
- K = [k₁, k₂, k₃, k₄] — proportional coefficients (response speed)
- P = [p₁, p₂, p₃, p₄] — integral coefficients (steady-state error)

The integral term eliminates the reaching phase — the trajectory starts on the sliding manifold.

### Control Law

Using constant-velocity reaching law with saturation function for chattering suppression:

```
u_s = [control terms from σ̇ = 0] - η·sat(σ/Δ)
```

Where sat(σ/Δ) replaces sign(σ):
```
sat(σ) = sign(σ)     if |σ| ≥ Δ
sat(σ) = σ/Δ         if |σ| < Δ
```

### Stability (Theorem 1)

By selecting appropriate η and Δ, the system reaches the sliding surface in finite time and is asymptotically stable. Proven via Lyapunov V = ½σ² with case analysis for σ > Δ, σ < -Δ, and |σ| ≤ Δ. The key requirement: η must satisfy K·B₁·η ≥ ε = sup(|K·B₂·e_f|), where e_f is the bounded GESO estimation error.

### Error Convergence (Theorem 2)

After reaching the sliding surface (σ = σ̇ = 0), the tracking error dynamics become:

```
ė_s = -Q·e_s    where Q = P ⊘ K (element-wise division)
```

All elements of Q are positive, so e_s → 0 as t → ∞. The convergence time depends on K and P values.

## Parameter Tuning via Quadratic Optimal Control

The integral coefficient P is determined from the proportional coefficient K using LQR-like optimization:

```
Minimize J = ∫₀^∞ (eᵀ·W·e + u²) dt
```

The weighting matrix W includes vibration suppression terms (q₅, q₆ = 1000 weight the axial vibration x_s1 - x_s2). Solving the associated Riccati equation yields the optimal feedback gain M, from which P = M·K.

**Result:** Only K needs manual tuning; P is derived automatically. This reduces the tuning problem from 8 parameters (K₁₋₄ + P₁₋₄) to 4 (K₁₋₄).

### Final Experimental Parameters

```
K = [0.5, 2, 15, 30]
P = [7.97×10⁵, 7.97×10⁵, 887, 1261]
Δ = 0.5
η = 20
```

## DCDDPG Reinforcement Learning for GESO Tuning

### Motivation

The GESO gain matrix L has 12 parameters, making manual tuning impractical. Engineering tuning revealed that most parameters primarily affect convergence speed, while **l₅₂ and l₆₂ have the dominant effect on observation error magnitude**. The RL agent tunes only these two critical gains.

### Agent Architecture (Double Critic DDPG)

Standard DDPG suffers from Q-value overestimation. DCDDPG adds a second independent critic network; the minimum of the two Q-values is used for updates.

**Actor network:**
- Input: 3 (expected output x_r2, linear displacement error e₁, rotational displacement error e₂)
- Hidden: 32 → 16 (tanh activation)
- Output: 1 (continuous action for l₅₂ or l₆₂)

**Critic network (×2):**
- State input: 32 nodes; Action input: 32 nodes
- Merged hidden: 64 → 64 (ReLU activation)
- Output: 1 (Q-value)

### Reward Function

```
r = kr₁ · e_f + c    (kr₁ < 0)
```

Where e_f = |e_f1| + |e_f2| + |e₁| + |e₂| (sum of observation errors). Stepwise increasing rewards (c = c₁, c₂, c₃ for decreasing error thresholds) encourage the agent to explore smaller error regions faster.

### Training Parameters

| Parameter | Value |
|---|---|
| Replay buffer size | 50,000 |
| Batch size | 256 |
| Soft update factor τ | 0.001 |
| Critic learning rate | 0.001 |
| Actor learning rate | 0.0001 |
| Episodes | 1,000 |
| Steps per episode | 1,000 |

## Results

### Simulation — Observer Performance

**Sinusoidal disturbance observation:**

| Observer | Max matched error (N·m) | Max mismatched error (N·m) |
|---|---|---|
| GESO (no RL) | 0.022 | — |
| DQN_GESO | 0.012 | — |
| DCDDPG_GESO | 0.007 | ±0.0075 (68.2% better than GESO) |

DQN discretizes actions, causing periodic error fluctuations. DCDDPG outputs continuous actions, achieving smoother and smaller errors.

**Step disturbance observation:**

| Observer | Steady-state matched error (N·m) | Mismatched error improvement |
|---|---|---|
| GESO | -0.018 | baseline |
| DQN_GESO | 0.008 | — |
| DCDDPG_GESO | 0.001 | 94.4% improvement vs GESO |

**Nonlinear friction observation (LuGre model):**

- GESO error during Stribeck phase: 0.023 N·m
- DQN_GESO: 0.011 N·m
- DCDDPG_GESO: 0.004 N·m (**82.6% improvement** vs GESO)
- Post-Stribeck DCDDPG_GESO error: 1.6×10⁻⁵ N·m

**Simulation tracking (sinusoidal, workbench):**

| Controller | Max error (μm) |
|---|---|
| PID + FC | 13 |
| AFCC | ±5.5 |
| RLOSMC | ±0.9 |

### Experimental Results (TDMS Hardware)

Test bench: TDMS with incremental grating ruler (10 nm resolution). Sinusoidal trajectory v = 3·sin(0.4πt) mm/s.

**Quasi-static stability (v_table = 0):**

| Controller | Position error range (μm) |
|---|---|
| PID + FC | ±3 (fluctuating) |
| AFCC | 1.3–1.5 |
| RLOSMC | 0.8–1.0 |

**Position tracking (Table 6):**

| Controller | Max error (μm) | Avg error (μm) | Std dev (μm) |
|---|---|---|---|
| PID + FC | 11.1 | 6.8 | 3.1 |
| AFCC | 7.1 | 3.6 | 1.4 |
| RLOSMC | 4.5 | 2.1 | 1.6 |

**Tracking accuracy improvement:** 69.1% vs PID+FC (average error).

**Speed combination comparison (all RLOSMC, Table 8):**

| Combination | Description | Max (μm) | Avg (μm) |
|---|---|---|---|
| 1 | Screw only (traditional) | 6.0 | 2.8 |
| 2 | Nut only (traditional) | 5.6 | 3.4 |
| 3 | Dual sine (both reverse) | 7.4 | 4.1 |
| 4 | TDMS optimal (no reversal) | 4.5 | 2.1 |

Combination 4 (TDMS operating mode) is best because neither shaft reverses, eliminating nonlinear friction from the drive shafts entirely.

## Limitations

1. **DCDDPG training offline** — the RL agent is trained in simulation, then the optimized gains are deployed. Not online adaptive during operation.
2. **LuGre friction model dependency** — simulation validation uses LuGre; real friction may differ. However, GESO treats friction as unknown disturbance, so model accuracy is less critical than for feedforward compensation.
3. **TDMS-specific** — the dual-drive mechanism adds hardware complexity (second motor, synchronous belt, bearings). Not applicable to single-motor systems without modification.
4. **Gain matrix pre-selection** — 10 of 12 GESO gains are still tuned by engineering methods; only l₅₂ and l₆₂ are RL-optimized.
5. **No comparison with other SMC variants** — compared only against PID+FC and AFCC, not against other SMC approaches (STA, NTSMC, etc.).

## Stepper Motor Relevance

This paper's contributions map to stepper motor and CNC applications:

- **Matched + mismatched disturbance handling** — stepper motors under FOC experience cogging torque (matched, motor side) and external cutting/friction forces (mismatched, load side). GESO's dual-channel estimation addresses both, unlike standard ESO used in typical stepper FOC implementations.
- **Friction-as-disturbance paradigm** — avoids the need for friction model identification (LuGre parameters), which is impractical for production stepper driver commissioning. The observer estimates and compensates friction without knowing its model.
- **TDMS concept for precision motion** — relevant to dual-drive configurations in CNC where lead screw and nut can be independently driven. The anti-crawling benefit applies to any application requiring smooth low-speed motion (scanning, probing, fine positioning).
- **RL for observer tuning** — the DCDDPG approach to gain tuning could be applied to autotuning disturbance observers in embedded stepper controllers, though the training phase requires a simulation environment or offline calibration.
- **Integral sliding surface** — eliminates reaching phase, providing immediate invariance. This property is valuable for stepper position control where initial transient errors could cause missed steps.
