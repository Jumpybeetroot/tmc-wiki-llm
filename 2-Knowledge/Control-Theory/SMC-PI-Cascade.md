# SMC-PI Cascaded Position Control for Elastic Feed Drives

**Type:** knowledge
**Summary:** Neubauer et al. — practical SMC position controller replacing the P controller in industrial P-PI cascade for ball screw feed drives. Single tuning parameter λ, Kalman-estimated acceleration, PT2·I plant model. Experimental validation: 76% tracking error reduction, 58% disturbance rejection improvement vs P-PI.
**Tags:** #control-theory #sliding-mode #feed-drives #cascade-control #motion #ball-screw
**Status:** draft
**Updated:** 2026-04-09
**Source:** Neubauer, M.; Brenner, F.; Hinze, C.; Verl, A. "Cascaded sliding mode position control (SMC-PI) for an improved dynamic behavior of elastic feed drives." *Int. J. Machine Tools and Manufacture*, 2024. DFG Project No. 438835664.
**Related:** [[Sliding-Mode-Control]], [[SMC-PMSM-Survey]], [[PID-Tuning-Methods]]

---

## Overview

Industrial feed drives (ball screw, rack-and-pinion) universally use P-PI cascade control: a P position controller wrapping a PI speed loop wrapping a current loop. The maximum bandwidth (Kv factor) is strictly limited by structural compliance — the first mechanical eigenmode of the coupled motor-screw-table system.

This paper replaces **only the P position controller** with a continuous sliding mode controller while keeping the PI speed loop and current loop intact. The novelty is practical applicability:

1. **One tuning parameter** (λ) — comparable commissioning effort to tuning Kv
2. **No chattering** — continuous reaching law, no discontinuous sign function
3. **No compensation filters** — the SMC inherently handles the compliant mechanics
4. **Generic plant model** — PT2·I captures any compliant single-axis feed drive, not just ball screws

## Plant Model (PT2·I)

The closed-loop velocity plant (speed loop already closed) combined with mechanical compliance is approximated as a second-order oscillatory element (PT2) followed by an integrator (I):

```
G_pos(s) = ω₀² / [s · (s² + 2Dω₀s + ω₀²)]
```

**Only two parameters required:**
- ω₀ — characteristic angular frequency of the dominant elastic mode
- D — damping ratio

These are typically already known from commissioning the velocity controller (frequency response measurements).

**Experimental values (Stuttgart test bench):**
- ω₀ = 277.8 rad/s (~44.2 Hz)
- D = 0.518
- Speed loop: Kp,n = 175 1/s, Ki,n = 50 1/s

## Controller Design

### Structure

```
[Setpoint] → [SMC Position Controller] → [PI Speed Loop] → [Current Loop] → [Motor]
                    ↑                            ↑
              Position + velocity          Motor encoder
              + acceleration (KF)
```

The SMC replaces the P controller. Inner PI speed loop and current loop are unchanged.

### Sliding Surface

Linear sliding surface built from position error and its derivatives:

```
S = Σ αᵢ · e⁽ⁱ⁾    (i = 0 to n-1)
```

Coefficients depend on the tuning parameter λ.

### Control Law

The control law (paper eq. 22, applied to PT2·I) consists of three terms:

1. **Compensation term** — inverts the modeled PT2 dynamics using ω₀ and D (requires position, velocity, acceleration feedback)
2. **Feedforward term** — jerk setpoint (third derivative of position command)
3. **Feedback term** — sliding surface error term, parameterized by K

Continuous reaching law: Ṡ = -KS (exponential convergence, no chattering).

### Single Tuning Parameter

The SMC design produces two degrees of freedom: λ (sliding surface shape) and K (reaching speed). Error dynamics analysis for the PT2·I shows closed-loop poles at:

- s₁,₂ = λ (double pole from PT2)
- s₃ = K

Setting **K ≡ λ** collapses both into a single tuning parameter. The double real pole guarantees **aperiodic (non-oscillatory) error dynamics** for all λ > 0.

**Recommended range:** λ ∈ [200, 300] 1/s (test bench dependent). At λ ≥ 400, unmodeled higher-order dynamics caused instability.

### Kalman Filter for Acceleration Estimation

The SMC control law requires acceleration feedback. A Kalman filter based on the PT2·I model provides this without an additional sensor:

- **States estimated:** position, velocity, acceleration (3 states)
- **Process noise Q:** 10⁻⁶ mm² · diag{1, 1/s², 100/s⁴}
- **Measurement noise R:** 10⁻² mm² · diag{1, 10/s²}
- R determined offline from measurement system noise; Q tuned empirically

## Stability and Robustness

### Theoretical

- Lyapunov-based design: continuous reaching law Ṡ = -KS ensures globally exponential stability
- Aperiodic error dynamics (poles at λ, always real) — inherently overshoot-free for any λ > 0
- Proven for the general controllable canonical form (applicable beyond PT2·I)

### Robustness to Model Uncertainties

Parameter variation analysis (±20% on ω₀ and D):

- Controller remains stable for moderate overestimation of ω₀ (up to ~3×ω₀)
- Practically advisable: λ > 0.5·ω₀ to maintain damping
- Experimental gain margin kr > 8 dB, phase margin φr > 60° across wide λ range
- Stability margins remain **almost independent of λ** — unlike P-PI where increasing Kv degrades margins

## Experimental Results

### Test Bench

- Industrial ball screw feed drive (Siemens 1FT7085 servo motor, Steinmeyer 3526/40 ball screw)
- Table mass: 400 kg, pitch: 40 mm, max velocity: 2000 mm/s
- First axial eigenmode: 51-66 Hz (position/load dependent)
- dSPACE rapid prototyping at 4 kHz; ~1 ms communication dead time to inverter

### Command Tracking (Ramp Trajectory)

| Controller | J₁(e) [mm] | std(e) [mm] | max(e) [mm] | f₀dB [Hz] | kr [dB] |
|---|---|---|---|---|---|
| P-PI (Kv=50) | 0.0986 | 0.1245 | 0.2426 | 9.70 | 10.19 |
| SMC-PI (λ=200) | 0.0722 | 0.1024 | 0.2225 | 11.06 | 11.40 |
| SMC-PI (λ=300) | 0.0174 | 0.0244 | 0.0585 | 16.40 | 10.54 |

### Disturbance Rejection (500 N Step)

| Controller | Max position error |
|---|---|
| P-PI (Kv=50) | 32.1 μm |
| SMC-PI (λ=200) | 20.6 μm |
| SMC-PI (λ=300) | 13.5 μm |

### HSC Milling Simulation

| Controller | J₁(e) [mm] | std(e) [mm] | max(e) [mm] |
|---|---|---|---|
| P-PI (Kv=50) | 0.0394 | 0.0742 | 0.2496 |
| SMC-PI (λ=200) | 0.0194 | 0.0409 | 0.1438 |
| SMC-PI (λ=300) | 0.0036 | 0.0056 | 0.0211 |

### Summary Improvements (λ=300 vs P-PI)

- **Tracking error (ramp):** max reduced by 75.9%
- **Disturbance rejection:** max reduced by 57.9%
- **HSC milling:** tracking error reduced by ~82%
- **Bandwidth:** f₀dB increased from 9.70 to 16.40 Hz (69% increase)
- **Stability margins:** comparable to P-PI (gain margin ~10.5 dB maintained)

## Limitations

1. **Requires direct load-side measurement** — table-mounted linear encoder or similar. Motor encoder alone is insufficient (table position differs due to compliance).
2. **PT2·I is a first-mode approximation** — higher-order dynamics act as disturbances. Excessive λ excites unmodeled modes.
3. **Kalman filter tuning** — incorrect Q/R degrades acceleration estimate.
4. **Communication latency** — test setup had ~1 ms dead time from dSPACE; production implementation in inverter would reduce this.

## Stepper Motor Relevance

This paper is directly applicable to stepper motor systems with compliance:

- **Belt-driven axes** — exhibit similar PT2·I dynamics with lower ω₀ than ball screws
- **TMC4671-based drives** — the cascaded control structure (current → velocity → position) maps directly to SMC-PI replacement of the outer P loop
- **Single tuning parameter** — practical for commissioning embedded motor controllers where complex tuning is infeasible
- **Non-oscillatory guarantee** — prevents position overshoot that causes missed steps
- The key prerequisite is load-side position feedback (encoder on the driven axis, not just motor encoder)
