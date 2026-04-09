# Direct Synthesis PID Tuning for Time-Delayed Systems

**Type:** knowledge
**Summary:** Kula's root-locus-based direct synthesis method for PI/PID tuning of FOPDT and SOPDT plants -- achieves exact non-oscillatory (critically damped) response without Taylor/Pade approximation errors that limit conventional DS methods.
**Tags:** #control-theory #pid #tuning #direct-synthesis #time-delay #motion
**Status:** draft
**Updated:** 2026-04-09
**Source:** Kula, K. S. "Tuning a PI/PID Controller with Direct Synthesis to Obtain a Non-Oscillatory Response of Time-Delayed Systems." *Applied Sciences* 14(13), 5468, 2024. https://doi.org/10.3390/app14135468
**Related:** [[PID-Tuning-Methods]], [[SIMC-PID-Tuning]], [[Feedforward-Compensators]]

---

## Overview

Direct synthesis (DS) derives controller settings analytically from a plant model and a desired closed-loop transfer function. For a plant G(s) and desired closed-loop H(s):

```
C(s) = H(s) / [G(s) * (1 - H(s))]
```

The fundamental problem with DS for time-delayed plants is that the delay term e^(-theta*s) produces a transcendental equation. All prior DS methods (Dahlin, Smith-Corripio, Chen-Seborg, SIMC) resolve this by approximating the delay:

| Method | Approximation | Limitation |
|---|---|---|
| Dahlin (1968) | Taylor: e^(-theta*s) ~ 1 - theta*s | First-order only, inaccurate for large theta/tau |
| Chen-Seborg (2002) | Taylor at desired-TF stage | Same |
| Kumar-Singh, Panda et al. | Taylor to 2nd order + Maclaurin | Better but still approximate |
| Abbas, Ajmeri-Ali | Pade: (1-0.5*theta*s)/(1+0.5*theta*s) | More accurate than Taylor but still approximate |
| SIMC (Skogestad 2003) | Taylor: e^(-theta*s) ~ 1 - theta*s | Compensated via improved rule (tau1 + theta/3) |

**Kula's contribution:** An exact method that avoids all approximations by using root-locus analysis to find the proportional gain Kp that places the closed-loop poles for critical damping (non-oscillatory, minimum settling time).

## The Problem with Approximations

When DS methods approximate the delay, the resulting controller settings do not exactly achieve the desired closed-loop response. The error grows with the ratio theta/tau (delay-to-time-constant ratio). For plants with significant delay, the designed "non-oscillatory" response may actually oscillate, or the settling time may differ substantially from the target.

**Key insight:** The desired closed-loop for a time-delayed plant is H(s) = e^(-theta*s) / (1 + tau_c*s). The delay term is inherent to the plant and cannot be removed by the controller -- it must appear in the closed-loop response. The difficulty is that substituting this into the DS formula produces a controller transfer function containing e^(-theta*s) in the denominator, which is not directly realizable.

## Kula's Root-Locus Method

Instead of approximating the delay, Kula's method works with the exact characteristic equation of the closed-loop system.

### For FOPDT Plants

Plant model: G(s) = K * e^(-theta*s) / (1 + T*s)

With a PI controller (Ti = T for pole-zero cancellation):

```
C(s) = Kp * (1 + 1/(T*s))
```

The closed-loop characteristic equation becomes:

```
1 + K*Kp * e^(-theta*s) / (T*s) = 0
```

Or equivalently:

```
T*s + K*Kp * e^(-theta*s) = 0
```

This is a transcendental equation with infinitely many roots. Kula analyzes the root locus as Kp varies:

1. **At Kp = 0:** One root at s = 0, plus infinitely many roots at s = -inf (from the delay).
2. **As Kp increases:** Roots move along branches. The dominant pair eventually becomes complex (oscillatory).
3. **Critical damping point:** The value of Kp where the dominant roots transition from real to complex. This is the maximum Kp for non-oscillatory response.

### Finding the Critical Gain

The breakaway point on the root locus satisfies both:

```
T*s + K*Kp * e^(-theta*s) = 0      (characteristic equation)
T - K*Kp*theta * e^(-theta*s) = 0   (derivative = 0)
```

From these two equations, eliminating Kp:

```
s = -1/theta
```

The breakaway always occurs at s = -1/theta. Substituting back:

```
Kp = T / (K * theta * e)     where e = 2.718...
```

Or equivalently:

```
Kp = T / (K * theta * exp(1))
```

This gives the **exact** proportional gain for critical damping -- no approximation required.

### Settling Time

With the dominant pole at s = -1/theta, the closed-loop time constant is tau_c = theta. The 2% settling time is:

```
ts = 3.91 * theta + theta = 4.91 * theta
```

(3.91 time constants for the exponential decay, plus one dead time).

### Comparison with Approximation-Based Methods

For the same FOPDT plant, the various DS methods give different Kp:

| Method | Kp formula | tau_c achieved |
|---|---|---|
| Dahlin / Chen-Seborg (Taylor) | T / [K*(tau_c + theta)] | Approximate |
| Abbas (Pade) | (T + theta/2) / [K*(tau_c + theta)] | Approximate |
| Kula (exact) | T / (K*theta*e) | Exact: tau_c = theta |

**At tau_c = theta (the SIMC default):**
- Taylor-based: Kp = T / (2*K*theta) = 0.500 * T/(K*theta)
- Pade-based: Kp = (T + theta/2) / (2*K*theta)
- Kula exact: Kp = T / (K*theta*e) = 0.368 * T/(K*theta)

The Taylor-based methods overestimate the allowable gain by ~36%, which can produce overshoot in the actual time-delayed system. Kula's gain is more conservative but guarantees non-oscillatory response.

## Extension to SOPDT Plants

For second-order plus dead-time plants:

```
G(s) = K * e^(-theta*s) / [(1 + T1*s)(1 + T2*s)]
```

With a PID controller (Ti = T1, Td = T2 for pole-zero cancellation):

```
Kp = T1 / (K * theta * e)
```

The same breakaway analysis applies. The dominant pole location and critical gain formula are structurally identical -- the PID zeros cancel the plant poles, reducing the effective problem to the same FOPDT form.

## Practical Implications

### Advantages

1. **Exact result:** No approximation errors -- the designed response is truly non-oscillatory.
2. **Simple formula:** Kp = T/(K*theta*e) is even simpler than SIMC.
3. **No tuning parameter:** Unlike SIMC (which has tau_c), this method has a single unique solution for critical damping.
4. **Overcomes delay limitations:** Other DS methods have maximum permissible delay ratios; this method works for any theta/tau.

### Limitations

1. **Fixed performance:** tau_c = theta is not adjustable -- you get critical damping or nothing. SIMC's adjustable tau_c offers a performance-robustness tradeoff dial.
2. **Setpoint tracking only:** The method targets setpoint response. Disturbance rejection performance is not explicitly optimized.
3. **Model accuracy required:** Like all DS methods, accuracy depends on the FOPDT/SOPDT model quality.
4. **Conservative:** The gain is lower than Taylor-based methods, giving longer settling times. In applications where some overshoot is acceptable, SIMC with tau_c = theta may be preferable.

### When to Use

- **Use Kula's method** when non-oscillatory response is a hard requirement (e.g., positioning axes where overshoot causes mechanical problems, fragile workpieces, or precision measurement settling).
- **Use SIMC** when you need the tau_c dial to trade off speed vs. robustness and can tolerate small overshoot.
- **Avoid both** when the plant model is poorly known -- use relay autotune instead.

## Stepper Motor Relevance

For stepper motor velocity loops with significant encoder latency or communication delay:

- The delay theta may dominate the loop dynamics (theta/tau > 0.5), exactly where Taylor approximation errors are largest.
- Non-oscillatory response prevents missed steps from velocity overshoot during acceleration profiles.
- The formula Kp = T/(K*theta*e) provides a direct, conservative starting point that guarantees stability without iteration.
- In TMC4671-based systems, the digital control loop introduces fixed-delay terms that map directly to the FOPDT dead-time parameter.

## Comparison with Other Methods in This Wiki

| Method | Page | Relationship |
|---|---|---|
| SIMC | [[SIMC-PID-Tuning]] | Also uses DS derivation; uses Taylor approximation + tau_c adjustment instead of exact root locus |
| General PID methods | [[PID-Tuning-Methods]] | DS is one of several tuning paradigms covered there |
| Feedforward | [[Feedforward-Compensators]] | Complementary: FF handles known disturbances/trajectories, DS tunes the feedback loop |

## Validation (from paper)

Kula tested against:

1. **Chen-Seborg method:** Kula's method achieves the exact desired response; Chen-Seborg shows deviation due to Taylor approximation.
2. **Abbas method (Pade):** Abbas gives closer but still approximate results; Kula is exact.
3. **CHR method (Chien-Hrones-Reswick):** Heuristic method with similar goals; Kula achieves faster settling for the same overshoot constraint.
4. **PSO-optimized PID:** Particle swarm optimization with ISE criterion. Kula's analytically derived settings match or exceed the PSO result without iterative search.
5. **Higher-order plants:** For plants not exactly FOPDT, the FOPDT approximation introduces the primary error -- not the DS derivation itself. Using an SOPDT model with PID recovers accuracy.

## References

- Kula, K. S. "Tuning a PI/PID Controller with Direct Synthesis to Obtain a Non-Oscillatory Response of Time-Delayed Systems." *Applied Sciences* 14(13), 5468, 2024.
- Dahlin, E. B. "Designing and Tuning Digital Controllers." *Instruments and Control Systems* 41(6), 77-83, 1968.
- Chen, D.; Seborg, D. E. "PI/PID Controller Design Based on Direct Synthesis and Disturbance Rejection." *Ind. Eng. Chem. Res.* 41(19), 4807-4822, 2002.
- Skogestad, S. "Simple analytic rules for model reduction and PID controller tuning." *J. Process Control* 13, 291-309, 2003.
- Chien, K. L.; Hrones, J. A.; Reswick, J. B. "On the Automatic Control of Generalized Passive Systems." *Trans. ASME* 74, 175-185, 1952.
