# IFC Ascent Guidance — Per-Aircraft Tuning Guide

This guide explains how to determine the nine aircraft-specific ascent parameters in your `*_cfg.ks` file. All nine default to `-1`, which uses the global fallback in `ifc_constants.ks`. Start with all at `-1` and override only the values that differ meaningfully from the defaults.

---

## The Nine Parameters

```kerboscript
"ascent_q_target",          -1,   // Pa  corridor centre q       (default: 30 000)
"ascent_q_max",             -1,   // Pa  structural limit        (default: 60 000)
"ascent_q_min",             -1,   // Pa  lower corridor bound    (default:  8 000)
"ascent_heat_limit",        -1,   // Pa·m/s  heating proxy       (default: 3.5e8)
"ascent_k_prop",            -1,   // propellant equivalency coef (default: 0.5)
"ascent_aoa_limit",         -1,   // deg  max AoA in ascent      (default: aa_max_aoa)
"ascent_regime_mach",       -1,   // Mach  AB thermal boundary   (default: 4.5)
"ascent_zoom_target_m",     -1,   // m   target apo before switch (default: 45 000)
"ascent_apoapsis_target_m", -1,   // m   final orbit apoapsis    (default: 80 000)
```

---

## 1. `ascent_q_max` — Structural Limit (Pa)

**What it controls:** J_ab degrades steeply above this dynamic pressure. The corridor controller also pitches down to avoid it.

**How to find it:** Look at the FAR Stability Analysis window during a high-speed level flight test. Watch for parts approaching their thermal limit or the FAR aero forces showing panel flex warnings. A safer method: fly manually at various altitudes/speeds and note the maximum q at which the vehicle feels stable and shows no structural warnings. Add a 10–15% margin below that value.

**Typical range:** 40 000–80 000 Pa. Heavier, more aerodynamically clean vehicles can sustain higher q. Fragile spaceplanes with large wings need a lower limit.

**If you get it wrong:** Too high — vehicle may overheat or shed parts during the corridor. Too low — corridor is artificially narrow, forcing early zoom and suboptimal AB use.

---

## 2. `ascent_q_target` — Corridor Centre (Pa)

**What it controls:** The dynamic pressure the corridor controller tries to maintain. This is where J_ab is assumed to be near its peak.

**How to find it:** From FAR data during manual flight, find the q at which your air-breathing engines produce maximum thrust-to-fuel-flow at cruise Mach numbers (typically Mach 2–4 for most RAPIER-class vehicles). The corridor controller will fly toward this value. Good starting point: 60–70% of `ascent_q_max`.

**Typical range:** 20 000–45 000 Pa.

**Practical note:** The default of 30 000 Pa works well for most RAPIER-type spaceplanes. Override it only if your engines have a clearly different performance peak — check the thrust curve in the VAB or FAR data.

---

## 3. `ascent_q_min` — Lower Corridor Bound (Pa)

**What it controls:** Below this q, the controller considers the corridor "left" and will trigger a zoom if apoapsis is still low. J_ab also starts degrading.

**How to find it:** The lower bound should represent the q below which your AB engines start losing meaningful thrust. For RAPIER-type engines this is roughly where intake air is insufficient — typically around 8 000–12 000 Pa depending on intake design.

**Typical range:** 5 000–15 000 Pa.

**If unsure:** Leave at `-1` (8 000 Pa default). Override upward if your vehicle has poor intake efficiency and loses thrust at higher q than usual.

---

## 4. `ascent_heat_limit` — Heating Proxy Limit (Pa·m/s)

**What it controls:** The product `q × v` is used as an aerodynamic heating proxy. As this approaches the limit, J_ab is penalised via P_heat, softly discouraging continued AB operation in thermally hazardous conditions.

**How to find it:** Check the FAR heating indicators and KSP's part temperature bars during high-speed flight. Note the `q` and `v` at the point where part temperatures start rising significantly. Multiply: `limit = q × v × 0.90` (10% margin below onset). For a typical spaceplane approaching the heating limit at q = 25 000 Pa and v = 1 800 m/s: limit ≈ 4.0 × 10⁸ Pa·m/s.

**Typical range:** 2.0 × 10⁸ to 5.0 × 10⁸ Pa·m/s.

**If unsure:** Leave at `-1` (3.5 × 10⁸ default). Raise it if your vehicle rarely heats up; lower it if you see thermal warnings during corridor flight.

---

## 5. `ascent_aoa_limit` — Maximum AoA in Ascent (deg)

**What it controls:** The zoom-phase pitch-up is halted when AoA approaches this limit. Also feeds the estimator validity AoA oscillation check.

**How to find it:** In FAR's Stability Analysis, look for the AoA at which lift starts to flatten or Cl/Cd peaks. Alternatively, fly manually at high Mach and note the AoA at which the vehicle becomes difficult to hold or FAR reports instability. Use 80–85% of the true stall AoA as your limit.

**Typical range:** 12–20 deg. Most stock-aerodynamics-style spaceplanes: 15 deg. FAR-modeled slender designs: 12 deg.

**Practical note:** If this is left at `-1`, it inherits `aa_max_aoa` from the aircraft config (global default: 12 deg). If your vehicle can safely fly at higher AoA in ascent (larger wings, better stability at high Mach), set this explicitly to unlock more zoom authority.

---

## 6. `ascent_regime_mach` — AB Thermal Regime Boundary (Mach)

**What it controls:** The Mach number at which your air-breathing engines approach their designed thermal limit and thrust begins to fall sharply. Used to distinguish a planned thrust decline from an unexpected flameout.

**How to find it:** Check your engine's thrust curve in the VAB editor or engine documentation. For RAPIER engines this is approximately Mach 4.5. For custom/modded engines, fly a level test at full throttle and note the Mach number where thrust starts declining noticeably.

**Typical range:** 3.5–5.5 Mach.

**Why it matters:** If set incorrectly too low, the flameout check is suppressed prematurely and a real flameout won't be detected. Too high — normal thrust decline near the regime boundary will be misread as a flameout and trigger an emergency mode switch.

---

## 7. `ascent_k_prop` — Propellant Equivalency Coefficient

**What it controls:** Weights J_rk to account for the different scarcity of liquid fuel (LF, used by AB engines) versus the LF+OX mixture used by rockets. When your oxidizer reserves are proportionally lower than your liquid fuel reserves, J_rk appears more expensive, favouring longer AB operation.

```
w_prop = 1 + k_prop × max(0, OX_fraction − LF_fraction)
```

**How to determine it:**
- **Symmetric tanks (equal LF and OX proportions):** leave at `-1` (0.5 default). The correction is small or zero when reserves are balanced.
- **Asymmetric tanks (more LF than OX, e.g., drop tanks for AB phase):** raise toward 1.0 or higher to make the controller more conservative about expending OX.
- **OX-heavy vehicles:** lower toward 0.1–0.2.

**Practical note:** For most simple spaceplanes with a single engine set and proportional tanks, this parameter has almost no effect. It becomes meaningful only when you have drop tanks, separate AB/rocket fuel bays, or vehicles with a large imbalance between LF and OX capacity.

---

## 8. `ascent_zoom_target_m` — Target Apoapsis Before Mode Switch (m)

**What it controls:** The apoapsis altitude the vehicle tries to reach during the zoom climb before committing to rocket mode. Higher values mean more kinetic energy is traded for altitude before the switch, reducing atmospheric drag burden on the rocket phase.

**How to determine it:**
1. Fly a manual test ascent and note the apoapsis at the point where AB thrust is no longer useful (near `ascent_regime_mach`).
2. The zoom target should be high enough that the rocket phase begins in thin air but not so high that the zoom burns too much AB propellant without climbing further.
3. As a rough guide: aim for the apoapsis to be above ~35–40 km so the rocket phase avoids most atmospheric drag. 45 000 m (default) works well for typical Kerbin ascents.

**Typical range:** 35 000–60 000 m.

**Interaction:** If your rocket TWR is low (< 1.4), raise this value — you need a higher "launch" apoapsis to compensate for the slower climb. If your vehicle has high rocket TWR, you can lower it and switch earlier.

---

## 9. `ascent_apoapsis_target_m` — Final Orbit Apoapsis (m)

**What it controls:** The target apoapsis altitude for the circularisation orbit. ROCKET_SUSTAIN drives apoapsis to this altitude; ROCKET_CLOSEOUT accumulates orbital velocity while keeping apoapsis within a band around it; CIRCULARISE then closes the orbit.

**How to set it:** This is a mission parameter, not a vehicle characteristic. Set it to your intended circular orbit altitude. Common choices:
- 80 000 m — minimum stable orbit above Kerbin's atmosphere (default)
- 100 000 m — comfortable low Kerbin orbit with margin
- 250 000 m+ — for missions requiring higher orbits

**Note:** The controller targets this apoapsis, then coasts and circularises. If you later want a different orbit, change this value. The vehicle's structural limits are not a concern here — by the rocket phase you are well above the atmosphere.

---

## Quick Tuning Checklist

| Step | What to do |
|------|------------|
| 1 | Set `ascent_regime_mach` from your engine's thrust curve |
| 2 | Set `ascent_q_max` from a high-speed stress test (q at structural onset − 15%) |
| 3 | Set `ascent_aoa_limit` from FAR stability analysis (80% of stall AoA at peak Mach) |
| 4 | Set `ascent_apoapsis_target_m` to your mission orbit altitude |
| 5 | Leave everything else at `-1` for the first test flight |
| 6 | Review the CSV log: check `asc_q_pa` vs. `ascent_q_target`, `asc_mach` at AB dropout, and `asc_j_ab`/`asc_j_rk` crossover timing |
| 7 | Adjust `ascent_q_target` if the vehicle is flying too low/fast or too high/slow |
| 8 | Adjust `ascent_zoom_target_m` if the mode switch happens too early (apo too low at switch) |
| 9 | Adjust `ascent_heat_limit` only if thermal warnings appear during corridor flight |
| 10 | Adjust `ascent_k_prop` only if your vehicle has asymmetric LF/OX tank ratios |

## Diagnosing from the Log

The CSV log columns `asc_j_ab`, `asc_j_rk`, `asc_q_pa`, `asc_mach`, `asc_validity`, and `asc_apo_m` are the primary diagnostic signals:

- **`asc_validity` stuck at DEGRADED** throughout the flight — this is a known issue with the orbital energy rate cross-check being noisy; it will widen the persistence window (conservative but functional).
- **Mode switch happens too early** (low apo at switch) — raise `ascent_zoom_target_m`.
- **Vehicle never zooms, stays in corridor until AB flame-out** — lower `ascent_q_min` or lower `ascent_zoom_target_m`.
- **Large apo overshoot before CIRCULARISE** — `ascent_zoom_target_m` is too high, or the zoom pitch rate is too aggressive (global constant `ASC_ZOOM_PITCH_RATE`).
- **Thermal warnings during corridor** — lower `ascent_heat_limit` by 10–20%.
- **`asc_j_ab` never exceeds `asc_j_rk`** — either the AB engines are weak relative to the rocket engines, or `ascent_q_target` is too far from the vehicle's performance optimum; try raising it.
