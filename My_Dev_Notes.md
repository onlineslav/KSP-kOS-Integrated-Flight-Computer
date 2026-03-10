Would be good if the FC could automatically pull the AC config from the name of the aircraft?

Is there a way we could limit bank angle during all phases of flight (except maybe in space but we can patch that in later)? AA doesn't have a bank limiter

(note for me)
Add these to cfg files?
Vs: stall speed in the configuration you mean

Vs0: stall speed in landing configuration
Usually: gear down, landing flaps, final landing mass

Vref: reference approach speed on final
Usually based on 1.3 × Vs0

Vapp: actual flown approach speed
Often Vref plus a small additive for gusts / handling margin

Vtd: target touchdown speed
Usually a bit below Vref, but still safely above stall

Vfe: maximum flap extension speed

---

Would be nice:
fine tune spoiler deflection and fine tune braking
- maybe by changing the deflection angle in real time?

set moderate aoa value to just below A_crit from FAR data

---

Some sensible KSP GUIs would be nice.

takeoff phase mode:
- does preflight checks
    - sets flaps to takeoff
    - sets ksp camera to locked
    - determine takeoff speed derived from aircraft variables

## Codex Notes - Determining v_r Theoretically

Use `v_r` as a dynamic-pressure problem, not a fixed speed guess.

1. Define the exact takeoff configuration
`mass = brake-release mass`, `flaps = takeoff`, `gear = down`, runway altitude.

2. Get FAR aero numbers in that config
From FAR static analysis, read:
- `S` (effective wing area)
- `C_Lmax_TO` (or at least `C_L` at your planned rotation AoA)
- `alpha_crit` (stall AoA)

3. Compute takeoff stall speed
$$
V_{S,TO}=\sqrt{\frac{2\,m\,g}{\rho\,S\,C_{Lmax,TO}}}
$$
Use local density `rho` at runway altitude.

4. Set first-pass speeds
- `V_LOF` (liftoff): `1.05–1.10 * V_S_TO`
- `V_R` (rotation command): usually slightly below `V_LOF` because pitch-up takes time
  Practical first pass: `V_R = 1.03–1.08 * V_S_TO`
- `V2`: `1.15–1.25 * V_S_TO`

5. AoA sanity check (important with FAR)
At your chosen `V_R`, required lift coefficient is:
$$
C_{L,req}=\frac{2\,m\,g}{\rho\,V_R^2\,S}
$$
Ensure this is achievable at your rotation AoA and that rotation AoA stays below `alpha_crit` by ~2-4 deg.

6. Map to IFC config
Set `v_r`, `v2`, and `takeoff_pitch_tgt` in `x10_f_cfg.ks`, then flight-test and adjust a few m/s for rotation lag/runway length.


## X10-F Reference Values

### Takeoff
$m_{to} = 99025kg$

$\alpha_{tailstrike} \approx10 deg$

$\alpha_{CR,TO}\approx 45deg$