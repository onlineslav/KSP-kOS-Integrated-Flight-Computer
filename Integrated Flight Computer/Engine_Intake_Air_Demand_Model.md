# Engine Intake Air Demand Model

Reference document for building a feedforward intake air starvation predictor.
Data extracted directly from part CFGs across all installed engine and intake mods.

---

## 1. Model Purpose

The goal is to predict intake air starvation **before** flameout occurs — i.e., before
`eng:CONSUMEDRESOURCES["IntakeAir"]:FUELFLOW` falls below `REQUIREDFLOW` at runtime.

A feedforward model answers: given throttle command, current Mach, and current
atmospheric pressure, what is the expected IntakeAir demand, and how does it compare
to the available supply from the installed intakes?

---

## 2. Runtime kOS Starvation Signal

The simplest live signal — no model needed:

```kerboscript
// Returns 0-1 satisfaction fraction (1.0 = fully fed, <1.0 = already starving)
FUNCTION GET_AIR_SAT_FRACTION {
    LOCAL worst IS 1.0.
    LIST ENGINES IN eng_list.
    FOR eng IN eng_list {
        IF eng:IGNITION AND NOT eng:FLAMEOUT {
            LOCAL cres IS eng:CONSUMEDRESOURCES.
            IF cres:HASKEY("IntakeAir") {
                LOCAL air IS cres["IntakeAir"].
                LOCAL req IS air:REQUIREDFLOW.
                IF req > 0.001 {
                    LOCAL frac IS air:FUELFLOW / req.
                    IF frac < worst { SET worst TO frac. }
                }
            }
        }
    }
    RETURN worst.
}
```

`FUELFLOW < REQUIREDFLOW` means starvation is already degrading thrust. The
feedforward model below predicts this condition from first principles so the
autopilot can act before it occurs.

---

## 3. Feedforward Model Math

### 3.1 Engine Thrust

```
F(throttle, Mach, P_atm) = maxThrust × throttle × velCurve(Mach) × atmCurve(P_atm)
```

Where:
- `maxThrust` — reference thrust from CFG (kN)
- `throttle` — commanded throttle, 0–1
- `velCurve(Mach)` — piecewise Hermite spline, keys extracted per engine below
- `atmCurve(P_atm)` — piecewise Hermite spline vs. pressure in atm (1.0 = Kerbin SL)

### 3.2 Total Mass Flow

```
ṁ_total = F / (Isp × g0)
```

Where:
- `Isp` — from `atmosphereCurve` (for most jets, a single key at pressure=0 gives
  effective vacuum Isp; the curves multiply thrust independently)
- `g0 = 9.80665 m/s²`

### 3.3 IntakeAir Required Flow

KSP resource densities (kg/unit):
- LiquidFuel: **5 kg/unit**
- IntakeAir: **5 kg/unit** (same density)

Because densities are equal, mass ratio = volume ratio:

```
ṁ_IA = ṁ_total × r_IA / (r_LF + r_IA)

Q_IA_required (units/s) = ṁ_IA / 5.0
                        = F / (Isp × g0 × 5.0) × r_IA / (r_LF + r_IA)
```

Where `r_IA` and `r_LF` are the PROPELLANT ratio values from the engine CFG.

**Full feedforward formula:**

```
Q_IA_required = (maxThrust × throttle × velCurve(Mach) × atmCurve(P_atm))
              / (Isp_vac × 9.80665 × 5.0)
              × r_IA / (r_LF + r_IA)
```

### 3.4 Intake Air Supply

Each intake contributes:

```
Q_supply (units/s) = area × intakeSpeed × ρ_atm(alt) × machCurve(Mach)
```

Where:
- `area` — intake cross-section area (m²) from CFG
- `intakeSpeed` — CFG multiplier (m/s equivalent)
- `ρ_atm(alt)` — atmospheric density at altitude (kg/m³)
- `machCurve(Mach)` — piecewise Hermite spline, keys per intake below

Kerbin atmospheric density can be approximated from pressure:
```kerboscript
LOCAL p_atm IS SHIP:BODY:ATM:ALTITUDEPRESSURE(SHIP:ALTITUDE). // atm
// Density at Kerbin SL ≈ 1.225 kg/m³; scales roughly linearly with pressure
LOCAL rho IS 1.225 * p_atm.
```

Total ship supply = sum over all open intakes.

### 3.5 Starvation Margin

```
margin = Q_supply_total - Q_IA_required_total
```

- `margin > 0` → intake air surplus
- `margin ≤ 0` → starvation occurring or imminent

### 3.6 Predictive Lookahead

```kerboscript
LOCAL lookahead_s IS 30.
LOCAL future_alt IS SHIP:ALTITUDE + SHIP:VERTICALSPEED * lookahead_s.
LOCAL future_p   IS SHIP:BODY:ATM:ALTITUDEPRESSURE(future_alt).
LOCAL future_rho IS 1.225 * future_p.
// Re-evaluate supply with future_rho; demand is unchanged if throttle/Mach held constant.
```

This gives a conservative starvation warning (ignores ram pressure increase with
speed, so true ceiling is slightly higher than predicted).

---

## 4. Engine Data

Curve format: `key = [x] [y] [in_tangent] [out_tangent]`
Tangents are Hermite spline slopes. Where only 2 values are shown, tangents default to 0.

---

### 4.1 Squad (Stock) Engines

---

#### J-20 "Juno" Basic Jet Engine

| Field | Value |
|---|---|
| Part name | `miniJetEngine` |
| maxThrust | 20 kN |
| machLimit | 0.75 |
| IntakeAir ratio | 22 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 6400 s |
| Air:Fuel mass ratio | 22:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0           0
key = 0.44   0.897  0           0
key = 1      1      0.1988732   0.1988732
key = 1.3    1.03   0           0
key = 2      0.68   -1.065708   -1.065708
key = 2.4    0      0           0
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      0           0.7448742
key = 0.072  0.13   2.075459    2.075459
key = 0.16   0.28   1.464173    1.464173
key = 0.42   0.578  0.93687     0.93687
key = 1      1      0.5529748   0
```

---

#### J-33 "Wheesley" Turbofan Engine

| Field | Value |
|---|---|
| Part name | `JetEngine` |
| maxThrust | 120 kN |
| machLimit | 0.85 |
| IntakeAir ratio | 127 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 10500 s |
| Air:Fuel mass ratio | 127:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0           0
key = 0.53   0.834  0           0
key = 1.3    0.96   0           0
key = 1.674  0.843  -0.876726   -0.876726
key = 2.5    0      0           0
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      0           1.186726
key = 0.072  0.092  1.339822    1.339822
key = 0.337  0.4    0.8976688   0.8976688
key = 1      1      0.9127604   0
```

---

#### J-90 "Goliath" Turbofan Engine

| Field | Value |
|---|---|
| Part name | `turboFanSize2` |
| maxThrust | 360 kN |
| machLimit | 0.75 |
| IntakeAir ratio | 227 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 12600 s |
| Air:Fuel mass ratio | 227:1 |
| Built-in intake area | 0.03 m² |
| Built-in intakeSpeed | 30 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      -0.1543096  -0.1543096
key = 0.61   0.79   0           0
key = 1.5    0.964  0           0
key = 2      0.31   -3.278422   -3.278422
key = 2.1    0      -0.9205825  -0.9205825
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      0           0
key = 0.072  0.085  1.172947    1.172947
key = 0.337  0.37   0.98425     0.98425
key = 1      1      1.179067    1.179067
```

---

#### J-404 "Panther" Afterburning Turbofan — DRY mode

| Field | Value |
|---|---|
| Part name | `turboJet` |
| maxThrust | 85 kN |
| machLimit | 1.75 |
| IntakeAir ratio | 40 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 9000 s |
| Air:Fuel mass ratio | 40:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0           0
key = 0.35   0.932  0           0
key = 1      1.13   0.4510796   0.4510796
key = 1.75   1.5    0           0
key = 2      1.38   -1.126258   -1.126258
key = 2.5    0      0           0
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      1.069445    0.7244952
key = 0.072  0.08   1.472049    1.472049
key = 0.17   0.21   1.227685    1.227685
key = 0.34   0.39   1.01426     1.01426
key = 1      1      0.969697    0.969697
```

#### J-404 "Panther" Afterburning Turbofan — WET (afterburner) mode

| Field | Value |
|---|---|
| maxThrust | 130 kN |
| machLimit | 1.75 |
| IntakeAir ratio | 12 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 4000 s |
| Air:Fuel mass ratio | 12:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0           0
key = 0.18   0.97   0           0
key = 0.43   1      0.202683    0.202683
key = 1      1.42   1.280302    1.280302
key = 2.5    3.63   0           0
key = 3      0.58   -2.708558   -2.708558
key = 3.35   0      -0.6150925  0
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      1.666667    1.666667
key = 0.0707 0.1397 1.961396    1.961396
key = 0.34   0.56   1.084002    1.084002
key = 1      1      0.5302638   0.5302638
```

---

#### J-X4 "Whiplash" Turbo Ramjet Engine

| Field | Value |
|---|---|
| Part name | `turboFanEngine` |
| maxThrust | 130 kN |
| machLimit | 2.5 |
| IntakeAir ratio | 8 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 4000 s |
| Air:Fuel mass ratio | 8:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0             0
key = 0.2    0.98   0             0
key = 0.72   1.716  2.433527      2.433527
key = 1.36   3.2    1.986082      1.986082
key = 2.15   4.9    1.452677      1.452677
key = 3      5.8    0.0005786046  0.0005786046
key = 4.5    3      -4.279616     -4.279616
key = 5.5    0      -0.02420209   0
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      0           0
key = 0.045  0.166  4.304647    4.304647
key = 0.16   0.5    0.5779132   0.5779132
key = 0.5    0.6    0.4809403   0.4809403
key = 1      1      1.013946    0
```

---

#### CR-7 R.A.P.I.E.R. — AIR-BREATHING mode

| Field | Value |
|---|---|
| Part name | `RAPIER` |
| maxThrust | 105 kN |
| IntakeAir ratio | 6 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 3200 s |
| Air:Fuel mass ratio | 6:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0           0.08333334
key = 0.2    0.98   0.42074     0.42074
key = 0.7    1.8    2.290406    2.290406
key = 1.4    4.00   3.887193    3.887193
key = 3.75   8.5    0           0
key = 4.5    7.3    -2.831749   -2.831749
key = 5.5    3      -5.260566   -5.260566
key = 6      0      -0.02420209 0
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      0           0
key = 0.018  0.09   7.914787    7.914787
key = 0.08   0.3    1.051923    1.051923
key = 0.35   0.5    0.3927226   0.3927226
key = 1      1      1.055097    0
```

*Note: RAPIER closed-cycle rocket mode uses LF/Oxidizer only — no IntakeAir.*

---

### 4.2 NearFutureAeronautics Engines

---

#### NFA Turbojet 2.5m

| Field | Value |
|---|---|
| Part name | `nfa-turbojet-25-1` |
| maxThrust | 540 kN |
| machLimit | 2.8 |
| IntakeAir ratio | 7 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 4300 s |
| Air:Fuel mass ratio | 7:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0             0
key = 0.2    0.98   0             0
key = 0.72   1.716  2.433527      2.433527
key = 1.36   3.2    1.986082      1.986082
key = 2.15   4.9    1.452677      1.452677
key = 3      5.8    0.0005786046  0.0005786046
key = 4.5    3      -4.279616     -4.279616
key = 5.5    0      -0.02420209   0
```
*(Identical to Whiplash velCurve; maxThrust is larger.)*

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      0           0
key = 0.045  0.166  4.304647    4.304647
key = 0.16   0.5    0.5779132   0.5779132
key = 0.5    0.6    0.4809403   0.4809403
key = 1      1      1.013946    0
```

---

#### NFA Turbofan 2.5m

| Field | Value |
|---|---|
| Part name | `nfa-turbofan-25-1` |
| maxThrust | 380 kN |
| machLimit | 0.76 |
| IntakeAir ratio | 11 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 10600 s |
| Air:Fuel mass ratio | 11:1 |

**velCurve** (thrust × vs Mach):
```
key = 0        1      0           -0.125804
key = 0.35     0.96   0           0
key = 0.8671   1.17   0.5409369   0.8292115
key = 1.3943   1.5    0           0
key = 2.2051   1.025  -1.837343   -3.49991
key = 2.4      0.5    -3.164169   -3.164169
key = 2.5      0      0.004306508 0
```

**atmCurve** (thrust × vs P_atm):
```
key = 0        0      0           0
key = 0.1878   0.4941 1.429354    2.48742
key = 0.3714   0.7589 0.8174777   0.8174777
key = 0.7211   0.8694 0.02965302  0.1299747
key = 1        1      0.9829093   0
```

---

#### NFA Turbofan V2 2.5m — CRUISE mode

| Field | Value |
|---|---|
| Part name | `nfa-turbofan-25-2` |
| maxThrust | 510 kN |
| machLimit | 2.1 |
| IntakeAir ratio | 40 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 9000 s |
| Air:Fuel mass ratio | 40:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0           0
key = 0.35   0.932  0           0
key = 1      1.13   0.4510796   0.4510796
key = 2      1.5    0           0
key = 2.25   1.38   -1.126258   -1.126258
key = 3      0      0           0
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      1.069445    0.7244952
key = 0.072  0.08   1.472049    1.472049
key = 0.17   0.21   1.227685    1.227685
key = 0.34   0.39   1.01426     1.01426
key = 1      1      0.969697    0.969697
```

*Reverse thrust mode uses identical curves but maxThrust = 260 kN.*

---

#### NFA Turboprop 1.25m

| Field | Value |
|---|---|
| Part name | `nfa-turboprop-125-1` |
| maxThrust | 148 kN |
| IntakeAir ratio | 18 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 11600 s |
| Air:Fuel mass ratio | 18:1 |
| Built-in intake area | 0.004 m² |
| Built-in intakeSpeed | 10 |

**velCurve** (thrust × vs Mach):
```
key = 0      0.7    0           0
key = 0.15   1      0.006952    0.006952
key = 0.35   1      0.009030    0.009030
key = 0.8    0.1    -0.6571437  -0.6571437
key = 1.4    0      0           0
```

**atmCurve** (thrust × vs P_atm — note: turboprop runs above 1 atm at SL):
```
key = 0.1      0      0           0
key = 0.651329 0.9368 0.629773    0.629773
key = 1        1      0           0
key = 5        2.5    0           0
key = 20       4      0.03074977  0.04311482
```

---

#### NFA Propfan 1.25m

| Field | Value |
|---|---|
| Part name | `nfa-propfan-125-1` |
| maxThrust | 120 kN |
| IntakeAir ratio | 18 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 13000 s |
| Air:Fuel mass ratio | 18:1 |
| Built-in intake area | 0.004 m² |
| Built-in intakeSpeed | 10 |

**velCurve** (thrust × vs Mach): *(identical to turboprop)*
```
key = 0      0.7    0           0
key = 0.15   1      0.006952    0.006952
key = 0.35   1      0.009030    0.009030
key = 0.8    0.1    -0.6571437  -0.6571437
key = 1.4    0      0           0
```

**atmCurve** (thrust × vs P_atm): *(identical to turboprop)*
```
key = 0.1      0      0           0
key = 0.651329 0.9368 0.629773    0.629773
key = 1        1      0           0
key = 5        2.5    0           0
key = 20       4      0.03074977  0.04311482
```

---

### 4.3 B9 Aerospace Engines

---

#### B9 D-30F7 Turbojet — DRY mode

| Field | Value |
|---|---|
| Part name | `B9_Engine_Jet_Turbojet` |
| maxThrust | 95 kN |
| machLimit | 2.75 |
| IntakeAir ratio | 21 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 7500 s |
| Air:Fuel mass ratio | 21:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0      0
key = 0.35   0.95   0      0
key = 1      1.25   1      1
key = 2      2      0      0
key = 2.5    0      -8     0
```

**atmCurve**: not specified in CFG — defaults to linear (thrust ∝ pressure).

#### B9 D-30F7 Turbojet — WET (afterburner) mode

| Field | Value |
|---|---|
| maxThrust | 155 kN |
| machLimit | 2.75 |
| IntakeAir ratio | 12 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 4000 s |
| Air:Fuel mass ratio | 12:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0      0
key = 0.2    0.975  0      0
key = 1      2      2      2
key = 3      5      0      0
key = 4      0      -3     0
```

---

#### B9 TFE731 Turbofan (Pod Small)

| Field | Value |
|---|---|
| Part name | `B9_Engine_Jet_Pod_Small` |
| maxThrust | 40 kN |
| machLimit | 0.85 |
| IntakeAir ratio | 90 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 10000 s |
| Air:Fuel mass ratio | 90:1 |
| Built-in intake area | 0.00512 m² |
| Built-in intakeSpeed | 30 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0           0
key = 0.53   0.834  0           0
key = 1.3    0.96   0           0
key = 1.674  0.843  -0.876726   -0.876726
key = 2      0      0           0
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      0           1.186726
key = 0.072  0.092  1.339822    1.339822
key = 0.337  0.4    0.8976688   0.8976688
key = 1      1      0.9127604   0
```

---

### 4.4 NeistAir Engines

---

#### NeistAir CF34-8C

| Field | Value |
|---|---|
| Part name | `CF34-8C` |
| maxThrust | 50 kN |
| machLimit | 0.75 |
| IntakeAir ratio | 227 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 9600 s |
| Air:Fuel mass ratio | 227:1 |
| Built-in intake area | 0.005 m² |
| Built-in intakeSpeed | 30 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      -0.1543096  -0.1543096
key = 0.61   0.79   0           0
key = 1.2    0.7    0           0
key = 2      0.31   -3.278422   -3.278422
key = 2.1    0      -0.9205825  -0.9205825
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      0           0
key = 0.072  0.085  1.172947    1.172947
key = 0.337  0.4    0.98425     0.98425
key = 0.5    0.7    0.98425     0.98425
key = 1      1      1.179067    1.179067
```

---

### 4.5 AirplanePlus Engines

---

#### AirplanePlus J-56 "Lotus" High-Bypass Turbofan

| Field | Value |
|---|---|
| Part name | `cfm56` |
| maxThrust | 180 kN |
| machLimit | 0.75 |
| IntakeAir ratio | 227 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 12600 s |
| Air:Fuel mass ratio | 227:1 |
| Built-in intake area | 0.03 m² |
| Built-in intakeSpeed | 30 |

**velCurve** (thrust × vs Mach):
```
key = 0        1        -0.1543096  -0.1543096
key = 0.1819   0.8587   0           0
key = 0.4      0.964    0           0
key = 0.6525   0.6440   -3.278422   -2.582144
key = 0.72     0        -17.68372   -8.84913
```

**atmCurve** (thrust × vs P_atm):
```
key = 0      0      0           0
key = 0.072  0.085  1.172947    1.172947
key = 0.337  0.37   0.98425     0.98425
key = 1      1      1.179067    1.179067
```

---

#### AirplanePlus J-34 "Chevron" High-Bypass Turbofan

| Field | Value |
|---|---|
| Part name | `cf34` |
| maxThrust | 100 kN |
| machLimit | 0.75 |
| IntakeAir ratio | 227 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 12600 s |
| Air:Fuel mass ratio | 227:1 |
| Built-in intake area | 0.03 m² |
| Built-in intakeSpeed | 30 |

**velCurve** and **atmCurve**: identical to J-56 "Lotus" above.

---

### 4.6 OPT Engines

---

#### OPT Mk2 Engine Short

| Field | Value |
|---|---|
| Part name | `opt_mk2_engine_short` |
| maxThrust | 200 kN |
| machLimit | 6.0 |
| IntakeAir ratio | 2.25 |
| LiquidFuel ratio | 1 |
| Isp (vac equiv) | 4000 s |
| Air:Fuel mass ratio | 2.25:1 |

**velCurve** (thrust × vs Mach):
```
key = 0      1      0.07    0.07
key = 1.5    1.6    0.95    0.95
key = 2.7    2.4    0       0
key = 4.2    2.4    0       0
key = 6.4    1.44   0       0
key = 7.5    0      0       0
```

**atmCurve** (thrust × vs P_atm):
```
key = 0          0      0           0
key = 0.005      0.3    0           0
key = 0.03469    0.8917 19.92897    0.5922443
key = 0.2        1      0           0
key = 0.3        1      0           0
key = 1.893259   0.6    0           0
key = 7.5        0      0           0
```

---

## 5. Intake Data

Supply formula: `Q_supply (units/s) = area × intakeSpeed × ρ_atm × machCurve(Mach)`

Curve format: `key = [Mach] [efficiency] [in_tangent] [out_tangent]`

---

### 5.1 Squad (Stock) Intakes

#### Circular Intake

| Field | Value |
|---|---|
| Part name | `CircularIntake` |
| area | 0.006 m² |
| intakeSpeed | 15 |

**machCurve:**
```
key = 1      1      0           0
key = 1.5    0.9    -0.4312553  -0.4312553
key = 2.5    0.45   -0.5275364  -0.5275364
key = 3.5    0.1    0           0
```
*Note: curve starts at Mach 1. At subsonic speeds (Mach < 1), efficiency = 1.0 (flat extrapolation).*

---

#### Small Circular Intake

| Field | Value |
|---|---|
| Part name | `miniIntake` |
| area | 0.001 m² |
| intakeSpeed | 15 |

**machCurve:** identical to Circular Intake above.

---

#### XM-G50 Radial Air Intake

| Field | Value |
|---|---|
| Part name | `airScoop` |
| area | 0.0031 m² |
| intakeSpeed | 15 |

**machCurve:** identical to Circular Intake above.

---

#### Adjustable Ramp Intake (inline)

| Field | Value |
|---|---|
| Part name | `ramAirIntake` |
| area | 0.0074 m² |
| intakeSpeed | 10 |

**machCurve:**
```
key = 0      0.85   0           0
key = 1      1      0           0
key = 2      0.95   -0.08751557 -0.08751557
key = 4      0.5    -0.4034287  -0.4034287
key = 8      0.01   0           0
```

---

#### Adjustable Ramp Intake (radial)

| Field | Value |
|---|---|
| Part name | `IntakeRadialLong` |
| area | 0.001 m² |
| intakeSpeed | 10 |

**machCurve:** identical to Adjustable Ramp Intake (inline) above.

---

#### Shock Cone Intake

| Field | Value |
|---|---|
| Part name | `shockConeIntake` |
| area | 0.0075 m² |
| intakeSpeed | 5 |

**machCurve:** Flat efficiency = 1.0 across all Mach numbers (no curve defined in CFG — isentropic default). This intake is deliberately tuned for supersonic efficiency by its geometry; the low `intakeSpeed` keeps subsonic supply modest while remaining efficient at high Mach.

---

### 5.2 B9 Aerospace Intakes

#### B9 DSI Diverterless Supersonic Inlet

| Field | Value |
|---|---|
| Part name | `B9_Aero_Intake_DSI` |
| area | 0.003 m² |
| intakeSpeed | 15 |

**machCurve:**
```
key = 0      0.9    0           0
key = 1      1      0           0
key = 1.5    0.95   -0.1608955  -0.1608955
key = 3      0.6    -0.4034287  -0.4034287
key = 5      0.01   0           0
```

---

#### B9 Circular Intake (High-Flow)

| Field | Value |
|---|---|
| Part name | `B9_Aero_Intake_CLR` |
| area | 0.0075 m² |
| intakeSpeed | 50 |

**machCurve:** identical to stock Circular Intake (subsonic-biased, falls off above Mach 1.5).

---

## 6. Quick Reference — IntakeAir Demand by Engine Type

At Mach 0, sea level (P_atm = 1.0, velCurve = 1.0, atmCurve = 1.0), full throttle:

```
Q_IA = maxThrust / (Isp_vac × 9.80665 × 5.0) × r_IA / (r_LF + r_IA)   [units/s]
```

| Engine | maxThrust kN | Isp_vac s | r_IA | Q_IA @ SL full throttle (units/s) |
|---|---|---|---|---|
| J-20 Juno | 20 | 6400 | 22 | 0.138 |
| J-33 Wheesley | 120 | 10500 | 127 | 0.278 |
| J-90 Goliath | 360 | 12600 | 227 | 0.526 |
| J-404 Panther dry | 85 | 9000 | 40 | 0.081 |
| J-404 Panther wet | 130 | 4000 | 12 | 0.095 |
| J-X4 Whiplash | 130 | 4000 | 8 | 0.058 |
| RAPIER air | 105 | 3200 | 6 | 0.040 |
| NFA Turbojet 2.5 | 540 | 4300 | 7 | 0.163 |
| NFA Turbofan 2.5 | 380 | 10600 | 11 | 0.074 |
| NFA Turbofan V2 2.5 | 510 | 9000 | 40 | 0.488 |
| NFA Turboprop 1.25 | 148 | 11600 | 18 | 0.051 |
| NFA Propfan 1.25 | 120 | 13000 | 18 | 0.038 |
| B9 D-30F7 dry | 95 | 7500 | 21 | 0.122 |
| B9 D-30F7 wet | 155 | 4000 | 12 | 0.113 |
| B9 TFE731 | 40 | 10000 | 90 | 0.073 |
| NeistAir CF34-8C | 50 | 9600 | 227 | 0.241 |
| AirplanePlus J-56 | 180 | 12600 | 227 | 0.263 |
| AirplanePlus J-34 | 100 | 12600 | 227 | 0.146 |
| OPT Mk2 Short | 200 | 4000 | 2.25 | 0.022 |

*Multiply by `velCurve(Mach) × atmCurve(P_atm) × throttle` for off-nominal conditions.*
*At high Mach, Whiplash/RAPIER/NFA-TJ demand grows dramatically due to velCurve > 1.*

---

## 7. Engine Spool-Up / Spool-Down Times

Spool dynamics are controlled by `engineAccelerationSpeed` (k_up) and
`engineDecelerationSpeed` (k_down) inside `ModuleEnginesFX`. KSP applies these
as a **first-order exponential approach** each physics tick:

```
θ[n+1] = θ[n] + (θ_cmd - θ[n]) × k × dt
```

In continuous time: `θ(t) = 1 - e^(-k·t)` for a 0→1 step input.

```
T63  = 1 / k          (time to reach 63% of commanded throttle)
T90  = 2.303 / k      (time to reach 90%)
T100 → ∞              (asymptotic — never truly reaches commanded value)
```

For a **throttle chop** (1→0): T63_down = 1 / k_down using the same formula.

**TweakScale does not affect spool time** — k_up and k_down are fixed properties
of the engine type regardless of diameter scaling.

| Engine | Part Name | k_up | k_down | T63_up (s) | T63_down (s) |
|---|---|---|---|---|---|
| J-20 Juno | `miniJetEngine` | 0.12 | 0.50 | 8.3 | 2.0 |
| J-33 Wheesley | `JetEngine` | 0.12 | 0.50 | 8.3 | 2.0 |
| J-90 Goliath | `turboFanSize2` | 0.06 | 0.25 | 16.7 | 4.0 |
| J-404 Panther dry | `turboJet` | 0.50 | 0.50 | 2.0 | 2.0 |
| J-404 Panther wet | `turboJet` | 0.80 | 0.80 | 1.25 | 1.25 |
| J-X4 Whiplash | `turboFanEngine` | 0.20 | 0.40 | 5.0 | 2.5 |
| RAPIER air-breathing | `RAPIER` | 0.20 | 0.35 | 5.0 | 2.9 |
| NFA Turbojet 2.5m | `nfa-turbojet-25-1` | 0.20 | 0.40 | 5.0 | 2.5 |
| NFA Turbofan 2.5m | `nfa-turbofan-25-1` | 0.10 | 0.40 | 10.0 | 2.5 |
| NFA Turbofan V2 2.5m | `nfa-turbofan-25-2` | 0.20 | 0.40 | 5.0 | 2.5 |
| NFA Turboprop 1.25m | `nfa-turboprop-125-1` | 0.15 | 0.40 | 6.7 | 2.5 |
| NFA Propfan 1.25m | `nfa-propfan-125-1` | 0.15 | 0.40 | 6.7 | 2.5 |
| B9 D-30F7 dry | `B9_Engine_Jet_Turbojet` | 0.12 | 0.21 | 8.3 | 4.8 |
| B9 D-30F7 wet | `B9_Engine_Jet_Turbojet` | 0.80 | 0.80 | 1.25 | 1.25 |
| B9 TFE731 pod small | `B9_Engine_Jet_Pod_Small` | 0.35 | 0.55 | 2.9 | 1.8 |
| B9 F119 turbofan dry | `B9_Engine_Jet_Turbofan_F119` | 0.25 | 0.45 | 4.0 | 2.2 |
| B9 F119 turbofan wet | `B9_Engine_Jet_Turbofan_F119` | 0.80 | 0.80 | 1.25 | 1.25 |
| NeistAir CF34-8C | `CF34-8C` | 0.06 | 0.25 | 16.7 | 4.0 |
| AirplanePlus J-56 Lotus | `cfm56` | 0.06 | 0.25 | 16.7 | 4.0 |
| AirplanePlus J-34 Chevron | `cf34` | 0.06 | 0.25 | 16.7 | 4.0 |
| AirplanePlus F5/Tiger dry | `f5jet` | 0.50 | 0.50 | 2.0 | 2.0 |
| AirplanePlus F5/Tiger wet | `f5jet` | 0.80 | 0.80 | 1.25 | 1.25 |
| AirplanePlus Raptor dry | `raptorjet` | 0.50 | 0.50 | 2.0 | 2.0 |
| AirplanePlus Raptor wet | `raptorjet` | 0.80 | 0.80 | 1.25 | 1.25 |
| OPT Mk2 Short | `opt_mk2_engine_short` | 0.20 | 0.40 | 5.0 | 2.5 |

**Cross-validation note:** A measured T63_up ≈ 2.1 s implies k_up ≈ 0.48, consistent
with Panther dry (k=0.50). The Wheesley (k=0.12) has T63_up ≈ 8.3 s — verify
which engine was used in any empirical spool tests before trusting a combo key
assignment.

---

## 8. Default Engine Diameters (TweakScale Reference)

The `defaultScale` value is the unscaled diameter KSP TweakScale uses as the
1.0× reference. To apply the scaling laws from the fitted engine model:

```
scale_factor = D_actual / defaultScale
thrust_scaled = thrust_ref × scale_factor ^ alpha_thrust
```

Where `D_actual` is the diameter set in TweakScale in-game, and `D_ref` in the
IFC model JSON is the diameter at which the reference test run was performed
(not necessarily the same as `defaultScale`).

**Sources:**
- Stock engines: `GameData/TweakScale/patches/Squad/Engines.cfg`
- NeistAir CF34-8C: `TweakScale MODULE` block inside the part CFG
- AirplanePlus engines: `TweakScaleCompanion/AirCrafts/APP/patches/AirplanePlus-Engine_TweakScale.cfg`
- NFA engines: inferred from `node_stack` size field (0=0.625m, 1=1.25m, 2=2.5m, 3=3.75m)
- B9 engines: inferred from `node_stack` size field (no explicit TweakScale patch found)

| Engine | Part Name | Default Diameter (m) | Source |
|---|---|---|---|
| J-20 Juno | `miniJetEngine` | 0.625 | TweakScale patch |
| J-33 Wheesley | `JetEngine` | 1.25 | TweakScale patch |
| J-90 Goliath | `turboFanSize2` | 2.5 | TweakScale patch |
| J-404 Panther | `turboJet` | 1.25 | TweakScale patch |
| J-X4 Whiplash | `turboFanEngine` | 1.25 | TweakScale patch |
| RAPIER | `RAPIER` | 1.25 | TweakScale patch |
| NFA Turbojet 2.5m | `nfa-turbojet-25-1` | 2.5 | node_stack size 2 |
| NFA Turbofan 2.5m | `nfa-turbofan-25-1` | 2.5 | node_stack size 2 |
| NFA Turbofan V2 2.5m | `nfa-turbofan-25-2` | 2.5 | node_stack size 2 |
| NFA Turboprop 1.25m | `nfa-turboprop-125-1` | 1.25 | node_stack size 1 |
| NFA Propfan 1.25m | `nfa-propfan-125-1` | 1.25 | node_stack size 1 |
| B9 D-30F7 Turbojet | `B9_Engine_Jet_Turbojet` | 1.25 | node_stack size 1 (implicit) |
| B9 TFE731 Pod Small | `B9_Engine_Jet_Pod_Small` | 1.25 | surface-attach; visual model estimate |
| B9 F119 Turbofan | `B9_Engine_Jet_Turbofan_F119` | 1.25 | node_stack size 1 (implicit) |
| NeistAir CF34-8C | `CF34-8C` | 1.25 | TweakScale MODULE in CFG |
| AirplanePlus J-56 Lotus | `cfm56` | 1.25 | TweakScaleCompanion patch |
| AirplanePlus J-34 Chevron | `cf34` | 1.25 | TweakScaleCompanion (type=free; estimated) |
| AirplanePlus F5/Tiger | `f5jet` | 0.625 | TweakScaleCompanion patch |
| AirplanePlus Raptor | `raptorjet` | 1.25 | TweakScaleCompanion patch |
| OPT Mk2 Short | `opt_mk2_engine_short` | N/A | Non-circular Mk2 cross-section; scaling laws do not apply |

**OPT Mk2 note:** The Mk2 engine uses an irregular (lift-body) cross-section.
TweakScale can still scale it, but diameter-based power laws are not meaningful
for this part. Treat it as a fixed-size engine or derive empirical scale laws
separately with Mk2-shaped test vessels.

---

## 9. Notes and Caveats

- **TweakScale:** If an engine is scaled, `maxThrust` scales with area (roughly `D²`) but the
  demand curves shape is unchanged. The IFC model (`engine_scale_agnostic_models_from_notebook.json`)
  handles this separately via diameter-based gain laws.
- **Built-in intakes:** Goliath, Turboprop, Propfan, TFE731, CF34-8C, J-56, and J-34 all
  include an internal `ModuleResourceIntake`. Their supply must be added to external intake
  supply when computing the ship total.
- **machCurve interpolation:** KSP uses Hermite splines (not linear) between keys.
  For kOS evaluation, a piecewise linear approximation introduces < 5% error across
  most of the Mach range; a proper Hermite evaluator eliminates this.
- **Shock Cone efficiency = 1.0 flat:** Despite low `intakeSpeed` (5), at Mach 3+ the
  shock cone is the only reliable subsonic intake substitute. Its flat efficiency
  curve combined with ram pressure makes it the preferred supersonic intake.
- **Firespitter and OPT_Legacy:** No IntakeAir-consuming engines were found in
  either mod directory. Firespitter piston/radial engines use a separate internal
  air model and do not expose `CONSUMEDRESOURCES["IntakeAir"]` to kOS.
