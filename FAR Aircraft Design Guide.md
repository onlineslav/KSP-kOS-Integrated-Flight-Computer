# FAR Aircraft Design Guide

A practical guide to designing, flight-testing, and tweaking aircraft in Kerbal Space Program with Ferram Aerospace Research (FAR). The goal is a stable, controllable aircraft that performs well across its intended flight envelope.

**Abbreviations used in this guide:**
- **FAR** — Ferram Aerospace Research (the aerodynamics mod)
- **CoM** — Centre of Mass — the point where the aircraft's total weight acts
- **CoL** — Centre of Lift — the point where the aircraft's total aerodynamic lift acts
- **AoA** — Angle of Attack — the angle between the wing and the oncoming airflow
- **IAS** — Indicated Airspeed — the speed shown on the airspeed indicator (in m/s)
- **FPA** — Flight Path Angle — the angle of climb or descent relative to horizontal (positive = climbing)
- **SPH** — Space Plane Hangar — the KSP editor for aircraft
- **SAS** — Stability Augmentation System — KSP's built-in autopilot/stability system
- **FBW** — Fly-By-Wire — a computer that mediates pilot inputs to prevent dangerous manoeuvres

---

## Contents

1. [FAR's Analysis Tools](#1-fars-analysis-tools)
2. [Static Stability — CoM, CoL, and Static Margin](#2-static-stability--com-col-and-static-margin)
3. [Control Surface Sizing and Authority](#3-control-surface-sizing-and-authority)
4. [Stall Characteristics](#4-stall-characteristics)
5. [Low-Speed Performance and Flaps](#5-low-speed-performance-and-flaps)
6. [Takeoff and Landing Performance](#6-takeoff-and-landing-performance)
7. [Climb and Cruise Performance](#7-climb-and-cruise-performance)
8. [High-Speed Behaviour and Compressibility](#8-high-speed-behaviour-and-compressibility)
9. [Dynamic Stability](#9-dynamic-stability)
10. [Common Problems and Fixes](#10-common-problems-and-fixes)
11. [Design Iteration Process](#11-design-iteration-process)

---

## 1. FAR's Analysis Tools

Before you fly, use FAR's in-editor analysis windows to catch problems on the ground. Open them by clicking the FAR button (the small wing icon) in the SPH toolbar.

### The Analysis Tabs

**Static Analysis tab**

This tab lets you sweep through angles of attack at a chosen speed and altitude, and shows how lift, drag, and the pitching moment change. The most important thing to check here is the shape of the pitching moment curve — it should slope downward as AoA increases, meaning the aircraft naturally pitches the nose back down if it gets pushed up too far. An upward-sloping pitching moment curve means the aircraft is unstable in pitch and will diverge if not corrected.

**Data and Graphs tab**

Shows lift and drag curves across a range of speeds or angles of attack. Use this to find:
- The maximum lift coefficient (how much lift the wing generates before stalling)
- The AoA at which the wing stalls
- The drag bucket — the speed range where drag is lowest, which is your ideal cruise region

**Stability Derivatives tab**

Shows a set of numbers that describe how the aircraft responds to disturbances. These are covered in detail in the [Dynamic Stability section](#9-dynamic-stability). For now, use this tab as a quick sanity check — the signs of the derivatives tell you whether the aircraft is naturally stable or unstable in each axis.

**Aero Forces overlay (in-flight)**

During a flight, the FAR overlay can display aerodynamic force arrows on each part. This is useful for diagnosing asymmetric loading — if the lift arrows are noticeably uneven between left and right, there is a build symmetry issue.

---

## 2. Static Stability — CoM, CoL, and Static Margin

This section covers the single most important aspect of making an aircraft fly predictably: the relationship between where the weight acts and where the lift acts.

### The Core Concept

When an aircraft is disturbed — say a gust pushes the nose up — a stable aircraft will naturally pitch back to its trimmed attitude on its own. An unstable aircraft will continue to diverge, rotating further and further until it departs controlled flight.

Whether an aircraft is stable or unstable depends on the relative positions of two points:

- **Centre of Mass (CoM):** The point through which the aircraft's total weight acts. This is shown as the yellow sphere in the SPH editor.
- **Neutral Point:** The point through which the total aerodynamic lift acts when the aircraft is disturbed. FAR uses the term "Centre of Lift (CoL)" for this, shown as the blue sphere. The neutral point is determined entirely by the shape of the aircraft — wing size, position, and tail size.

For the aircraft to be stable, **the Centre of Mass must be ahead of the Neutral Point.** If the CoM is behind the neutral point, any disturbance will cause the aircraft to pitch further away from trim rather than return to it.

```
STABLE:
← nose                                          tail →
    [CoM]──────────────────────────[Neutral Point]
           ← this gap is Static Margin →

UNSTABLE:
← nose                                          tail →
    [Neutral Point]────────────────[CoM]
           (CoM is behind NP — will diverge)
```

### Static Margin

FAR expresses the gap between CoM and the neutral point as a percentage of the wing's mean aerodynamic chord (MAC), which is essentially the average width of the wing from front to back. This percentage is called the **static margin**.

- **Positive static margin** = stable. The larger the value, the more stable (and more sluggish) the aircraft.
- **Negative static margin** = unstable. The aircraft will diverge unless FBW keeps it corrected.
- **Target range for a conventional well-flying aircraft: +5% to +15% of MAC.**
  - Below +3%: the aircraft is very sensitive and hard to hand-fly. SAS or FBW is practically required.
  - Above +20%: the aircraft is very stable but resists pitch inputs strongly. Landing can be difficult because raising the nose takes a lot of elevator deflection.

FAR displays the current static margin as a number in the SPH analysis panel. Check it there.

**In the SPH:** To increase static margin (make it more stable), move the wing assembly slightly rearward — this moves the neutral point further back, increasing the gap. To reduce static margin (make it more agile), move the wing forward.

### Centre of Mass Shifts as Fuel Burns

As fuel burns off, the CoM moves. If fuel tanks are concentrated in the nose or tail, the CoM can shift far enough to change the aircraft from stable to unstable (or vice versa) mid-flight.

**Always check static margin with both full tanks and empty tanks.** The aircraft must remain within an acceptable range in both states. A shift from +10% to +5% is fine. A shift from +8% to −3% means the aircraft will become uncontrollable as fuel runs out.

**Best practice for fuel placement:**
- Place the main fuel tanks close to and evenly distributed around the CoM.
- Avoid putting large fuel masses far forward or far aft — they will pull the CoM with them as they drain.
- If the shift is unavoidable (e.g., a long fuselage with tanks spread along it), use KSP's fuel transfer system to actively manage CoM during flight.

### Setting Up the Pitch Stability in the SPH

1. Open the SPH. Enable the CoM and CoL indicators (the sphere icons in the toolbar).
2. The blue CoL sphere should sit slightly behind the yellow CoM sphere.
3. If the CoL is ahead of CoM, the aircraft will pitch up uncontrollably. To fix this:
   - Move the wing assembly rearward, or
   - Add a larger horizontal stabiliser on the tail, or
   - Both.
4. Verify FAR's static margin readout is in the +5% to +15% range.
5. Drain the fuel tanks (use the resource editor or set sliders to zero), then check again.

### Horizontal Tail Size

The horizontal tail (or horizontal stabiliser) serves two purposes: it contributes to the neutral point position (moving it aft), and it provides the elevator authority needed to pitch the aircraft.

A tail that is too small leaves the neutral point too far forward (making the aircraft unstable or requiring large static margin via other means), and also means you may not have enough elevator to raise the nose at slow speeds.

A tail that is very large makes the aircraft very stable and hard to rotate on takeoff. This is acceptable for trainers but undesirable for anything that needs manoeuvrability.

**Rule of thumb:** The horizontal tail area multiplied by the distance from the wing to the tail, all divided by the wing area multiplied by the wing chord length, should be between 0.35 and 0.50 for most subsonic jets. You do not need to calculate this exactly — just use it as a sense check. If the tail looks proportionally small compared to the wing, it probably is.

### Vertical Tail Size

The vertical tail (fin and rudder) determines how strongly the aircraft yaws back into alignment when it is pushed sideways (directional stability, also called weathercock stability).

A fin that is too small produces weak directional stability and makes the aircraft prone to a side-to-side snaking or "Dutch roll" motion. A fin that is too large makes the aircraft overly resistant to yaw inputs and can cause the aircraft to roll when the rudder is applied.

**Rule of thumb:** Fin area multiplied by distance from the wing to the fin, divided by wing area multiplied by wing span, should be between 0.04 and 0.07 for a subsonic aircraft. Again, use this as a sanity check rather than a precise target.

### Wing Incidence Angle

Wing incidence is the angle at which the wing is mounted relative to the fuselage centreline. A small positive incidence angle (root leading edge angled slightly upward, typically 1–3 degrees) means the wing generates its cruise lift at a lower fuselage angle of attack. This keeps the fuselage more level in cruise, which reduces fuselage drag and keeps the nose pointing more horizontally (better visibility and less drag).

Do not use too much incidence — if the wing is steeply angled relative to the fuselage, it will stall at a lower fuselage angle of attack, which reduces the aircraft's overall stall AoA and makes the aircraft harder to handle near the stall.

---

## 3. Control Surface Sizing and Authority

Each control surface (elevator, ailerons, rudder) must be large enough to give the pilot adequate control throughout the flight envelope, but not so large that small inputs produce violent reactions.

### Elevator

The elevator controls pitch. It must be strong enough to:
- Raise the nose at the rotation speed on takeoff (overcome the aircraft's inertia and rotate it to the liftoff attitude)
- Arrest the descent during the flare manoeuvre on landing
- Hold the aircraft at its trimmed pitch attitude across all speeds without requiring full deflection

**How to check in the SPH:** In FAR's Static Analysis tab, set the speed to your slowest expected speed (approach speed) and sweep through angles of attack. Look at the pitching moment graph — the elevator must be able to bring the pitching moment to zero somewhere within its travel range. If the graph never reaches zero with maximum elevator deflection, your elevator is too small.

**How to check in flight:** On approach, if you find you need near-full back-stick to hold the nose up, the elevator (or the entire horizontal tail) needs to be larger, or moved further aft to increase its leverage.

**Fixing a weak elevator:**
- Increase the elevator's chord (front-to-back length) or span (tip-to-tip length)
- Move the entire horizontal tail further aft on the fuselage — this increases the distance from the wing, which means a smaller force produces a larger pitching moment

### Ailerons

Ailerons control roll. They must produce a usable roll rate at approach speed (the slowest speed where manoeuvrability is needed) and at cruise speed.

**How to test in flight:** At your approach speed with full flaps, apply full aileron input. Count how many degrees of bank are achieved per second. A roll rate below about 5 degrees per second at approach speed is uncomfortably slow even for a large airliner. A rate below 15 degrees per second may feel sluggish for a smaller jet.

**Fixing slow roll rate:**
- Extend the ailerons further toward the wingtip — the further from the centreline, the longer the lever arm and the more rolling moment per unit of force
- Increase the aileron chord
- Do not place ailerons too close to the wing root — they are much less effective there and increase induced drag significantly

### Rudder

The rudder controls yaw. It must be strong enough to:
- Counteract the yawing moment from an engine failure on a multi-engine aircraft (so one engine can fail and the aircraft can still fly straight)
- Coordinate turns (prevent the nose from slipping outward in a banked turn)
- Hold the nose straight during the takeoff ground roll, where crosswinds and asymmetric rolling resistance can push the aircraft off the centreline

**Fixing a weak rudder:**
- Increase the fin and rudder area
- Extend the fuselage aft of the CoM — this increases the distance from the fin to the CoM, giving the same fin area more leverage

### Control Surface Deflection Limits and Axis Authority

FAR lets you configure each control surface individually. Right-click a surface and look for FAR's settings in the part action window:

- **Maximum deflection angle:** The furthest the surface will travel. Set this to the largest angle at which FAR does not report flow separation (the airflow detaching from the surface). Separated flow on a control surface means it loses effectiveness and adds a lot of drag.
- **Pitch / Roll / Yaw authority sliders:** These let you scale how much each axis uses a given surface. For example, if your ailerons are causing the aircraft to yaw in the wrong direction when you roll (called adverse yaw), you can reduce the yaw authority of the ailerons to zero, or even give them a small negative yaw authority to counteract it.

---

## 4. Stall Characteristics

The stall is the point where the wing exceeds its maximum angle of attack and lift collapses. Every aircraft stalls at some speed — the goal is to make sure the stall happens at a low speed, gives the pilot clear warning, and is easy to recover from.

### How to Test Stall Speed

Perform all stall tests at above 3,000 metres altitude so you have plenty of room to recover.

**Clean stall (no flaps, gear up):**
1. Level flight at a safe altitude. Reduce throttle to idle.
2. Maintain wings level. Pull back gradually on the stick to slow down — reduce speed at about 1 m/s per second. This is a slow, deliberate deceleration, not a sudden pull.
3. Note the IAS when the aircraft begins to buffet (shake) or FAR's overlay shows the stall warning. This is the buffet onset speed — ideally it should occur 5–10 m/s above the actual stall.
4. Continue decelerating until the aircraft stalls fully (the nose drops or a wing drops). Note that IAS. This is the clean stall speed.
5. Note the angle of attack (AoA) displayed by FAR at the moment of the stall. This value — the AoA at which the wing stalls — should be entered as `a_crit` in the IFC aircraft config. It is the critical AoA that the IFC uses for stall protection.

**Dirty stall (full flaps, gear down — landing configuration):**
1. Extend full flaps and lower the landing gear.
2. Repeat the same deceleration procedure.
3. Record the IAS at the stall break. This is `vs0` in the IFC aircraft config — the stall speed in landing configuration.

### What Good Stall Behaviour Looks Like

A well-designed stall should:
- Give noticeable buffet (shaking/vibration) at least 5 m/s before the stall break, giving the pilot time to react
- Break straight ahead — both wings stall at the same time, so the nose drops but the aircraft does not roll
- Recover promptly when the pilot pushes the nose down and adds power
- Not "lock in" to a deeper stall (sometimes called a deep stall or super-stall) from which recovery is impossible

### Fixing Bad Stall Behaviour

**Wing drops at the stall (one wing stalls before the other):**

The most common cause is the wingtip stalling before the wing root. The tip generates most of the aileron authority, so once the tip stalls the ailerons lose effectiveness and the aircraft rolls toward the stalled tip.

Fix: Add **washout** — twist the wingtip slightly so its leading edge is angled lower than the root's leading edge, typically 2–4 degrees. This means the root reaches the critical AoA and stalls first, leaving the tips flying and the ailerons effective. In KSP, apply this by selecting the wingtip section and rotating it slightly nose-down relative to the root.

Alternatively, move the ailerons more inboard, away from the tip.

**T-tail deep stall (aircraft pitches up hard at the stall and cannot recover):**

On aircraft where the horizontal stabiliser is mounted at the top of the vertical tail (a T-tail arrangement), the stabiliser sits in the stalled wake of the wing at very high angles of attack. It loses effectiveness just when it is most needed, causing the aircraft to pitch up further and lock into an irrecoverable attitude.

Fix: Avoid the T-tail configuration for aircraft that need good stall recovery. Use a conventional tail (stabiliser mounted low on the rear fuselage) or a cruciform tail (stabiliser at the mid-height of the fin).

**Stall is very abrupt with no warning:**

Usually caused by a thin, sharp wing leading edge that causes the airflow to separate suddenly rather than progressively.

Fix: Use a slightly rounder leading edge profile, add a small amount of wing camber, or add leading-edge slats (if available in your parts pack).

**Very high stall speed:**

The aircraft needs too much speed to generate enough lift for level flight.

Fix: Increase wing area (more wing generates more lift for a given speed), improve the flap design to generate more low-speed lift, or reduce the aircraft's weight.

---

## 5. Low-Speed Performance and Flaps

### What Flaps Do

Flaps are movable surfaces on the trailing edge (rear) of the wing that change the wing's camber — the curvature of its cross-section. More camber generates more lift at a given angle of attack and speed, which allows the aircraft to fly at slower speeds. However, flaps also increase drag significantly at larger deflection angles.

FAR models flaps as part of the control surface system. Each "detent" (step position) in the FAR flap schedule corresponds to a different deflection angle and a different combination of lift increase and drag increase.

**What each detent should be designed for:**

| Detent | Intended use | What it should do |
|--------|-------------|-------------------|
| 0 (fully retracted) | Cruise | Wing in its cleanest shape. Minimum drag. Maximum top speed. |
| 1 (small deflection) | Takeoff and initial climb | A small increase in lift with a modest increase in drag. Allows a lower rotation speed. |
| 2 (medium deflection) | Descent and approach | More lift and noticeably more drag. The extra drag helps the aircraft slow down on approach without needing to reduce power to idle. |
| 3 (large deflection) | Short final and landing | Maximum camber, maximum lift, high drag. Allows the slowest possible approach speed. |

### Determining Maximum Flap Speed (Vfe)

Each flap detent has a maximum speed above which the structural loads on the flap become too high. This is called Vfe (velocity, flaps extended) for that detent.

To find Vfe for each detent:
1. Set the aircraft to that flap detent.
2. Climb to a safe altitude and level off.
3. Accelerate slowly — about 2–3 m/s per second.
4. Watch FAR's structural integrity indicator. Note the IAS at which FAR first shows any structural stress, or where the drag increase becomes extreme.
5. Set Vfe for that detent 5–10 m/s below that speed, as a safety margin.
6. Enter this value as `vfe_climb`, `vfe_approach`, or `vfe_landing` in the IFC config.

### Flap-Induced Pitching Moment

Extending flaps usually causes a nose-down pitch. This happens because flaps shift the Centre of Lift rearward and also create a downward pitching moment directly. The elevator must be able to counteract this and hold the nose up.

**Test:** Extend flaps in steps from fully retracted to fully extended while flying at a steady speed. Watch for any sudden, violent pitch change. A gentle, progressive nose-down pitch that you can easily hold with back-stick is acceptable. A sudden lurch that is difficult to catch is not.

**Fix if the pitch change is too large:**
- Reduce the maximum deflection angle of the flaps in FAR's settings — this reduces both the lift increase and the pitching moment
- Use a smaller flap (shorter chord) spread over more of the wing span, rather than a large deep flap
- Add a small variable-incidence canard or trim surface that automatically compensates when flaps are extended

---

## 6. Takeoff and Landing Performance

### Takeoff Roll

The ground roll from brake release to liftoff should be smooth and straight, with the aircraft accelerating progressively and lifting off cleanly at the rotate speed.

**Ground roll is too long (or aircraft will not accelerate enough):**
- Increase thrust by adding more powerful engines or additional engines
- Reduce the aircraft's weight
- Angle the wing with a small positive incidence (see [Wing Incidence](#wing-incidence-angle)) so the wing generates more lift at the nose-down attitude on the runway
- Extend the takeoff flap detent earlier to get extra lift at lower speed

**The nose will not lift at rotation speed:**

The elevator is not generating enough nose-up moment to rotate the aircraft around the main landing gear. There are a few possible causes:

- The elevator is too small — see the [Elevator section](#elevator)
- The main landing gear is positioned too far aft of the CoM. The mains act as a pivot point; if the CoM is behind them, the tail will go down and the nose will go up easily. If the CoM is far forward of the mains, it takes a very large elevator force to rotate. Move the main gear slightly rearward (closer to or just behind the CoM).
- The wing has so much positive incidence that the wing is already generating full cruise lift before the nose lifts, making the aircraft "fly itself off" rather than rotate. Reduce wing incidence slightly.

**Tailstrike on rotation (tail hits the runway):**

The aircraft pitches up too fast or too far during rotation.
- Reduce the target rotation pitch angle (`takeoff_pitch_tgt` in the IFC config)
- Reduce the rotation rate (`takeoff_pitch_slew_dps` in the IFC config) so the pitch increases more slowly
- If the geometry is the problem, taller rear landing gear raises the tail and provides more rotation clearance
- Moving the main gear further forward gives a greater angle of rotation before the tail contacts the ground

**Nosewheel lifts off before the intended rotation speed:**

The nose rises spontaneously before the pilot inputs rotation. This means the balance of forces is tipping the nose up on its own. Usually caused by the nosewheel being too far forward of the CoM — the wing lift at the rear is creating a torque that lifts the front.

Fix: Move the main landing gear slightly rearward, or reduce wing incidence.

### Landing Flare and Touchdown

The flare is the manoeuvre just before touchdown where the pilot raises the nose to reduce the descent rate. The aircraft should settle smoothly onto the runway with a gentle sink rate.

**Aircraft floats down the runway and will not touch down:**

The aircraft has too much energy (excess speed and/or excess lift from ground effect) after the flare.

- Reduce the approach speed (`v_app` in the IFC config) — even a small reduction significantly reduces the energy that needs to be dissipated
- Increase `flare_ias_to_vs_gain` in the IFC config — this causes the autothrottle to allow a slightly larger sink rate when the aircraft is fast, bleeding energy
- Deploy spoilers or airbrakes earlier in the approach to add drag and allow a steeper descent profile
- If `flare_agl` (the height at which the flare begins) is too high, the aircraft levels off with a large altitude buffer and then floats for a long time. Lower it slightly.

**Aircraft drops in hard (high sink rate at touchdown):**

The flare is not arresting the descent rate in time.

- `flare_agl` is too low — the aircraft does not have enough height for the pitch-up to take effect before the wheels hit. Raise it.
- `flare_pitch_rate_min` and `flare_pitch_rate_max` are too low — the flare is happening too slowly. Increase these values so the IFC pitches up more aggressively.
- The approach speed is too low, meaning the aircraft arrives with a high sink rate (near-stall descent). Increase `v_app`.

**Aircraft bounces after touchdown:**

The mains contact the runway, compress, then spring back up.

- Reduce the landing gear spring rate — stiff gear rebounds strongly
- The touchdown sink rate was too high — refine the flare
- The IFC's bounce recovery parameters (`bounce_recovery_agl_m`, `bounce_recovery_min_vs`) can be tuned to detect and correct bounces automatically

### Ground Effect

FAR models ground effect — the increase in lift efficiency that occurs when the aircraft is flying very close to the ground (within roughly one wing span of the surface). In ground effect, the wing generates more lift for the same speed than it does at altitude. This causes the aircraft to float further than expected in the last few metres of the approach.

When setting `flare_agl`, be aware that the effective lift increases noticeably below about 15 metres AGL. The aircraft will feel like it wants to float and may need less back-pressure than at altitude. Factor this in when evaluating flare height.

---

## 7. Climb and Cruise Performance

### Climb Rate

An aircraft climbs when its engines produce more thrust than is needed to overcome drag at the current speed. The excess thrust accelerates the aircraft upward.

The faster you climb, the more power you need. If the aircraft is underpowered, you must fly a shallower climb angle to maintain speed — you gain altitude more slowly but do not bleed off speed.

**To increase climb rate:**
- Add more thrust (more powerful or additional engines)
- Reduce weight
- Fly at the speed that gives the best ratio of lift to drag — this speed gives the most efficient use of the available thrust. In FAR's Data and Graphs tab, look at the graph of lift-to-drag ratio versus speed (or Mach number). The peak of that curve is the best efficiency speed for level flight, and it is close to the best climb speed.
- If the aircraft cannot maintain speed on a steep climb, reduce the flight path angle (`takeoff_climb_fpa` in the IFC config) until speed stabilises

**Best climb speed as a rough guide:**
- Jet aircraft flying subsonically: typically around 100–160 m/s IAS, staying below the speed where the drag starts rising steeply (the drag divergence region)

### Cruise Speed and Drag

The aircraft will reach its maximum cruise speed when thrust equals drag — it cannot go faster because all the thrust is being used to overcome drag. To fly faster, reduce drag or increase thrust.

**FAR's Data and Graphs tab** shows a drag coefficient (how much drag the aircraft produces per unit of size) versus Mach number plot. Look for the "drag bucket" — the range of Mach numbers where the drag coefficient is low and relatively flat. This is the efficient cruise region. Avoid cruising in the steep upward slope of the drag curve.

**Reducing drag:**
- Reduce the frontal area of the aircraft — a thinner, narrower fuselage produces less pressure drag
- Fill in gaps between parts with nosecones and fuselage adapters — exposed steps and gaps produce significant interference drag
- For aircraft flying above about Mach 0.8, area-rule the fuselage (see the [High-Speed section](#8-high-speed-behaviour-and-compressibility))
- Retract all landing gear and close all cargo bay doors before cruise

### Lift-to-Drag Ratio

The lift-to-drag ratio (often written L/D) tells you how efficiently the aircraft generates lift compared to the drag it produces. A higher L/D means the aircraft uses less thrust to fly level, gives a shallower glide when unpowered, and has longer range.

**Measuring L/D in flight:**
1. Establish straight and level flight at cruise speed. Note the throttle setting.
2. Reduce throttle to idle (zero thrust).
3. Maintain the same attitude and let the aircraft descend in a steady glide.
4. Read the flight path angle from the IFC telemetry or the navball pitch indicator.
5. The L/D ratio equals 1 divided by the tangent of the glide angle. As a rough guide: a 5-degree glide gives L/D of about 11; a 3-degree glide gives L/D of about 19.

**Improving L/D:**
- Increase the wing's aspect ratio (make it longer and narrower relative to its area) — high-aspect-ratio wings are much more efficient at producing lift for a given drag
- Add winglets to the wingtips — they reduce the drag caused by air spilling from the high-pressure underside of the wing to the low-pressure top (induced drag)
- Reduce the wing's thickness (thinner wings have lower drag at cruise speed)

---

## 8. High-Speed Behaviour and Compressibility

When an aircraft approaches the speed of sound, the airflow around the wings and fuselage reaches supersonic speeds locally even before the aircraft itself is supersonic. This causes a series of effects collectively called compressibility effects.

### Drag Divergence

As speed increases toward Mach 1, shockwaves begin to form on the aircraft. These shockwaves cause a large, rapid increase in drag. The speed at which this drag rise becomes steep is called the **drag divergence Mach number**. You can see this clearly in FAR's drag-versus-Mach plot as the point where the drag coefficient curve starts rising sharply.

Flying faster than the drag divergence Mach number requires a large increase in thrust for a relatively small increase in speed — it becomes very expensive in terms of fuel.

**Raising the drag divergence Mach number** (making the aircraft capable of higher efficient cruise speed):
- Sweep the wings backward — a swept wing effectively presents a smaller cross-section to the airflow, delaying the onset of the shockwave
- Use thinner wing sections (reduce the wing's thickness-to-chord ratio)
- **Area-rule the fuselage:** The total cross-sectional area of the aircraft (fuselage + wings + everything else) should change as smoothly as possible along the length of the aircraft. At transonic speeds, the fuselage must become narrower (waisted) at the wing station to keep the total cross-section smooth. This is called the "Coke bottle" or "wasp waist" shape and dramatically reduces transonic drag.

### Mach Tuck

At speeds above roughly Mach 0.7–0.8, the centre of pressure of the wings shifts progressively rearward as the shockwave moves toward the trailing edge. This shift of lift rearward causes the aircraft to pitch nose-down — the further the pressure centre moves aft, the stronger the nose-down tendency. This is called Mach tuck.

**Symptoms:** The aircraft progressively wants to pitch nose-down as you accelerate above Mach 0.7. The faster it goes, the more forward stick (nose-up input) you need to hold level flight.

**Fixes:**
- Swept wings delay and reduce the magnitude of the centre-of-pressure shift, reducing the severity of Mach tuck
- A larger horizontal tail with more elevator range gives you more nose-up authority to trim against the tuck
- If the tuck is severe, a T-tail or high-mounted stabiliser can remain effective in clean airflow above the wing wake at high speed — but note the deep-stall risk this creates at low speed (see [Stall section](#4-stall-characteristics))

### Flutter

Aerodynamic flutter is a self-sustaining oscillation of a control surface or wing caused by the interaction between the aerodynamic forces and the structure's natural vibration frequency. Once started, it amplifies rapidly and can destroy the aircraft within seconds.

**Symptoms:** A control surface vibrates very rapidly, the aircraft yaws or rolls rapidly and uncontrollably at high speed.

**Prevention:**
- Use structurally stiffer wing and control surface parts (if using B9 Procedural Wings, choose a higher-strength material option)
- Keep control surfaces relatively short in span and chord on high-speed aircraft — smaller surfaces have higher flutter speeds
- Ensure the aircraft's maximum speed in the IFC config (`vfe_*` values and cruise speeds) stays below FAR's reported flutter speed for the aircraft

### Supersonic Static Margin Shift

When the aircraft goes supersonic, the neutral point shifts significantly further aft. This means a supersonic aircraft automatically becomes more stable at supersonic speeds than it was at subsonic speeds.

This has a design implication: if you design the aircraft for a modest static margin subsonically (say +8%), the supersonic static margin may jump to +15–20%, making the aircraft feel much heavier and more sluggish in pitch at supersonic speed. This is generally acceptable.

However, if you design for a very small subsonic static margin to get agile subsonic handling, the supersonic shift may result in an extremely sluggish aircraft that cannot be manoeuvred supersonically without a very powerful FBW system.

**Recommended approach for supersonic aircraft:** aim for +8% to +12% static margin at Mach 0.9. The supersonic shift will bring this to roughly +15–20%, which gives acceptable handling on both sides of Mach 1.

---

## 9. Dynamic Stability

Static stability (covered in [Section 2](#2-static-stability--com-col-and-static-margin)) describes whether the aircraft naturally returns to its trimmed attitude after a disturbance. Dynamic stability describes how it returns — does it oscillate gently and damp out, or does it oscillate with increasing amplitude until it departs?

FAR's Stability Derivatives panel shows a set of numbers (called derivatives) that characterise the aircraft's natural tendency in each axis. Each derivative has a letter name, and its sign (positive or negative) tells you whether it is stabilising or destabilising.

### The Key Stability Derivatives

**Cm_α (pitch stiffness):** How strongly the aircraft pitches the nose back when the angle of attack increases. Should be **negative** — meaning the nose pitches back down when it goes up. This is the mathematical expression of static pitch stability. If it is positive, the aircraft is statically unstable in pitch.

**Cm_q (pitch rate damping):** How strongly pitch rotation is resisted. Should be **negative** — meaning the aircraft damps out pitch oscillations rather than amplifying them. A large horizontal tail far aft of the CoM produces strong pitch damping.

**Cl_β (dihedral effect):** How the aircraft rolls when it sideslips (slides sideways through the air). Should be **negative** — meaning the upwind wing generates more lift and rolls the aircraft back toward wings-level. This is provided by wing dihedral (wings angled upward from root to tip). A high-wing configuration naturally has strong dihedral effect even with wings mounted flat.

**Cl_p (roll rate damping):** How quickly the roll rate decays when aileron input is released. Should be **negative** — the aircraft should stop rolling when the ailerons are centred.

**Cn_β (directional stability / weathercock stability):** How the aircraft yaws back into alignment when it sideslips. Should be **positive** — meaning the nose yaws back into the wind. This is provided by the vertical fin. A larger fin further aft produces stronger directional stability.

**Cn_r (yaw rate damping):** How quickly yaw rotation damps out. Should be **negative**.

**Cn_δa (aileron adverse yaw):** How much yaw is generated by aileron deflection. Ideally this should be a small negative number — meaning ailerons generate a small amount of adverse yaw (yaw in the opposite direction to the roll). A large adverse yaw value means the aircraft yaws strongly opposite to the roll, requiring a lot of coordinating rudder input.

### The Phugoid (Long, Slow Pitch Oscillation)

The phugoid is a very slow, large-amplitude pitch oscillation that occurs when the aircraft is flying hands-off. It has a period of anywhere from 10 to 60 seconds. It looks like the aircraft gently pitching nose-up and climbing, then gradually pitching nose-down and descending, alternating slowly.

A mildly unstable phugoid (where the oscillation slowly grows) is generally acceptable in practice because the pilot naturally damps it out with small stick inputs every half-cycle. A very strongly divergent phugoid is uncomfortable to live with.

**If the phugoid is noticeably divergent:** increase the static margin slightly by moving the wing rearward.

### The Short-Period Oscillation (Rapid Pitch Bobbing)

The short-period mode is a fast pitch oscillation with a period of 1–3 seconds. Unlike the phugoid, it must be well-damped — if it is not, the aircraft will feel like it bobbles up and down after every pitch input, which is very unpleasant and can degrade tracking accuracy.

**Too lightly damped (aircraft bobbles for several cycles after a pitch input):** Enlarge the horizontal tail, or move it further aft on the fuselage. Both increase pitch damping.

**Too heavily damped (aircraft feels extremely sluggish in pitch, very slow to respond to elevator):** Reduce the tail area slightly, or reduce the static margin.

### Dutch Roll

Dutch roll is a coupled oscillation involving both rolling and yawing simultaneously. When the aircraft yaws to the right, the dihedral effect causes the left wing (now the leading wing, seeing more airflow) to rise, banking the aircraft to the right. Then the directional stability yaws the nose back left, and the cycle repeats. The result is a side-to-side waggling motion.

**Symptoms:** The aircraft wags its tail slowly from side to side, with each yaw accompanied by a roll in the same direction.

**Root cause:** The directional stability (Cn_β — fin size) is too weak relative to the dihedral effect (Cl_β). The yaw restoring force is too slow, so the roll response outpaces it and a sustained oscillation develops.

**Fixes:**
- Increase the fin size — this increases Cn_β and closes the gap between yaw and roll response speeds
- Reduce the wing dihedral — this reduces Cl_β, which counterintuitively helps by slowing down the roll response to match the yaw response
- Enable the SAS yaw damper — a yaw damper applies small automatic rudder inputs to damp out the oscillation electronically, which is the real-world solution for airliners

### Spiral Instability

Spiral instability is the opposite of Dutch roll. The aircraft has too little roll restoring force — when banked slightly, instead of rolling back toward wings-level, the bank angle slowly but continuously steepens. Left unchecked, the aircraft gradually enters a tightening spiral dive.

**Symptoms:** In a hands-off banked attitude, the bank angle slowly increases. The nose drops slightly and the aircraft begins to descend in a tightening turn.

**Root cause:** The dihedral effect (Cl_β) is too weak. The fin is strong (providing good directional stability) but the wings do not naturally roll back to level.

**Fixes:**
- Add dihedral to the wings
- Lower the wing mounting position — a low-wing configuration has less natural dihedral effect than a high-wing configuration, so a low-wing aircraft typically needs more geometric dihedral to achieve the same stability

Note: a mild spiral instability is very common in real aircraft and most pilots consider it acceptable since a slow divergence is easy to correct.

### Roll Coupling (Inertia Coupling)

At high angles of attack combined with high roll rates, the aircraft's rotational inertia can cause the rolling motion to generate unexpected pitch and yaw moments. This can lead to a sudden departure from controlled flight — the aircraft suddenly pitches or yaws violently during a fast roll.

This is mostly relevant to swept-wing fighters and high-performance aircraft being rolled rapidly at high AoA.

**Fixes:**
- Limit the maximum roll rate at high AoA using the FBW system
- Reduce the wing sweep angle
- Increase the fin size to improve yaw stiffness

---

## 10. Common Problems and Fixes

A quick-reference guide to the most frequent issues found during test flights.

### Pitch Problems

| Symptom | Most Likely Cause | Fix |
|---------|------------------|-----|
| Aircraft pitches nose-up uncontrollably at all speeds | Centre of Mass is too far forward; wing is generating too much nose-up moment | Move the CoM rearward by removing forward ballast, or reduce wing incidence angle |
| Aircraft pitches nose-down as speed increases | Mach tuck at high speed; or the horizontal tail is generating too much downforce | Sweep the wings back to reduce Mach tuck; check tail incidence is not creating excessive downforce |
| Cannot raise the nose at rotation speed | Elevator too small; or main gear too far forward of the CoM | Enlarge the elevator or move it further aft; move the main gear slightly rearward toward the CoM |
| Aircraft pitches up suddenly when flaps are extended | Flap extension is shifting the Centre of Lift forward | Move the flaps more inboard; reduce the maximum flap deflection angle |
| Aircraft oscillates nose-up/nose-down after stick input | Short-period oscillation is not well-damped | Enlarge the horizontal tail or move it further aft |
| Aircraft is very reluctant to pitch at all; elevator feels ineffective | Static margin is too large | Reduce tail size, or move the wing slightly rearward |

### Roll Problems

| Symptom | Most Likely Cause | Fix |
|---------|------------------|-----|
| Roll rate is very slow; bank angle changes sluggishly | Ailerons are too small or positioned too close to the wing root | Extend the ailerons further toward the wingtip; increase aileron chord |
| Aircraft rolls when rudder is applied (without aileron input) | Dihedral effect is too strong — sideslip caused by the rudder is creating a large roll | Reduce wing dihedral; lower the wing mounting position |
| One wing drops at the stall | Wingtip stalls before the root, creating an asymmetric lift loss | Add washout (twist the wingtip slightly nose-down relative to the root); move ailerons inboard |
| Aircraft oscillates in roll (wags side to side) | Dutch roll — see the Dynamic Stability section | Enlarge the vertical fin; reduce wing dihedral |

### Yaw Problems

| Symptom | Most Likely Cause | Fix |
|---------|------------------|-----|
| Aircraft drifts sideways in cruise without any input | Asymmetric thrust (one engine producing more than the other) or a build symmetry error | Check all engines are identical and aligned; check the aircraft for any asymmetric parts |
| Aircraft snakes left and right on the takeoff roll | Nosewheel steering or rudder authority is insufficient to hold the centreline | Increase the nosewheel steering angle limit; enlarge the rudder |
| Aircraft swerves violently on the runway at low speed | Nosewheel steering authority is too high | Reduce the nosewheel maximum steering angle |
| Aircraft yaws in the opposite direction to a roll input | Adverse yaw from the ailerons — the down-going aileron produces more drag than the up-going one | Reduce the yaw authority contribution of the ailerons in FAR's settings; add a small automatic rudder-aileron coupling |
| Aircraft hunts left and right in yaw at cruise | Directional stability is too weak (small fin) | Enlarge the vertical fin; lengthen the fuselage aft of the CoM |

### Speed and Drag Problems

| Symptom | Most Likely Cause | Fix |
|---------|------------------|-----|
| Aircraft cannot reach its design cruise speed despite adequate thrust | Too much aerodynamic drag | Fair over joints and gaps with nosecones; reduce the frontal cross-section area of the fuselage |
| Approach speed is very high even with full flaps | Wing area is insufficient to generate enough lift at slow speed | Increase wing area; redesign flaps to provide more lift at full deflection |
| Drag increases dramatically above Mach 0.8 | No area-ruling on the fuselage; thick wing section | Waist the fuselage at the wing station; reduce wing thickness |
| Engine loses power or flames out at high altitude | Air intake area is insufficient to feed the engines at low air density | Enlarge the intakes; add additional intake area; for supersonic intakes, ensure shock cones are sized correctly |

### Ground Handling Problems

| Symptom | Most Likely Cause | Fix |
|---------|------------------|-----|
| Landing gear collapses on touchdown | Gear spring or structural attachment is too weak | Increase the gear spring and damper rates; attach the gear to a structural fuselage part rather than a wing panel |
| Aircraft bounces repeatedly after touchdown | Gear spring rate is too high (gear rebounds strongly); or touchdown sink rate was too high | Reduce the gear spring rate; refine the flare to achieve a lower sink rate at touchdown |
| Aircraft veers off the centreline on rollout | Asymmetric gear loads or weak directional control | Adjust `rollout_yaw_kp` and `rollout_yaw_sign` in the IFC config to tune the rudder centreline-hold |
| Nosewheel slams down immediately after mains touch | Insufficient pitch hold applied after touchdown | Increase `rollout_nose_hold_cmd` in the IFC config to apply aft stick briefly after mains contact |

---

## 11. Design Iteration Process

Use this sequence for each new aircraft or any major redesign. Work through it in order — each stage builds on the one before.

```
STAGE 1 — EDITOR CHECKS (before first flight)
   ┣ Check CoM position relative to CoL in the SPH
   ┣ Verify FAR static margin is between +5% and +15%
   ┣ Drain tanks and re-check static margin
   ┗ Review stability derivative signs in FAR panel — all should be stable
        ↓
STAGE 2 — FIRST FLIGHT (manual control, no IFC)
   ┣ Take off and immediately check for any uncontrolled divergence
   ┣ Test pitch, roll, and yaw authority at a safe altitude
   ┗ Perform a clean stall test to verify safe stall behaviour
        ↓
STAGE 3 — PERFORMANCE CHARACTERISATION
   ┣ Clean stall speed and critical AoA → record a_crit
   ┣ Dirty stall speed (full flaps + gear) → record vs0
   ┣ Vfe determination for each flap detent
   ┣ Rotation speed test → record v_r
   ┣ Climb-out: find stable V2 and climb FPA → record v2, takeoff_climb_fpa
   ┣ Cruise performance: measure L/D glide ratio
   ┗ High-speed checks: verify behaviour up to maximum intended speed
        ↓
STAGE 4 — APPROACH AND LANDING TESTS
   ┣ Fly several manual approaches, find comfortable v_app and v_ref
   ┣ Test flare at various heights to find the right flare_agl
   ┣ Verify rollout directional control and nose-hold behaviour
   ┗ Confirm gear and braking are stable
        ↓
STAGE 5 — IFC CONFIGURATION
   ┣ Fill in all measured values in the aircraft's *_cfg.ks file
   ┣ Run IFC-assisted approaches and compare to manual results
   ┗ Refine IFC config parameters based on approach quality
        ↓
STAGE 6 — DOCUMENT
   ┗ Record vs0, a_crit, Vfe values, v_r, v2, and any quirks in the
     config "notes" field so future flights have the context
```

---

## Reference: FAR Stability Derivative Signs

| Derivative | What it represents | Stable sign | What a wrong sign means |
|------------|--------------------|-------------|------------------------|
| Cm_α | Pitch stiffness — nose pitches back when AoA increases | Negative | Aircraft is statically unstable in pitch — will diverge |
| Cm_q | Pitch rate damping — pitch oscillations die out | Negative | Pitch oscillations grow — short-period is divergent |
| Cl_β | Dihedral effect — wing rolls back to level when sideslipped | Negative | No roll restoring force — prone to spiral instability |
| Cl_p | Roll rate damping — roll stops when aileron is released | Negative | Roll oscillates after input |
| Cn_β | Directional stability — nose yaws back into the wind | Positive | Aircraft is directionally unstable — will diverge in yaw |
| Cn_r | Yaw rate damping — yaw oscillations die out | Negative | Yaw oscillations grow |
| Cn_δa | Adverse yaw from aileron deflection | Small negative | Large magnitude means poor turn coordination; requires heavy rudder use |

If any derivative has the wrong sign and the aircraft is not protected by a fly-by-wire system, it will diverge in that axis given any disturbance. Always fix the structural cause first — adjust geometry, sizing, and placement. Only rely on SAS authority as a last resort for dynamic stability, since SAS has authority limits and adds complexity.
