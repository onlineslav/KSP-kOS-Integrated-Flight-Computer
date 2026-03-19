# CLAUDE.md — kOS / IFC Project Notes

This file contains kOS-specific rules and context for working on the Integrated
Flight Computer (IFC) codebase. Read this before writing or editing any `.ks` file.

---

## Project Structure

```
Integrated Flight Computer/
  ifc_main.ks            — main loop, flight plan executor, leg queue
  lib/
    ifc_constants.ks     — all tunables and phase/subphase name constants
    ifc_state.ks         — all mutable runtime globals; reset via IFC_INIT_STATE()
    ifc_helpers.ks       — utility functions (CLAMP, MOVE_TOWARD, WRAP_180, etc.)
    ifc_autothrottle.ks  — adaptive cascade speed/throttle controller
    ifc_aa.ks            — AtmosphereAutopilot wrapper
    ifc_logger.ks        — CSV telemetry logging
    ifc_ui.ks            — terminal UI layer
    ifc_display.ks       — per-phase display rendering
    ifc_menu.ks          — menu/overlay input handling
    ifc_fms.ks           — FMS plan editor state
    ifc_gui.ks           — kOS GUI pre-arm window
  nav/
    nav_beacons.ks       — beacon/plate database and helpers
    nav_math.ks          — geodesic helpers (GEO_DESTINATION, etc.)
    nav_routes.ks        — named routes
    nav_custom_wpts.ks   — user waypoints
  phases/
    phase_takeoff.ks     — takeoff roll, rotation, climb-out
    phase_cruise.ks      — en-route cruise with waypoint nav
    phase_approach.ks    — ILS approach (FLY_TO_FIX → ILS_TRACK)
    phase_autoland.ks    — flare, touchdown, rollout
    phase_ascent.ks      — spaceplane ascent to orbit (STUB — not yet implemented)
    phase_reentry.ks     — reentry (stub)
  aircraft/
    aircraft_template.ks — canonical config template; copy for each aircraft
    *_cfg.ks             — per-aircraft configs
  logs/                  — CSV telemetry output (gitignored)
```

---

## kOS Language Rules

### @LAZYGLOBAL OFF

Every `.ks` file in this project begins with `@LAZYGLOBAL OFF.`

This means every variable must be explicitly declared with `LOCAL`, `GLOBAL`, or
`PARAMETER` before use. Never add an implicit global. Never remove this directive.

### Variable Declarations

- `GLOBAL` — used in `ifc_state.ks` for mutable runtime state; also for shared
  constants in `ifc_constants.ks`.
- `LOCAL` — used inside functions for all local temporaries.
- Never use `GLOBAL` inside a function body.

### Naming Conventions Used in This Codebase

- Constants: `UPPER_SNAKE_CASE` (e.g. `KP_LOC`, `FLARE_AGL_M`)
- Runtime globals: `UPPER_SNAKE_CASE` (e.g. `THROTTLE_CMD`, `IFC_PHASE`)
- Functions: `UPPER_SNAKE_CASE` (e.g. `RUN_APPROACH`, `IFC_SET_ALERT`)
- Local variables: `lower_snake_case` (e.g. `dt`, `pitch_err`, `q_actual`)
- Prefixes by subsystem: `AT_` autothrottle, `AA_` atmosphere autopilot,
  `ILS_` ILS tracking, `APP_` approach, `TO_` takeoff, `TELEM_` telemetry export

### Loop Discipline

The main loop in `ifc_main.ks` ends with `WAIT IFC_LOOP_DT.` (currently 0.02 s,
50 Hz). This is the `wait 0` equivalent — it yields to the physics engine.
Never use `wait 0` directly; use `WAIT IFC_LOOP_DT.` so the rate is configurable.

All timing-sensitive computations use real measured dt:

```kerboscript
LOCAL actual_dt IS TIME:SECONDS - IFC_CYCLE_UT.
SET IFC_CYCLE_UT  TO TIME:SECONDS.
SET IFC_ACTUAL_DT TO CLAMP(actual_dt, 0.01, 0.5).
```

Never hardcode a Δt constant. Always read `IFC_ACTUAL_DT`.

### Throttle and Steering Locks

Throttle is set by writing to `THROTTLE_CMD` (a regular global), and the lock
`LOCK THROTTLE TO THROTTLE_CMD.` is established once at flight start. Phase
functions never call `LOCK THROTTLE` themselves.

Steering: `LOCK STEERING TO desired_steering.` where `desired_steering` is a
global that phase functions write each cycle. Never lock steering to a complex
inline expression — compute it once per loop, store it, let the lock be trivial.

### Phase Management

Use `SET_PHASE(PHASE_*)` and `SET_SUBPHASE(SUBPHASE_*)` — never write to
`IFC_PHASE` or `IFC_SUBPHASE` directly. These functions log the transition and
reset `IFC_PHASE_START_UT`.

### Aircraft Config

Use `AC_PARAM(key, fallback, guard)` to read per-aircraft values. The guard is
the minimum acceptable value; values below guard fall back to the global constant.
Sentinel for "not set" is `-1`. Never access `ACTIVE_AIRCRAFT` lexicon keys
directly in phase code without an `AC_PARAM` call.

### FAR and AA Guards

Always check `FAR_AVAILABLE` before reading `ADDONS:FAR:*` suffixes.
Always check `AA_AVAILABLE` before calling AA functions.
`FAR_AVAILABLE` and `AA_AVAILABLE` are set in `AA_INIT()` at flight start.

---

## kOS Reserved / Protected Names

All kOS identifiers are **case-insensitive**. None of the names below may be used
as variable names, function names, or parameter names. kOS 1.4+ will throw
`"Not allowed to SET a name that will clobber or hide the variable called 'X'"`.

### Hard Language Keywords

These are rejected by the parser regardless of any settings:

```
ADD         ALL         AND         AT
BREAK       CHOOSE      CLEARSCREEN COMPILE
COPY        DECLARE     DEFINED     DELETE
DO          EDIT        ELSE        FALSE
FILE        FOR         FROM        FUNCTION
GLOBAL      IF          IN          IS
LIST        LOCAL       LOCK        LOG
NOT         OFF         ON          ONCE
OR          PARAMETER   PRESERVE    PRINT
REBOOT      REMOVE      RENAME      RETURN
RUN         RUNONCEPATH RUNPATH     SET
SHUTDOWN    STAGE       STEP        SWITCH
THEN        TO          TOGGLE      TRUE
UNLOCK      UNSET       UNTIL       VOLUME
WAIT        WHEN
```

### Protected Bound Variable Names (SHIP field aliases and system vars)

```
ALTITUDE        ANGULARMOMENTUM ANGULARVEL      ANGULARVELOCITY
APOAPSIS        AIRSPEED        BODY            FACING
GEOPOSITION     GROUNDSPEED     HEADING         LATITUDE
LONGITUDE       MASS            MAXTHRUST       NORTH
OBT             PERIAPSIS       PROGRADE        RETROGRADE
SENSORS         SHIPNAME        SRFPROGRADE     SRFRETROGRADE
STATUS          UP              VELOCITY        VERTICALSPEED
```

### Flight Control Lock Targets (cannot be used as regular variables)

```
STEERING        THROTTLE        WHEELSTEERING   WHEELTHROTTLE
```

### Vessel / Universe Globals

```
ADDONS          ALT             ALLNODES        ARCHIVE
CONFIG          CONSTANT        CONTROLCONNECTION
CORE            ETA             ENCOUNTER       HASTARGET
HASNODE         HOMECONNECTION  KUNIVERSE       LOADDISTANCE
MAPVIEW         MISSIONTIME     NEXTNODE        OPCODESLEFT
PROFILERESULT   SESSIONTIME     SHIP            SOLARPRIMEVECTOR
TARGET          TERMINAL        TIME            VERSION
WARP            WARPMODE
```

### Action Groups (Boolean get/set)

```
ABORT   AG1   AG2   AG3   AG4   AG5   AG6   AG7   AG8   AG9   AG10
BAYS    BRAKES CHUTES CHUTESSAFE DEPLOYDRILLS DRILLS FUELCELLS
GEAR    INTAKES ISRU  LADDERS LEGS  LIGHTS PANELS RADIATORS RCS SAS
```

### Resource Names

```
ELECTRICCHARGE  INTAKEAIR  LIQUIDFUEL  MONOPROPELLANT  OXIDIZER  SOLIDFUEL
```

### Color Constants

```
BLACK  BLUE  CYAN  GRAY  GREY  GREEN  MAGENTA  PURPLE  RED  WHITE  YELLOW
```

### Built-in Functions (cannot be redeclared as user functions)

**Math:**
```
ABS     ARCCOS  ARCSIN  ARCTAN  ARCTAN2 CEILING CHAR    FLOOR
LN      LOG10   MAX     MIN     MOD     RANDOM  RANDOMSEED  ROUND
SIN     SQRT    TAN     UNCHAR
```

**Vector / Direction:**
```
ANGLEAXIS  HEADING  LOOKDIRUP  Q  R  ROTATEFROMTO
V  VANG  VCRS  VDOT  VECTORANGLE  VECTORCROSSPRODUCT
VECTORDOTPRODUCT  VECTOREXCLUDE  VXCL
```

**Collections:**
```
LEXICON  LEX  LIST  QUEUE  RANGE  STACK  UNIQUESET
```

**Constructors / Prediction:**
```
HIGHLIGHT  HUDTEXT  LATLNG  NODE  ORBITAT  PATH  POSITIONAT
PROCESSOR  RGB  RGBA  HSVA  SCRIPTPATH  TIMESTAMP  TIMESPAN
VECDRAW  VECDRAWARGS  VELOCITYAT  VOLUME  WRITEJSON  READJSON
```

**File I/O:**
```
CD  COPYPATH  CREATE  CREATEDIR  DELETEPATH  EXISTS  MOVEPATH  OPEN
```

### Names That Look Safe But Are NOT

| Name | Status |
|------|--------|
| `STAGE` | PROTECTED — both keyword and built-in structure |
| `ALTITUDE` | PROTECTED — global alias for SHIP:ALTITUDE |
| `VELOCITY` | PROTECTED — global alias for SHIP:VELOCITY |
| `MASS` | PROTECTED — global alias for SHIP:MASS |
| `HEADING` | PROTECTED — global alias for SHIP:HEADING |
| `THROTTLE` | PROTECTED — flight control lock target |
| `STEERING` | PROTECTED — flight control lock target |
| `TIME` | PROTECTED — built-in TimeStamp global |
| `BODY` | PROTECTED — global alias for SHIP:BODY |
| `APOAPSIS` | PROTECTED — global alias |
| `PERIAPSIS` | PROTECTED — global alias |
| `LIQUIDFUEL` | PROTECTED — resource name binding |
| `OXIDIZER` | PROTECTED — resource name binding |
| `MAXTHRUST` | PROTECTED — global alias for SHIP:MAXTHRUST |
| `LOG` | PROTECTED — language keyword (file logging command) |
| `LIST` | PROTECTED — language keyword AND collection constructor |
| `NORTH` | PROTECTED — global alias for SHIP:NORTH |
| `UP` | PROTECTED — global alias for SHIP:UP |
| `STATUS` | PROTECTED — global alias for SHIP:STATUS |

### Names That Are Safe (Common Confusions)

| Name | Why it looks risky | Reality |
|------|-------------------|---------|
| `SPEED` | sounds built-in | safe; `AIRSPEED`/`GROUNDSPEED` are protected, `SPEED` is not |
| `THRUST` | sounds built-in | safe; `MAXTHRUST` is protected, bare `THRUST` is not |
| `FUEL` | sounds built-in | safe; `LIQUIDFUEL`/`OXIDIZER` are protected, `FUEL` is not |
| `ORBIT` | sounds built-in | safe; `OBT` is the protected alias, `ORBIT` is not |
| `ENGINES` | sounds built-in | safe; it's a Vessel suffix, not a global |
| `CONTROL` | sounds built-in | safe; it's a Vessel suffix, not a global |
| `PARTS` | sounds built-in | safe; it's a Vessel suffix, not a global |
| `CREW` | sounds built-in | safe; it's a Vessel suffix, not a global |

---

## kOS-Specific Gotchas

### Frame Alignment (critical for ascent guidance)

`ADDONS:FAR:AEROFORCE` is in the **ship body frame** — it must be rotated to the
orbital (inertial) frame before any dot product with velocity. The rotation matrix
columns from `SHIP:FACING` are:

```
[SHIP:FACING:STARVECTOR, SHIP:FACING:TOPVECTOR, -SHIP:FACING:FOREVECTOR]
```

Note the negative on FOREVECTOR. Getting this wrong produces a drag vector that
points forward — a silent sign error that corrupts all energy calculations.

`SHIP:VELOCITY:ORBIT` is already in the inertial frame. No transform needed.
`SHIP:VELOCITY:SURFACE` is in the rotating surface frame — do not use for
orbital energy calculations.

Compute the rotation matrix once per loop iteration. Never recompute it inside
individual functions within the same cycle.

### SHIP:ORBIT:APOAPSIS Edge Cases

Returns a negative value on suborbital trajectories with periapsis below the
surface. Always guard: `IF SHIP:ORBIT:APOAPSIS < 0 { ... }`.

`ETA:APOAPSIS` can return very large or discontinuous values on nearly-circular
or hyperbolic orbits. Validate before using in error terms.

### EMA Update Pattern

All exponential moving averages use the standard form:

```kerboscript
SET x_filt TO alpha * x_raw + (1 - alpha) * x_filt.
```

Higher alpha = faster response, less smoothing.
Lower alpha = slower response, more smoothing.

Valuation signals (mode comparison): low alpha (0.10–0.20).
Control signals (pitch/throttle commands): higher alpha (0.30–0.50).
These must be separate variables — never share a smoothed copy between
valuation and control uses.

### LOCK Statement Cost

`LOCK STEERING` and `LOCK THROTTLE` expressions are evaluated every physics
tick. If the locked expression is expensive (vector math, loops), it runs at full
physics rate and starves the main loop. Always lock to a pre-computed variable:

```kerboscript
// WRONG — runs vector math every physics tick:
LOCK STEERING TO SHIP:VELOCITY:ORBIT:NORMALIZED + some_offset.

// CORRECT — lock to a variable, compute in main loop:
GLOBAL desired_steering IS SHIP:PROGRADE.
// ... in main loop: SET desired_steering TO <computed direction>.
LOCK STEERING TO desired_steering.
```

### SAS Conflict

kOS `LOCK STEERING` and KSP SAS fight each other and cause oscillation. SAS must
be OFF before engaging `LOCK STEERING`. This is done once in `_RUN_FLIGHT_PLAN`:

```kerboscript
SAS OFF.
LOCK THROTTLE TO THROTTLE_CMD.
```

Do not re-enable SAS while lock steering is active. Provide an `ON ABORT` handler
that unlocks steering as its first action.

### Division by Zero Guards

kOS does not throw on divide-by-zero — it produces infinity or NaN silently.
Always guard denominators that can be zero:
- Mass flow (ṁ) at engine startup/shutdown → passes through zero
- Velocity magnitude at very low speeds → can make unit vectors erratic
- Dynamic pressure at guidance engagement → near zero
- ETA:APOAPSIS when at or past apoapsis → zero or negative

Pattern:
```kerboscript
IF ABS(denominator) < MIN_THRESHOLD { SET result TO 0. }
ELSE { SET result TO numerator / denominator. }
```

### Part List Iteration is Expensive

`SHIP:ENGINES`, `SHIP:PARTS`, etc. traverse the full vessel hierarchy each call.
Never iterate these in the main guidance loop. Classify and cache engine lists
once at phase initialisation.

### WHEN...THEN Triggers

`WHEN...THEN` blocks are evaluated every physics tick independently of the main
loop. They share the IPU budget. Keep trigger bodies minimal — set a flag and
return immediately. Never put complex logic inside a trigger body.

```kerboscript
WHEN (some_condition) THEN {
  SET some_flag TO TRUE.
  PRESERVE.  // keep the trigger alive for re-evaluation
}
```

The main loop reads the flag and executes the response.

### IPU Setting

Default IPU (200) is insufficient for a full guidance system. The kOS config
should have IPU set to at least 2000. Log actual loop timing with
`TIME:SECONDS - IFC_CYCLE_UT` to verify the loop is achieving its target rate.

### File I/O Blocking

`LOG x TO file.` is synchronous and can block for 100–500 ms on Windows with
antivirus active. The logger in this codebase uses a rate-limited write
(`IFC_CSV_LOG_PERIOD = 2.0 s`) for this reason. Never add synchronous LOG calls
inside the tight guidance loop.

---

## Adding a New Phase

1. Add phase name constant to `ifc_constants.ks` (`GLOBAL PHASE_FOO IS "FOO".`)
2. Add any new leg type constant if needed (`GLOBAL LEG_FOO IS "FOO".`)
3. Add all mutable state globals to `ifc_state.ks` and reset them in
   `IFC_INIT_STATE()`.
4. Add any new tunables to `ifc_constants.ks`.
5. Add per-aircraft config keys to `aircraft_template.ks` with `-1` sentinel.
6. Create `phases/phase_foo.ks` with a `RUN_FOO()` function.
7. Load it in `ifc_main.ks` with `RUNPATH(ifc_root + "phases/phase_foo.ks").`
8. Add `ELSE IF IFC_PHASE = PHASE_FOO { RUN_FOO(). }` to the main loop.
9. Handle the new leg type in `_INIT_LEG()`.
10. Add display rendering in `ifc_display.ks`.
11. Add telemetry export globals (`TELEM_FOO_*`) to `ifc_state.ks` and log them.

## Adding New Ascent-Specific Globals

New globals for ascent go in `ifc_state.ks` under a clearly labelled section
`// --- Ascent phase state ---`. They must be reset in `IFC_INIT_STATE()`.
Ascent telemetry exports follow the `TELEM_` prefix convention.
