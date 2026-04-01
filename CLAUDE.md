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
STATUS          SURFACESPEED    UP              VELOCITY
VERTICALSPEED
```
Note: `SURFACESPEED` is obsolete since kOS 0.18 (replaced by `GROUNDSPEED`) but remains a protected bound name.

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

---

## Module Contracts

This section documents the public interface and ownership rules for each module.
"Owns" means sole writer. Phase modules read from state and write commands back
to state — they never call `LOCK THROTTLE` or `LOCK STEERING` directly.

### ifc_constants.ks
- **Exports:** None (constants only — never functions)
- **Owns:** All tuning constants and phase/subphase/leg-type name strings
- **Reads:** Nothing
- **Rule:** Never add mutable state here. If a value changes at runtime, it belongs in `ifc_state.ks`.

### ifc_state.ks
- **Exports:** `IFC_INIT_STATE()` (reset all runtime globals to defaults)
- **Owns:** Every mutable runtime global in the system
- **Reads:** Phase/subphase name constants (from `ifc_constants.ks`) for init defaults
- **Rule:** All new globals go here with a reset in `IFC_INIT_STATE()`. No logic — only declarations and the reset function.

### ifc_helpers.ks
- **Exports:** `CLAMP()`, `MOVE_TOWARD()`, `WRAP_180()`, `WRAP_360()`, `SET_PHASE()`, `SET_SUBPHASE()`, `PHASE_ELAPSED()`, `AC_PARAM()`, `GET_IAS()`, `GET_AGL()`, `GET_AOA()`, `GET_COMPASS_HDG()`, `GET_CURRENT_THROTTLE()`, `GET_PITCH()`, `GET_RUNWAY_REL_HEIGHT()`, `TRIGGER_AG()`, `PULSE_AG()`, `IFC_SET_ALERT()`, `IFC_PUSH_EVENT()`, `IFC_SET_UI_MODE()`
- **Owns:** Nothing (pure functions; reads/writes state globals it doesn't own)
- **Reads:** Phase state, alert state, facing vectors, `ACTIVE_AIRCRAFT`, `FAR_AVAILABLE`, `THROTTLE_CMD`
- **Rule:** No flight control output. Must never write to `THROTTLE_CMD` or `IFC_DESIRED_STEERING`.

### ifc_aa.ks
- **Exports:** `AA_INIT()`, `AA_SET_DIRECTOR()`, `AA_SET_CRUISE()`, `AA_DISABLE_ALL()`, `AA_POLL_TELEM()`, `AA_RESTORE_FBW()`
- **Owns:** Nothing declared; sets AA addon properties and writes `TELEM_AA_*` fields
- **Reads:** `AA_AVAILABLE`, `AA_FBW_ON`, `AA_DIRECTOR_ON`, `AA_MAX_*` limits, `ACTIVE_AIRCRAFT`, `IFC_UP_VEC`, `IFC_NORTH_VEC`
- **Rule:** Always guard every call with `IF AA_AVAILABLE`. Never touch `THROTTLE_CMD`.

### ifc_autothrottle.ks
- **Exports:** `AT_RESET()`, `AT_RUN_SPEED_HOLD()`
- **Owns:** Sole writer of `THROTTLE_CMD`. Also writes all `AT_*` estimator state and `TELEM_AT_*` fields.
- **Reads:** `IFC_ACTUAL_DT`, `PREV_IAS`, `A_ACTUAL_FILT`, `THR_INTEGRAL`, AT tuning constants, `GET_IAS()`, `GET_CURRENT_THROTTLE()`
- **Rule:** `THROTTLE_CMD` is written **only** here. Phase modules must not touch it directly. Call `AT_RUN_SPEED_HOLD()` from the phase loop; never call it more than once per cycle.

### ifc_ui.ks
- **Exports:** String formatters (`STR_PAD()`, `STR_RJUST()`, `UI_FMT()`, `UI_FORMAT_TIME()`, `UI_FORMAT_FLP()`), screen primitives (`UI_P()`, `UI_SEP()`, `UI_THICK_SEP()`, `UI_CLR()`, `UI_BAR_FILL()`, `UI_BAR_DEV()`), `UI_INIT()`
- **Owns:** `_UI_SPACES` lookup table; sets `UI_W` during `UI_INIT()`
- **Reads:** `UI_W`
- **Rule:** No flight state reads or writes. Pure presentation layer.

### ifc_display.ks
- **Exports:** `DISPLAY_TICK()`, per-phase display functions (`DISPLAY_CRUISE()`, `DISPLAY_ILS_TRACK()`, `DISPLAY_FLARE()`, `DISPLAY_ROLLOUT()`, `DISPLAY_TAKEOFF()`, `DISPLAY_ASCENT()`, etc.), `DISPLAY_HEADER()`, `DISPLAY_ALERT()`, `DISPLAY_SECONDARY_DEBUG()`
- **Owns:** Nothing — read-only view of state
- **Reads:** All major state and telemetry variables for rendering
- **Rule:** Must never write to flight state. If a display function appears to need to set something, that write belongs in a phase module or helper instead.

### ifc_logger.ks
- **Exports:** `LOGGER_INIT()`, `LOGGER_WRITE()`, `LOGGER_CLOSE()`
- **Owns:** `LOG_ACTIVE`, `LOG_FILE`, `LOG_LAST_WRITE_UT`
- **Reads:** All `TELEM_*` globals, phase state, control commands
- **Rule:** Rate-limited to `IFC_CSV_LOG_PERIOD` (2 s default). Never call `LOG x TO file.` outside this module — it blocks for 100–500 ms on Windows.

### ifc_menu.ks
- **Exports:** `MENU_BUILD_ITEMS()`, `MENU_RENDER()`, `MENU_DISPATCH()`, flap/gear/spoiler/reverser action functions
- **Owns:** `IFC_MENU_OPT_*`, `IFC_MENU_STG_*`, `IFC_MENU_SLOT`, `IFC_MENU_CURSOR`, `IFC_MENU_SCROLL`, `IFC_MENU_PAGE`, `IFC_MENU_ITEMS`
- **Reads:** `DRAFT_PLAN`, `ACTIVE_AIRCRAFT`, `IFC_PHASE`, `AA_AVAILABLE`, `FAR_AVAILABLE`, `LOG_ACTIVE`, `IFC_MANUAL_MODE`
- **Rule:** Menu actions (flaps, gear, etc.) are the only place AG triggers are called outside of phase code. Keep action handlers thin — set a flag or call `TRIGGER_AG()`, nothing more.

### ifc_fms.ks
- **Exports:** `FMS_SAVE_PLAN()`, `FMS_LOAD_PLAN()`, `FMS_SLOT_EXISTS()`
- **Owns:** `FMS_SLOT_MIN`, `FMS_SLOT_MAX`, `FMS_SLOT_PREFIX`, `FMS_SLOT_SUFFIX`
- **Reads:** `DRAFT_PLAN`, `FMS_LEG_CURSOR`, leg type constants, `PLATE_IDS`, `CUSTOM_WPT_IDS`
- **Rule:** File I/O only. No flight logic. Slot files are JSON; leg type strings are the canonical serialization format.

### ifc_gui.ks
- **Exports:** `_GUI_BUILD()`, `_GUI_CLOSE()`, `_GUI_TICK()`
- **Owns:** Nothing (writes to GUI handle globals declared in `ifc_state.ks`)
- **Reads:** `DRAFT_PLAN`, `FMS_LEG_CURSOR`, `FMS_EDITING_LEG`, `FMS_EDIT_FIELD`, all `GUI_*` handle state
- **Rule:** Pre-arm editor only. Must not be called after flight starts. Never touches flight control state.

### nav_math.ks
- **Exports:** `GEO_DISTANCE()`, `GEO_BEARING()`, `GEO_DESTINATION()`, `CROSS_TRACK_ERROR()`, `ALONG_TRACK_DIST()`
- **Owns:** Nothing (pure functions)
- **Reads:** `SHIP:BODY:RADIUS` only
- **Rule:** Stateless geodesic math. No side effects. Safe to call from any context.

### nav_beacons.ks
- **Exports:** `MAKE_BEACON()`, `MAKE_PLATE()`, `REGISTER_BEACON()`, `GET_BEACON()`
- **Owns:** `NAV_BEACON_DB`, all `PLATE_*` lexicons, `PLATE_IDS`
- **Reads:** `GEO_DESTINATION()` (for threshold point computation)
- **Rule:** Static database — never modified at runtime. Add new plates here only; do not create beacon globals elsewhere.

### Phase modules (phase_takeoff.ks, phase_cruise.ks, phase_approach.ks, phase_autoland.ks)
- **Exports:** One top-level `RUN_*()` function each, called from the main loop dispatcher
- **Owns:** Nothing — all state written through `ifc_state.ks` globals
- **Reads:** Their respective `TO_*` / `CRUISE_*` / `APP_*` / `FLARE_*` / `ROLLOUT_*` state variables, AA and autothrottle functions, nav functions
- **Write targets (per phase):**
  - All phases: `IFC_DESIRED_STEERING`, `TELEM_*` export vars
  - Takeoff/cruise/approach: call `AT_RUN_SPEED_HOLD()` (which writes `THROTTLE_CMD`)
  - Autoland: writes `THROTTLE_CMD` directly during flare/rollout (idle/manual thrust — autothrottle bypassed)
- **Rule:** Phase functions must call `SET_PHASE()` / `SET_SUBPHASE()` for all transitions — never write `IFC_PHASE` directly. Never call `LOCK THROTTLE` or `LOCK STEERING`.

---

## Claude Working Guidelines

These rules govern how Claude should approach changes in this codebase.

### Before starting any task
0. Read CLAUDE.md (this file) first. In particular, cross-reference every new variable
   and function name against the **kOS Reserved / Protected Names** section before writing
   code. Single-letter names (`r`, `v`, `q`, `n`, `g`, etc.) are especially risky —
   kOS built-ins include `R()`, `V()`, `Q()`. When in doubt, use a longer name.

### Before making any edit
1. Read the full file being changed, not just the function in question.
2. Identify which module contract(s) are affected and check the ownership rules above.
3. If the change touches a variable owned by a different module, flag it explicitly before proceeding.

### Scope discipline
- Only edit files that are necessary for the specific fix or feature requested.
- If a file was not mentioned in the request, do not edit it unless asked or unless the cross-module dependency makes it unavoidable — and in that case, say so.

### After making edits
- State exactly which lines changed and in which file(s).
- Call out any side-effects on other modules: e.g. "this changes the type of `CRUISE_WP_INDEX`, which `ifc_display.ks` also reads."
- If a variable's ownership was ambiguous or a contract was violated to make the fix work, flag it so the contracts section above can be updated.

### Commit message
After any session that modifies code, always produce a suggested git commit message following this format:
- Subject line: imperative mood, ≤72 chars, no period (e.g. `Fix FPA command sign error in ifc_aa.ks`)
- Body (if needed): what changed and why, one bullet per file or logical change
- Do not include Co-Authored-By or other metadata unless the user asks to commit

### Regression hygiene
- Before starting non-trivial work, note the current git state (which files are modified).
- Prefer small, targeted changes over refactors that touch multiple modules at once.
- When diagnosing a bug, compare against the CSV telemetry logs — identify *which channel* diverged and *when* before touching code.
