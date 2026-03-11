# IFC Flight Management System — UI Design Plan

## Overview

Transform the Integrated Flight Computer from a scrolling-print debug terminal into a
proper Flight Management System: a structured, non-scrolling terminal display with
context-sensitive pages, a navigable menu system, and live keyboard controls.

The existing autopilot logic (phase runners, nav math, ILS coupling, autothrottle,
logger) is **unchanged**. The UI layer sits on top, reading the same global state
variables and writing nothing back to autopilot internals except through explicit
user-initiated commands.

---

## Goals

- Show only the information relevant to the current flight phase — nothing more.
- Replace scrolling PRINT output with a fixed-position, overwrite-updated display.
- Replace the text-prompt startup menu with a full navigable FMS CDU-style menu.
- Allow the pilot to interact with the IFC during flight (arm/disarm, flaps, gear,
  debug toggle) without breaking the main loop.
- Keep the implementation 100% within kOS — no external tools or addons required
  beyond what already exists (kOS-AA, FAR).

---

## New File Structure

```
Integrated Flight Computer/
├── ifc_main.ks              ← updated: boot, main loop, menu action dispatch
├── lib/
│   ├── ifc_ui.ks            ← NEW: low-level screen engine
│   ├── ifc_menu.ks          ← NEW: menu tree, keyboard input, navigation state
│   ├── ifc_display.ks       ← NEW: context-sensitive page renderers
│   ├── ifc_constants.ks     ← unchanged
│   ├── ifc_state.ks         ← unchanged
│   ├── ifc_helpers.ks       ← unchanged
│   ├── ifc_aa.ks            ← unchanged
│   ├── ifc_telemetry.ks     ← retired (replaced by ifc_display.ks)
│   └── ifc_logger.ks        ← unchanged
├── nav/                     ← unchanged
├── phases/                  ← unchanged
└── aircraft/                ← unchanged
```

Load order in `ifc_main.ks`:
```
ifc_constants → ifc_state → ifc_helpers → ifc_aa →
ifc_ui → ifc_menu → ifc_display →
ifc_logger → nav_math → nav_beacons →
phase_approach → phase_autoland → phase_takeoff
```

---

## Terminal Layout

Target: 50-column × 36-row kOS terminal (configurable via `TERMINAL:WIDTH` /
`TERMINAL:HEIGHT`). All rows are fixed-position — the screen never scrolls. Each
zone is overwritten every display tick.

```
Row  0  ╔══════════════════════════════════════════════════╗
Row  1  ║  IFC 2.0   X10-F Bigboi           T+ 00:02:14  ║
Row  2  ╠══════════════════════════════════════════════════╣
Row  3  ║  APPROACH › ILS TRACK     KSC ILS 09   3.0° GS ║
Row  4  ╠══════════════════════════════════════════════════╣
Row  5  ║                                                  ║
Row  6  ║   [PRIMARY DATA — phase-specific, 6 rows]        ║
...
Row 10  ║                                                  ║
Row 11  ╠══════════════════════════════════════════════════╣
Row 12  ║  [SECONDARY DATA — debug, toggleable, 3 rows]   ║
Row 14  ╠══════════════════════════════════════════════════╣
Row 15  ║  [ALERT BAR — phase events, warnings, 1 row]    ║
Row 16  ╠══════════════════════════════════════════════════╣
Row 17  ║  [LOGGER STATUS — file, size, 1 row]            ║
Row 18  ╠══════════════════════════════════════════════════╣
Row 19  ║  [KEY HINTS — context-sensitive, 1 row]         ║
Row 20  ╚══════════════════════════════════════════════════╝
```

### Zone Definitions

| Zone | Rows | Content | Update Rate |
|---|---|---|---|
| **HEADER** | 0–2 | Aircraft name, mission timer, armed state | 1 Hz |
| **BREADCRUMB** | 3 | Phase › Subphase, runway, glideslope angle | 1 Hz |
| **PRIMARY** | 5–10 | Phase-specific main data (see Pages) | 2 Hz |
| **SECONDARY** | 12–14 | Debug telemetry, controller internals | 1 Hz (toggle) |
| **ALERT** | 15 | Most recent phase event / warning | on event |
| **LOGGER** | 17 | Log filename, byte count, status | 0.5 Hz |
| **KEYHINTS** | 19 | Active key bindings for current context | on phase change |
| **MENU** | 5–17 | Overlays primary+secondary when open | on input |

---

## Display Pages (Primary Zone — Rows 5–10)

One renderer function per flight phase, called by `DISPLAY_TICK()`.

---

### Page: PRE-FLIGHT / ARM SCREEN

Shown before ARM is confirmed. Replaces the current scrolling startup text.

```
  AIRCRAFT  X10-F Bigboi              MASS  42.3 t
  RUNWAY    KSC ILS 09       HDG  090°    GS  3.0°
  PROCEDURE ILS APPROACH     DIST  60 km

  SPEEDS    Vapp 140  Vref 120  VR 160  V2 176  m/s
  FLAPS     TAKEOFF (detent 1)   Vfe 250 m/s

  ARM TAKEOFF/APPROACH?   [Y] confirm   [N] cancel
```

---

### Page: FLY TO FIX

Shown during `SUBPHASE_FLY_TO_FIX`.

```
  IAS  139.8 m/s     AGL  2400 m      VS  -4.1 m/s
  HDG   065°         THR   0.52       FLP  ▌▌ CLMB

  ┌─ ACTIVE FIX ─────────────────────────────────┐
  │  KSC_IAF_09     BEARING  065°   DIST  42.1 km│
  │  TARGET ALT     1500 m          ETA   ~8 min  │
  └──────────────────────────────────────────────┘

  NEXT FIX: KSC_FAF_09    SPD MODE: INTERCEPT
  Vtgt 148.0   Vint 148.0   Vapp 140.0
```

---

### Page: ILS TRACK

Shown during `SUBPHASE_ILS_TRACK`.

```
  IAS  139.8 m/s     AGL   312 m      VS  -3.2 m/s
  HDG   090°         THR   0.41       FLP  ▐▐ APPR

  LOC  ◄━━━━━━╋━━━━━━━━━►   +1.2 m RIGHT
  G/S  ◄━━━━━━━━╋━━━━━━►   -0.5 m LOW     DIST  8.75 km

  SPD  FINAL     Vtgt 139.8   Vapp 140.0   Vref 120.0
  GEAR ▼ DOWN    SPOILERS ■ ARMED
```

ILS deviation bars are 20-character wide ASCII graphics:

- Center tick `╋` at position 10.
- Aircraft position marker `◄►` rendered relative to deviation scaled to ±10 chars.
- Text label shows numeric deviation and direction.

---

### Page: FLARE

Shown during `PHASE_FLARE`.

```
  IAS  131.2 m/s     THR   0.12       FLP  ▐▐▐ LAND
  AoA    12.4°       PITCH   6.1°

  AGL  ████████████████████░░░░░░░░░░░░░░░  18 m
  VS   ━━━━━━━━━━━━━━━━━━━━━━━━━━►  -1.8 m/s  (tgt -0.9)

  FLARE PROGRESS  ████████████████░░░░░░  68%
  FPA CMD  -1.2°
```

- AGL bar fills from right to left as altitude decreases to zero.
- VS bar shows current vs. target with a moving marker.
- Flare progress bar shows interpolation fraction from entry to touchdown.

---

### Page: TOUCHDOWN

Shown during `PHASE_TOUCHDOWN`.

```
  IAS  128.4 m/s     AGL    0.4 m     VS  -0.6 m/s

  ████  TOUCHDOWN CONFIRMED  ████

  BRAKES  DEPLOYING (0.5 s delay)
  SPOILERS ■ DEPLOYED
  DROGUE   ○ NOT ARMED
```

Brief phase — typically 0.5 s — before transitioning to ROLLOUT.

---

### Page: ROLLOUT

Shown during `PHASE_ROLLOUT`.

```
  IAS  115.2 m/s     AGL    0.0 m     BRAKES ■ ON

  HDG ERR  ━━━━━━━━━━╋━━━━━►   +1.8°  (tgt 090°)
  PITCH    ━━━━━━━━━━╋━━━━━░   +3.2°  (tgt  0.8°)

  YAW ASSIST  ▐░░░░░░░░░░  0.08   GATE: ACTIVE
  STEER BLEND  ████░░░░░░  42%

  STOPPING   ~900 m remaining (est.)
```

- Heading error bar: 20-char, ±15° scale.
- Pitch bar: 20-char, target marked with `░`.
- Stopping estimate: `(IAS² / (2 × decel_estimate))` — rough only, clearly labelled est.

---

### Page: TAKEOFF

Shown during all `PHASE_TAKEOFF` sub-phases.

```
  IAS   95.2 m/s     AGL    0.2 m     THR  1.00

  V SPEEDS   V1 ──── VR [160] ──── V2 [176] m/s
  IAS TAPE   ▐▐▐▐▐▐▐▐▐▐▐░░░░░░░░░░░░░░░░░░  95 m/s
                     ▲ VR                ▲ V2

  SUB-PHASE   GROUND ROLL
  YAW ASSIST  ▐▐▐▐░░░░░░   0.28   CENTERLINE  +2.1 m
  PITCH CMD   ───   (waiting for VR)
  THRUST      824 / 860 kN   95.8%
```

Sub-phase line changes text as the phase advances:
- `SPOOL UP` → `GROUND ROLL` → `ROTATE` → `CLIMB OUT` → `COMPLETE`

IAS tape is a 30-char fill bar with V-speed markers overlaid as labels below.

---

### Page: COMPLETE

Shown after `PHASE_DONE`.

```
  ╔════════════════════════════════╗
  ║   LANDING COMPLETE             ║
  ╠════════════════════════════════╣
  ║  RWY        KSC ILS 09        ║
  ║  TOTAL TIME T+ 00:08:42       ║
  ║  LOG FILE   ifc_log_001.csv   ║
  ╚════════════════════════════════╝

  Controls released.  Logger closed.
  [Q] Exit   [L] Open log folder
```

---

## Secondary Zone (Rows 12–14) — Debug Panel

Toggled with `D` key. Hidden by default. Shows controller internals useful for
tuning. Content is phase-sensitive:

**APPROACH / ILS TRACK:**
```
  SPD  FINAL  Vtgt 139.8  Vbase 139.8  Vint 148.0  ThrI  0.04
  CAP  LOC ✓  GS ✓  arm 0.0s  SF 0.62   THR_INTEGRAL  0.04
  AA   hdg 090.0  fpa -3.08  locCorr -0.08  gsCorr +0.42
```

**FLARE:**
```
  tgtVS -0.92  VS -1.81  frac 0.68  FPAcmd -1.22
  AoA 12.4  pitch 6.1  IAS 131.2  AGL 18
  AA   hdg 090.0  fpa -1.22
```

**ROLLOUT:**
```
  yawTgt 0.082  yawCmd 0.079  scale 0.72  gate ACTIVE  hdgErr +1.8
  pitchTgt 3.2  pitchErr 0.3  ff 0.012  cmd 0.028  steerBlend 0.42
  steerHdg 090.0  locCorr -0.04  rollAssist OFF
```

**TAKEOFF:**
```
  sub GROUND_ROLL  Vtgt 0.0  VR 160.0  V2 176.0  FLP 1->1
  hdg 090.0  steer 090.0  locCorr +0.022  pCmd 0.00  THR 1.00
  yawTgt 0.28  yawCmd 0.25  scale 0.58  locErr +2.1 m
```

---

## Alert Bar (Row 15)

Single line, overwritten on each event. Cleared after 5 seconds.

Examples:
```
  ⚠  LOC CAPTURE — STABILIZING                     [T+ 04:21]
  ✓  GEAR DOWN AND LOCKED                          [T+ 05:02]
  ⚠  FLARE ENTRY — AGL 130 m   VS -3.2 m/s        [T+ 06:44]
  ✓  TOUCHDOWN — VS -0.6 m/s                       [T+ 07:01]
  ⚠  BOUNCE DETECTED — RE-ENTERING FLARE           [T+ 07:03]
  ✓  FLAPS APPROACH — 2 detents                    [T+ 03:10]
  ⚠  VFE LIMIT — RETRACTING FLAPS (220 m/s)        [T+ 02:30]
  ✓  ILS TRACK ENGAGED                             [T+ 05:15]
  ⚠  AA NOT DETECTED — kOS steering fallback       [T+ 00:01]
```

All events currently printed via scrolling `PRINT` are redirected here.

---

## Logger Status Bar (Row 17)

```
  LOG  ifc_log_042.csv   12.4 KB   ■ ACTIVE   [L] stop
```

When logger is inactive:
```
  LOG  ○ inactive                              [L] start
```

---

## Key Hints Bar (Row 19)

One line, changes by phase:

**Pre-flight/ARM:**
```
  [Y] ARM   [N] Cancel
```

**Approach/Takeoff (in-flight):**
```
  [M] Menu   [D] Debug   [G] Gear   [F/f] Flap+/-   [S] Spoiler   [Q] Abort
```

**Rollout:**
```
  [M] Menu   [D] Debug   [B] Brakes   [Q] Abort
```

**Complete:**
```
  [Q] Exit
```

---

## Keyboard Controls

All key reads use `TERMINAL:INPUT:HASCHAR` + `TERMINAL:INPUT:GETCHAR()` polled
once per main loop cycle (20 Hz). No blocking waits outside the pre-flight ARM
confirmation.

| Key | Context | Action |
|---|---|---|
| `M` | In-flight | Toggle menu open/close |
| `D` | In-flight | Toggle secondary debug panel |
| `G` | In-flight | Toggle gear (GEAR action group) |
| `F` | In-flight | Flap step up (AG `ag_flaps_step_up`) |
| `f` | In-flight | Flap step down (AG `ag_flaps_step_down`) |
| `S` | In-flight | Toggle spoilers (AG `ag_spoilers`) |
| `T` | In-flight | Toggle thrust reversers (AG `ag_thrust_rev`, if equipped) |
| `P` | In-flight | Toggle drogue chute (AG `ag_drogue`, if equipped) |
| `B` | Rollout | Toggle wheel brakes |
| `L` | Any | Start/stop/cycle logger |
| `Q` | Any | Abort IFC, unlock all controls, return to manual |
| `Y` / `N` | ARM screen | Confirm / cancel ARM |
| `↑` / `↓` | Menu open | Navigate menu items |
| `←` / `→` | Menu open | Change inline value (runway, procedure, distance) |
| `Enter` | Menu open | Execute selected item or confirm value |
| `Esc` / `M` | Menu open | Close menu without action |

kOS character codes for arrow keys:
- Up: `TERMINAL:INPUT:GETCHAR() = "↑"` → use `CHAR(38)` / scan for kOS keycodes
- In practice: use `TERMINAL:INPUT:GETCHAR()` returning the raw kOS character
  constant and compare against `Terminal:Input` constants.

---

## Menu System

Opened with `M`. Renders as an overlay on the primary zone (rows 5–17).
Menu state is a global lexicon maintained by `ifc_menu.ks`.

### Menu Tree

```
MAIN MENU
├── FLIGHT PLAN
│   ├── Procedure ........... [ILS APPROACH] / [TAKEOFF]
│   ├── Runway .............. [09] / [27]
│   ├── Approach Distance ... [LONG 60 km] / [SHORT 30 km]
│   ├── ─────────────────────────────────────────────────
│   └── ARM ▶  (executes selected procedure, closes menu)
│
├── AIRCRAFT
│   ├── View Config ▶  (shows all AC_PARAM values, scrollable)
│   ├── V Speeds
│   │   ├── Vapp ............. [140.0] m/s  (read-only, from config)
│   │   ├── Vref ............. [120.0] m/s  (read-only)
│   │   ├── VR ............... [160.0] m/s  (read-only)
│   │   └── V2 ............... [176.0] m/s  (read-only)
│   ├── Flap Schedule ▶  (shows detents, Vfe limits, km thresholds)
│   └── Action Groups ▶  (shows AG assignments)
│
├── CONTROLS (in-flight only)
│   ├── Gear ................. [UP] / [DOWN]  → toggle gear AG
│   ├── Flaps ................ [0] [1] [2] [3]  → set target detent
│   ├── Spoilers ............. [ARMED] / [DISARMED]  → toggle AG
│   ├── Thrust Rev ........... [OFF] / [ON]  (if ag_thrust_rev > 0)
│   └── Drogue ............... [STOWED] / [DEPLOYED]  (if ag_drogue > 0)
│
├── DISPLAY
│   ├── Debug Panel .......... [OFF] / [ON]  → toggle secondary zone
│   ├── Logger ............... [ACTIVE] / [STOPPED]  → start/stop
│   └── Cycle Log File ▶  (close current log, open new numbered file)
│
├── SYSTEM
│   ├── Addons ▶  (shows kOS-AA status, FAR status)
│   ├── Loop Rate ............ [20 Hz] (read-only)
│   └── ABORT IFC ▶  (unlock all, disable AA, return manual control)
│
└── [ESC/M] Close Menu
```

### Menu Rendering (overlay on rows 5–17)

```
Row  5  ╔═══════════════════════════════╗
Row  6  ║  FLIGHT PLAN                  ║
Row  7  ║  ▶ Procedure    [ILS APPROACH]║
Row  8  ║    Runway       [09]          ║
Row  9  ║    Distance     [LONG 60 km]  ║
Row 10  ║    ─────────────────────────  ║
Row 11  ║    ARM ▶                      ║
Row 12  ║                               ║
Row 13  ║  AIRCRAFT                     ║
Row 14  ║  CONTROLS                     ║
Row 15  ║  DISPLAY                      ║
Row 16  ║  SYSTEM                       ║
Row 17  ╚═══════════════════════════════╝
```

- `▶` marks the currently selected row (cursor).
- `[value]` items cycle left/right with arrow keys.
- Selecting a top-level item expands its sub-menu in place.
- `ARM ▶` executes immediately and closes the menu.
- `ABORT IFC ▶` prompts a second confirmation before acting.

### Menu State (lexicon in `ifc_menu.ks`)

```kos
GLOBAL IFC_MENU_STATE IS LEXICON(
  "open",        FALSE,
  "top_sel",     0,        // selected top-level item index
  "sub_open",    "",       // which sub-menu is expanded ("" = top level)
  "sub_sel",     0,        // selected item within sub-menu
  "pending_val", LEXICON() // staged value changes before ARM
).
```

Values in `[brackets]` items are staged into `pending_val` and only applied when
`ARM ▶` is executed. This prevents live changes mid-flight from breaking an
in-progress approach.

---

## `ifc_ui.ks` — Screen Engine

Low-level primitives. All other display code calls these; nothing else calls
`PRINT AT` directly.

```kos
// Clear entire screen and draw chrome (borders, separators).
FUNCTION UI_INIT { ... }

// Print text at a fixed position, padding to width w with spaces.
// Prevents leftover characters when new text is shorter than old.
FUNCTION UI_PRINT { PARAMETER text, col, row, w. ... }

// Print a progress/deviation bar of width w centred at col, row.
// val: current value.  lo/hi: range.  zero: position of centre tick.
// fill_char, empty_char, marker_char: appearance characters.
FUNCTION UI_BAR { PARAMETER val, lo, hi, zero, w, col, row, fill_char, empty_char, marker_char. ... }

// Clear a rectangular zone (rows r0..r1, cols c0..c1) with spaces.
FUNCTION UI_CLEAR_ZONE { PARAMETER r0, r1, c0, c1. ... }

// Write a key-value pair: "  KEY   value" padded to terminal width.
FUNCTION UI_KV { PARAMETER key, value, col, row. ... }

// Write a section separator line.
FUNCTION UI_SEPARATOR { PARAMETER row. ... }

// Format seconds as "MM:SS" or "HH:MM:SS".
FUNCTION UI_FORMAT_TIME { PARAMETER secs. ... }

// Format m/s with one decimal, right-justified to width w.
FUNCTION UI_FORMAT_SPEED { PARAMETER v, w. ... }
```

---

## `ifc_display.ks` — Page Renderers

```kos
// Master tick function, called from main loop at IFC_DISPLAY_PERIOD.
// Dispatches to the correct page renderer based on IFC_PHASE/IFC_SUBPHASE.
// Does nothing if menu is open (MENU takes the primary zone).
FUNCTION DISPLAY_TICK { ... }

// Individual page renderers (one per phase/subphase combination):
FUNCTION DISPLAY_PREARM     { ... }   // ARM screen
FUNCTION DISPLAY_FLY_TO_FIX { ... }  // enroute to waypoint
FUNCTION DISPLAY_ILS_TRACK  { ... }  // coupled ILS approach
FUNCTION DISPLAY_FLARE      { ... }  // flare sequence
FUNCTION DISPLAY_TOUCHDOWN  { ... }  // ground contact
FUNCTION DISPLAY_ROLLOUT    { ... }  // decelerate on runway
FUNCTION DISPLAY_TAKEOFF    { ... }  // all TO sub-phases
FUNCTION DISPLAY_COMPLETE   { ... }  // PHASE_DONE

// Secondary (debug) panel — called after primary if debug is ON:
FUNCTION DISPLAY_SECONDARY  { ... }

// Header + breadcrumb (called every tick regardless of page):
FUNCTION DISPLAY_HEADER     { ... }
FUNCTION DISPLAY_BREADCRUMB { ... }

// Alert bar — writes event text; called by phase logic on events:
FUNCTION DISPLAY_ALERT { PARAMETER text. ... }

// Logger status bar:
FUNCTION DISPLAY_LOGGER_STATUS { ... }

// Key hints bar — called on phase change:
FUNCTION DISPLAY_KEYHINTS   { ... }
```

---

## `ifc_menu.ks` — Menu System

```kos
// Called once per main loop cycle. Checks TERMINAL:INPUT for keypresses.
// If menu is open: routes keys to menu navigation, renders overlay.
// If menu is closed: routes keys to live flight controls.
// Returns an action lexicon or 0 if no action this cycle.
FUNCTION MENU_TICK { ... }

// Open/close menu, reset navigation state.
FUNCTION MENU_OPEN  { ... }
FUNCTION MENU_CLOSE { ... }

// Render the current menu state into the primary zone.
FUNCTION MENU_RENDER { ... }

// Apply staged values from pending_val and execute an ARM or action.
FUNCTION MENU_EXECUTE_ARM { ... }
FUNCTION MENU_EXECUTE_ABORT { ... }

// Toggle helpers (called by live key handler and menu execute):
FUNCTION MENU_DO_GEAR    { ... }
FUNCTION MENU_DO_FLAP_UP { ... }
FUNCTION MENU_DO_FLAP_DN { ... }
FUNCTION MENU_DO_SPOILERS { ... }
FUNCTION MENU_DO_LOGGER  { ... }
FUNCTION MENU_DO_DEBUG   { ... }
```

---

## `ifc_main.ks` — Updated Main Loop

The main loop gains three additions per cycle:

```kos
UNTIL IFC_PHASE = PHASE_DONE {
    LOCAL actual_dt IS TIME:SECONDS - IFC_CYCLE_UT.
    SET IFC_CYCLE_UT  TO TIME:SECONDS.
    SET IFC_ACTUAL_DT TO CLAMP(actual_dt, 0.01, 0.5).

    // 1. Autopilot (unchanged)
    IF IFC_PHASE = PHASE_APPROACH {
        RUN_APPROACH().
    } ELSE IF IFC_PHASE = PHASE_FLARE OR
              IFC_PHASE = PHASE_TOUCHDOWN OR
              IFC_PHASE = PHASE_ROLLOUT {
        RUN_AUTOLAND().
    } ELSE IF IFC_PHASE = PHASE_TAKEOFF {
        RUN_TAKEOFF().
    }

    // 2. Input + menu (NEW)
    LOCAL action IS MENU_TICK().
    IF action <> 0 { _HANDLE_MENU_ACTION(action). }

    // 3. Display (NEW — replaces PRINT_TELEMETRY)
    DISPLAY_TICK().

    // 4. Logger (unchanged)
    LOGGER_WRITE().

    WAIT IFC_LOOP_DT.
}
```

`_HANDLE_MENU_ACTION` is a local function that interprets the action lexicon
returned by MENU_TICK and calls the appropriate IFC state changes (e.g., switching
runway, triggering ARM, toggling gear).

The interactive startup is replaced by the ARM screen (DISPLAY_PREARM) which renders
as the first display page and blocks in a non-scrolling way until Y or N is pressed.

---

## Display Update Rates

| Zone | Rate | Rationale |
|---|---|---|
| Header | 1 Hz | Mission timer changes every second |
| Breadcrumb | 1 Hz | Phase changes are infrequent |
| Primary | 2 Hz | Fast enough to track deviations; avoids flicker |
| Secondary | 1 Hz | Debug values don't need real-time refresh |
| Alert | On event | Event-driven, not polled |
| Logger | 0.5 Hz | File size grows slowly |
| Key hints | On phase change | Static within a phase |

All rates implemented via `(now - LAST_x_UT) < x_PERIOD` guards, same pattern
as the existing `PRINT_TELEMETRY`.

---

## Global State Additions (ifc_state.ks)

New variables added to `IFC_INIT_STATE()`:

```kos
GLOBAL IFC_DEBUG_PANEL_ON  IS FALSE.  // secondary zone visible
GLOBAL LAST_DISPLAY_UT     IS 0.      // primary display rate limiter
GLOBAL LAST_HEADER_UT      IS 0.      // header rate limiter
GLOBAL LAST_LOGGER_UT      IS 0.      // logger bar rate limiter
GLOBAL IFC_ALERT_TEXT      IS "".     // current alert message
GLOBAL IFC_ALERT_UT        IS -99.    // UT when alert was set (for 5 s expiry)
GLOBAL IFC_DISPLAY_PERIOD  IS 0.5.    // s between primary refreshes (2 Hz)
GLOBAL IFC_HEADER_PERIOD   IS 1.0.    // s between header refreshes
```

---

## Constants Additions (ifc_constants.ks)

```kos
GLOBAL IFC_ALERT_EXPIRE_S  IS 5.0.   // s before alert bar clears
GLOBAL UI_WIDTH            IS 52.    // terminal columns (including borders)
GLOBAL UI_PRIMARY_TOP      IS 5.     // first row of primary zone
GLOBAL UI_PRIMARY_BOT      IS 10.    // last row of primary zone
GLOBAL UI_SECONDARY_TOP    IS 12.    // first row of secondary zone
GLOBAL UI_SECONDARY_BOT    IS 14.
GLOBAL UI_ALERT_ROW        IS 15.
GLOBAL UI_LOGGER_ROW       IS 17.
GLOBAL UI_KEYHINTS_ROW     IS 19.
```

---

## Migration from ifc_telemetry.ks

`ifc_telemetry.ks` is retired. Its `PRINT_TELEMETRY()` call in `ifc_main.ks`
is replaced with `DISPLAY_TICK()`. All scrolling `PRINT` statements in phase
files that announce events (phase transitions, flap changes, gear, bounce recovery)
are replaced with `DISPLAY_ALERT("text")` calls. No other changes to phase files.

Mapping:

| Old (ifc_telemetry.ks) | New (ifc_display.ks) |
|---|---|
| `PRINT "IFC T+..." AT (0,0)` | `DISPLAY_HEADER()` |
| `PRINT "IAS ... AGL ..." AT (0,1)` | Primary zone, row 5–6 |
| `PRINT "HDG ... PITCH ..." AT (0,2)` | Primary zone, row 6 |
| `PRINT "LOC ... GS ..." AT (0,3)` | `DISPLAY_ILS_TRACK()`, rows 7–8 |
| `PRINT "SPD ... CAP ..." AT (0,4)` | Secondary zone (debug panel) |
| `PRINT "AAcmd ..." AT (0,6)` | Secondary zone (debug panel) |
| `PRINT "RWY ..." AT (0,7)` | Breadcrumb zone, row 3 |
| Phase-change `PRINT` calls | `DISPLAY_ALERT(...)` |

---

## Implementation Order

1. **`ifc_ui.ks`** — Screen engine primitives. Test standalone: draw the chrome,
   print a static layout, verify all rows fit.

2. **`ifc_display.ks` header/breadcrumb/keyhints** — Non-scrolling header with
   mission timer. Wire into main loop, remove old `PRINT_TELEMETRY` call.

3. **`DISPLAY_ILS_TRACK()`** — Most complex primary page; validates bar renderer.
   Verify LOC/GS bars match numeric values.

4. **Remaining primary pages** — `DISPLAY_FLY_TO_FIX`, `DISPLAY_FLARE`,
   `DISPLAY_ROLLOUT`, `DISPLAY_TAKEOFF`, `DISPLAY_COMPLETE`.

5. **Secondary debug panel** — Port all per-phase debug lines from old telemetry.
   Wire `D` key toggle (inline, before menu system exists).

6. **Alert bar** — Replace all phase-file `PRINT` event announcements.

7. **`ifc_menu.ks`** — Menu tree, keyboard routing, live key controls.
   Wire `M` key and overlay rendering.

8. **ARM screen** — Replace `_IFC_INTERACTIVE_START` with `DISPLAY_PREARM`.

9. **Polish** — Tune column alignment, bar widths, abbreviations to fit 50 chars.
   Test on all aircraft configs and both runways.

---

## kOS Technical Notes

- `PRINT "text" AT(col, row)` — overwrites from `col`; always pad to clear old chars.
- `CLEARSCREEN` — avoid in the main loop; only call on mode transitions to prevent
  flash. The UI_INIT function calls it once; after that all updates are AT(x,y).
- `TERMINAL:INPUT:HASCHAR` — non-blocking check; poll once per main loop cycle.
- `TERMINAL:INPUT:GETCHAR()` — returns a single character; for arrow keys kOS
  returns multi-character escape sequences — test on the actual terminal to
  confirm the exact codes received and store them as named constants.
- `TERMINAL:WIDTH` / `TERMINAL:HEIGHT` — read at UI_INIT to auto-size UI_WIDTH
  and row constants; allows the layout to adapt to different terminal sizes.
- String padding: kOS has no printf-style formatting. Implement fixed-width
  number formatting in `ifc_ui.ks` (e.g., `UI_FORMAT_SPEED`, `UI_FORMAT_TIME`)
  using string concatenation and `SUBSTRING`/`PADLEFT` patterns.
- kOS has no `PADLEFT`/`PADRIGHT` built-in — implement as a helper that appends
  spaces until the string reaches the target width, or prepends spaces for
  right-justification.
