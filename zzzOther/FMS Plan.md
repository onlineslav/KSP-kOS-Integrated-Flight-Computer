# IFC/FMS UI Remake Plan

## 1. Audit Scope Completed
I reviewed the full active IFC codebase before planning this remake.

Files reviewed:
- `Integrated Flight Computer/ifc_main.ks`
- `Integrated Flight Computer/lib/ifc_constants.ks`
- `Integrated Flight Computer/lib/ifc_state.ks`
- `Integrated Flight Computer/lib/ifc_helpers.ks`
- `Integrated Flight Computer/lib/ifc_aa.ks`
- `Integrated Flight Computer/lib/ifc_logger.ks`
- `Integrated Flight Computer/lib/ifc_fms.ks`
- `Integrated Flight Computer/lib/ifc_ui.ks`
- `Integrated Flight Computer/lib/ifc_menu.ks`
- `Integrated Flight Computer/lib/ifc_display.ks`
- `Integrated Flight Computer/lib/ifc_telemetry.ks` (legacy path still present)
- `Integrated Flight Computer/nav/nav_math.ks`
- `Integrated Flight Computer/nav/nav_beacons.ks`
- `Integrated Flight Computer/nav/nav_routes.ks`
- `Integrated Flight Computer/phases/phase_approach.ks`
- `Integrated Flight Computer/phases/phase_autoland.ks`
- `Integrated Flight Computer/phases/phase_takeoff.ks`
- `Integrated Flight Computer/phases/phase_cruise.ks`
- `Integrated Flight Computer/phases/phase_ascent.ks`
- `Integrated Flight Computer/phases/phase_reentry.ks`
- `Integrated Flight Computer/tests/takeoff_vr_probe.ks`
- `Integrated Flight Computer/aircraft/*.ks` configs and template
- `boot/ifc_bootloader.ks`
- `boot/ifc_testflight_bootloader.ks`

## 2. Current IFC State (What Changed)
The IFC is no longer a simple approach-only script. It now has:
- A leg-based flight plan executor in `ifc_main.ks`.
- Separate phase runners for takeoff, cruise, approach, autoland, plus ascent/reentry stubs.
- A new UI stack split into `ifc_ui.ks`, `ifc_display.ks`, and `ifc_menu.ks`.
- A JSON-backed plan save/load helper in `ifc_fms.ks`.
- Centralized state and telemetry export variables in `ifc_state.ks`.
- Continuous CSV logging through `ifc_logger.ks`.

This is a strong architectural direction, but the UX layer is inconsistent and fragile.

## 3. Why It Feels Broken Right Now
### Critical UX/quality issues
- Character rendering is corrupted (mojibake for box-drawing/arrow glyphs), so readability collapses.
- Menu and display responsibilities are mixed with direct control actions and side effects.
- There is no coherent “interaction model” across pre-arm, in-flight, and abort states.
- `ifc_telemetry.ks` still exists as a legacy path, creating confusion about the true UI source of truth.

### Structural UI issues
- Layout is hardcoded to specific rows with limited adaptability and no formal screen-state machine.
- Menu options are fixed-index and brittle (`_MENU_NITEMS` + cursor math) instead of data-driven.
- Alerting is single-slot overwrite only, so important events are easily lost.
- No explicit focus/input mode handling beyond open/closed menu boolean.
- Save/load UX is minimal (single filename, no slotting, no validation feedback loop).

### QoL gaps
- No “quick actions” page for common actions during rollout/takeoff.
- No dedicated failures/warnings panel (AA unavailable, FAR unavailable, bad plate/config).
- No guided arm-checklist before committing ARM.
- No clear distinction between “configuration values”, “staged overrides”, and “live controls”.

## 4. UX Remake Goals
- Make the terminal predictable: one source of truth for rendering and one for input.
- Make every phase page useful at a glance with minimal scanning.
- Keep manual overrides and high-frequency actions safe and obvious.
- Preserve all existing autopilot logic and telemetry/logging behavior.
- Keep all implementation kOS-native and maintainable.

## 5. Target Interaction Model
### Global interaction modes
- `MODE_PREARM`: editable flight setup, checklist, ARM gating.
- `MODE_AUTOFLOW`: autopilot engaged, phase pages active.
- `MODE_MENU_OVERLAY`: focused menu navigation over current page.
- `MODE_MANUAL_OVERRIDE`: AP suspended, monitoring still active.
- `MODE_COMPLETE`: post-flight summary and exit options.

### Input contract
- Global hotkeys always available (`M`, `D`, `L`, `Q`) with strict behavior.
- Menu keys only active in `MODE_MENU_OVERLAY`.
- Action keys (`G`, flap step, spoilers, brakes) routed through one command dispatcher.
- Every command generates a structured alert entry with severity and timestamp.

## 6. Technical Remake Design
### 6.1 UI foundation
- Keep `ifc_ui.ks` as low-level primitives only.
- Replace non-ASCII visual dependencies with ASCII-safe defaults to stop charset failures.
- Define one layout schema in constants/state, not scattered row literals.

### 6.2 View-model layer (new)
- Add a small “UI view model” builder per tick that gathers display-ready values.
- Keep renderers pure: they should read view-model data and render only.
- Remove phase-function calls from display code where possible (reduce coupling).

### 6.3 Menu system rewrite
- Convert menu from index-based hardcoding to data-driven entries.
- Add explicit item types: `choice`, `numeric`, `toggle`, `action`, `submenu`.
- Add staged edits + validation before ARM.
- Add saved-plan slots (`ifc_plan_1.json`, etc.) plus confirmation messaging.

### 6.4 Alert/event system
- Replace single alert string with a short event queue.
- Severity tags: `INFO`, `WARN`, `ERROR`.
- Show latest event on alert bar; allow reviewing last N events in debug/menu.

### 6.5 Legacy cleanup
- Retire `ifc_telemetry.ks` from active runtime path.
- Preserve logger behavior; ensure UI does not duplicate logger concerns.
- Keep backward-compatible wrappers `RUN_IFC` and `RUN_TAKEOFF_IFC`.

## 7. Implementation Phases
### Phase 1: Stabilize rendering and control flow
- Normalize all display strings to ASCII-safe output.
- Introduce explicit UI mode state in `ifc_state.ks`.
- Keep current pages, but route through mode-aware render dispatcher.
- Exit criteria: no corrupted characters, no row stomping, no menu/input deadlocks.

### Phase 2: Menu architecture rewrite
- Replace `_MENU_NITEMS` style handling with declarative menu tables.
- Add validation and commit/apply flow for staged values.
- Add clear ARM gating summary.
- Exit criteria: predictable navigation, no invalid state transitions.

### Phase 3: QoL and pilot workflow upgrades
- Add pre-arm checklist page.
- Add quick actions panel per phase.
- Add event history viewer.
- Add plan slot save/load UX.
- Exit criteria: pilot can configure/arm/operate without reading source code.

### Phase 4: Final polish + compatibility checks
- Verify all primary phases render correctly: prearm, takeoff, cruise, approach, flare, touchdown, rollout, done.
- Verify testflight path with VR probe still works with UI changes.
- Verify old entry points and bootloaders still function.

## 8. Acceptance Criteria
- No scrolling spam during normal operation.
- All active controls are discoverable from key hint row or menu.
- Menu is fully operable without “hidden” key knowledge.
- Alerts are readable, timestamped, and not silently lost.
- UI remains stable at 20 Hz loop with no functional AP regressions.
- Bootloader and testflight scripts can still launch IFC reliably.

## 9. Risks To Manage
- Tight coupling between display and phase internals can cause regressions if changed too aggressively.
- kOS terminal capabilities are limited; avoid over-designing widgets.
- Input handling can conflict with pilot manual controls if mode boundaries are weak.

## 10. Immediate Next Step
Implement Phase 1 first, then test only for UI stability before any major menu-feature additions.

