# IFC TODO

## Handoff: AMO Differential Steering (next session)

### Status
- [x] Differential braking path works and respects `diff_brake_strength`.
- [x] Default brake strength baseline restored when not steering (`abrk_default_strength`).
- [x] Nose-gear brake suppression added during differential steer (nose bindings forced to 0 while split braking is active).
- [ ] Differential thrust still not functioning in flight tests.

### Open Work (Diff Thrust)
- [ ] Diagnose why AMO engine banks are not being applied even when engines are present.
- [ ] Verify `_AMO_MAKE_ENGINE_ENTRY()` creates valid limiter bindings for this craft (suffix or module field path).
- [ ] Confirm writes in `_AMO_SET_ENTRY_LIMIT_FRAC()` actually change per-engine limiter values at runtime.
- [ ] If `ModuleEnginesFX` field writing is unreliable, add/try alternate limiter field/module fallbacks for this aircraft.
- [ ] Add temporary debug logging in AMO discovery/apply path.
- [ ] Re-test in KSP: A/D steering should set outside bank limiter to throttle fraction and inside bank to idle.

### Files to revisit
- `Integrated Flight Computer/lib/ifc_amo.ks`
- `Integrated Flight Computer/aircraft/c_1000h_cfg.ks`
- `Integrated Flight Computer/lib/ifc_display.ks`
## Feature: Cruise Speed Mode Popup (`ifc_gui.ks`)
- [ ] Update handle layout comment at top of file
- [ ] Simplify `_GUI_CRUISE_SPD_TEXT()` — numeric only, no "M" prefix
- [ ] Simplify `_GUI_CRUISE_PARSE_SPD_INPUT()` — takes mode string, returns clamped number
- [ ] `_GUI_BUILD_EDIT()`: add `spd_mode_pm` popup at [5], shift nav [5–7]→[6–8], nav-specific [8+]→[9+]
- [ ] `_GUI_REFRESH_EDIT()`: refresh [5] popup index, update nav label index [6]→[7], shift nav-specific indices
- [ ] `_GUI_TICK_EDIT()`: handle [5] popup CHANGED, update nav cycle buttons [5]/[7]→[6]/[8], shift nav-specific indices
- [ ] `_GUI_COMMIT_EDIT_FIELDS()`: read mode from [5] popup, shift nav-specific indices

## Feature: New State Globals (`ifc_state.ks`)
- [x] Add `FLIGHT_PLAN_DRAFT_COPY`, `GUI_INFLIGHT_MODE`, `GUI_INFLIGHT_COMMIT_PENDING` declarations
- [x] Reset all three in `IFC_INIT_STATE()`

## Feature: End-of-Flight Confirmation
- [x] `ifc_gui.ks`: add `_GUI_SHOW_END_CONFIRM()` — blocking kOS GUI dialog, returns TRUE/FALSE
- [x] `ifc_main.ks`: `_RUN_FLIGHT_PLAN()` — add `draft_copy` param, save to `FLIGHT_PLAN_DRAFT_COPY` after init, track quit flag, return `"DONE"` or `"QUIT"`
- [x] `ifc_main.ks`: `_IFC_INTERACTIVE_START()` — wrap in outer loop; on `"DONE"` call confirm dialog, save/restore `DRAFT_PLAN` across `IFC_INIT_STATE()`; on `"QUIT"` exit
- [x] `ifc_main.ks`: update `RUN_IFC()` and `RUN_TAKEOFF_IFC()` to pass `LIST()` as `draft_copy`

## Feature: In-Flight Plan Editing
- [x] `ifc_gui.ks`: add `_GUI_OPEN_INFLIGHT()` — populate `DRAFT_PLAN` from remaining `FLIGHT_PLAN_DRAFT_COPY` legs, rename COMMIT/DISCARD buttons, set `GUI_INFLIGHT_MODE`
- [x] `ifc_gui.ks`: add `_GUI_TICK_INFLIGHT()` — delegate to `_GUI_TICK()`, handle ARM→commit / QUIT→discard
- [x] `ifc_main.ks`: add `_IFC_SKIP_LEG()` function
- [x] `ifc_main.ks`: in main loop — inflight commit splice handler + call `_GUI_TICK_INFLIGHT()` when `GUI_INFLIGHT_MODE`
- [x] `ifc_menu.ks`: add "Edit Plan" and "Skip Leg" items to in-flight menu + dispatch handlers


