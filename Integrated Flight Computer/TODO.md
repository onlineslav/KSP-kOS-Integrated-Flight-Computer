# IFC Performance Review — TODO

## P1 — Cache facing/up/north vectors once per loop
- [x] **P1a** — Add `IFC_FACING_FWD`, `IFC_FACING_STAR`, `IFC_FACING_TOP`, `IFC_UP_VEC`, `IFC_NORTH_VEC` globals to `ifc_state.ks`; reset in `IFC_INIT_STATE`
- [x] **P1b** — Compute all five vectors once at the top of the main loop in `ifc_main.ks` before any phase function runs
- [x] **P1c** — Rewrite `GET_COMPASS_HDG`, `GET_PITCH`, `GET_BANK` in `ifc_helpers.ks` to read the cached globals instead of querying `SHIP:FACING`/`SHIP:UP`/`SHIP:NORTH` directly
- [x] **P1d** — Add `TELEM_COMPASS_HDG`, `TELEM_PITCH_DEG`, `TELEM_BANK_DEG` globals to `ifc_state.ks`; compute them once per loop in `ifc_main.ks` after the vector cache; replace all call-sites in `ifc_display.ks` and `ifc_logger.ks`

## P2 — Cache ILS runway vectors on plate load
- [x] **P2a** — Add `ILS_RWY_FWD`, `ILS_RWY_RIGHT` globals to `ifc_state.ks`; reset in `IFC_INIT_STATE`
- [x] **P2b** — Compute them once in `IFC_LOAD_PLATE` (or equivalent plate-init path) from `ils["hdg"]`
- [x] **P2c** — Replace the two `HEADING()` constructions inside `_COMPUTE_ILS_DEVIATIONS` with reads of `ILS_RWY_FWD`/`ILS_RWY_RIGHT`

## P3 — Deduplicate GEO_DISTANCE in approach
- [x] **P3a** — Add `ILS_DIST_KM` global to `ifc_state.ks`; reset in `IFC_INIT_STATE`
- [x] **P3b** — Compute `ILS_DIST_KM` once inside `_COMPUTE_ILS_DEVIATIONS` and store it
- [x] **P3c** — Replace the two independent `GEO_DISTANCE` calls in `_CHECK_FLAP_DEPLOYMENT` and `_CHECK_APPROACH_SPOILERS` with reads of `ILS_DIST_KM`

## P4 — Heading/pitch/bank as loop-level globals (display + logger)
- [x] **P4a** — Remove direct `GET_COMPASS_HDG()`/`GET_PITCH()`/bank calls from `ifc_display.ks`; replace with `TELEM_COMPASS_HDG`, `TELEM_PITCH_DEG`, `TELEM_BANK_DEG`
- [x] **P4b** — Remove direct calls from `ifc_logger.ks`; replace with telemetry globals

## P5 — SHIP:ENGINES traversal in logger
- [x] **P5a** — Add `TELEM_ENG_IGN_ON`, `TELEM_ENG_FLAMEOUTS` globals to `ifc_state.ks`; reset in `IFC_INIT_STATE`
- [x] **P5b** — Populate them inside the existing engine iteration in `ASC_STATE_UPDATE` (ascent) and equivalent phase-level engine checks; fall back to a cheap one-time count for non-ascent phases
- [x] **P5c** — Remove `SHIP:ENGINES` traversal from `LOGGER_WRITE`; read `TELEM_ENG_IGN_ON`/`TELEM_ENG_FLAMEOUTS` instead

## P6 — Display string allocation reduction
- [x] **P6a** — Replace `STR_REPEAT` loop in `ifc_ui.ks` with a lookup table of pre-built space strings (up to the terminal width)
- [x] **P6b** — Replace CSV row string concatenation in `ifc_logger.ks` with `LIST` + `:JOIN(",")`
