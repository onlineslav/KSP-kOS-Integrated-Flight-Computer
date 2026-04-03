@LAZYGLOBAL OFF.

// ============================================================
// engine_model_test.ks  -  Engine Model Standalone Test
//
// Loads only the minimum module stack required for
// ifc_engine_model.ks and runs it in a live display loop.
//
// Use this to:
//   1) Verify engine detection and DB matching
//   2) Check scale factor inference (TweakScale / thrust-ratio)
//   3) Validate intake supply formula units vs live REQUIREDFLOW
//   4) Confirm multimode eng:MODE strings for your DB entries
//   5) Tune EM_LOOKAHEAD_S and starvation margins
//
// This script does NOT touch throttle, steering, or any flight
// control — it is read-only observation only.
//
// ABORT → exits immediately.
//
// HOW TO USE:
//   1) Build a test aircraft with the engines/intakes you want
//      to characterise.
//   2) Run this script from the kOS terminal.
//   3) Use the throttle slider to observe demand at different
//      throttle settings. Fly to different altitudes / speeds
//      to see supply and margin update.
//   4) Check the DIAG panel for any DB mismatches or AMBIGUOUS
//      fields that need in-game verification.
// ============================================================

LOCAL ifc_root IS "0:/Integrated Flight Computer/".

// ── Minimum module stack ────────────────────────────────────
RUNPATH(ifc_root + "lib/ifc_constants.ks").
RUNPATH(ifc_root + "lib/ifc_state.ks").
RUNPATH(ifc_root + "lib/ifc_helpers.ks").
RUNPATH(ifc_root + "lib/ifc_engine_model.ks").

// ── Minimal stubs for helpers that ifc_helpers may reference ─
// ACTIVE_AIRCRAFT is read by AC_PARAM inside ifc_helpers.
// Provide a bare minimum so it doesn't crash.
IF NOT (DEFINED ACTIVE_AIRCRAFT) {
    GLOBAL ACTIVE_AIRCRAFT IS LEXICON("name", "EngModelTest").
}

// ── Fake THROTTLE_CMD so EM_INIT doesn't crash before lock ──
// The real IFC locks throttle and writes THROTTLE_CMD.
// In this test we mirror the actual throttle each tick instead.
GLOBAL THROTTLE_CMD IS SHIP:CONTROL:PILOTMAINTHROTTLE.

// ── IFC_ACTUAL_DT stub ───────────────────────────────────────
// ifc_state.ks already declares this; just make sure it has a
// safe initial value.
SET IFC_ACTUAL_DT TO 0.05.

// ── Init ────────────────────────────────────────────────────
IFC_INIT_STATE().
EM_INIT().

// ── Display constants ────────────────────────────────────────
LOCAL COL_W IS 50.
LOCAL UPDATE_HZ IS 10.   // display refresh rate (lighter than 50 Hz)
LOCAL update_dt IS 1.0 / UPDATE_HZ.
LOCAL last_disp_t IS 0.
LOCAL last_tick_t IS TIME:SECONDS.

CLEARSCREEN.

// ── ABORT handler ────────────────────────────────────────────
ON ABORT {
    UNLOCK THROTTLE.
    CLEARSCREEN.
    PRINT "Engine model test aborted.".
}

// ── Print static header ──────────────────────────────────────
PRINT "======================================================".
PRINT " ENGINE MODEL TEST  —  " + SHIP:NAME.
PRINT "======================================================".
PRINT "".
PRINT "  [THROTTLE] adjust demand  |  [ABORT] exit".
PRINT "------------------------------------------------------".

// ── Detection summary (printed once at init) ─────────────────
PRINT "".
PRINT "DB loaded    : " + TELEM_EM_DB_LOADED.
PRINT "Engines found: " + TELEM_EM_ENG_COUNT.
PRINT "Intakes found: " + TELEM_EM_INTAKE_COUNT.
PRINT "".

// ── Per-engine detail header ─────────────────────────────────
PRINT "------------------------------------------------------".
PRINT "ENGINES DETECTED:".
// (Printed once from EM state — the module already pushed events)
// We read them back via SHIP for the static block.
LIST ENGINES IN _disp_eng_list.
LOCAL di IS 0.
UNTIL di >= _disp_eng_list:LENGTH {
    LOCAL eng IS _disp_eng_list[di].
    LOCAL cr IS eng:CONSUMEDRESOURCES.
    IF cr:HASKEY("IntakeAir") {
        LOCAL mm_str IS "".
        IF eng:MULTIMODE { SET mm_str TO "  mode=" + eng:MODE. }
        PRINT "  [" + di + "] " + eng:PART:NAME + mm_str.
        PRINT "      MAXTHRUST=" + ROUND(eng:MAXTHRUST,1) + " kN" +
              "  FLAMEOUT=" + eng:FLAMEOUT.
        PRINT "      REQUIREDFLOW=" +
              ROUND(cr["IntakeAir"]:REQUIREDFLOW,4) + " u/s (live)".
    }
    SET di TO di + 1.
}
PRINT "------------------------------------------------------".

LOCAL disp_row IS 20.   // row where live data starts

// ── Main loop ────────────────────────────────────────────────
LOCAL running IS TRUE.
UNTIL NOT running {

    // Mirror actual throttle into THROTTLE_CMD so EM uses what
    // the player is commanding (no lock needed in test mode).
    SET THROTTLE_CMD TO SHIP:CONTROL:PILOTMAINTHROTTLE.

    // Measure dt
    LOCAL now IS TIME:SECONDS.
    LOCAL dt  IS now - last_tick_t.
    SET last_tick_t TO now.
    SET IFC_ACTUAL_DT TO CLAMP(dt, 0.01, 0.5).

    // Run engine model at full rate
    EM_TICK().

    // Live telemetry display at reduced rate
    IF now - last_disp_t >= update_dt {
        SET last_disp_t TO now.

        // ------ Live values block ------
        LOCAL row IS disp_row.

        PRINT AT(0, row):
            "  Throttle cmd : " + ROUND(THROTTLE_CMD * 100, 1) + " %       ".
        SET row TO row + 1.
        PRINT AT(0, row):
            "  Mach         : " + ROUND(TELEM_EM_MACH,    3) + "          ".
        SET row TO row + 1.
        PRINT AT(0, row):
            "  P_atm        : " + ROUND(TELEM_EM_P_ATM,   5) + " atm      ".
        SET row TO row + 1.
        PRINT AT(0, row):
            "  Rho          : " + ROUND(TELEM_EM_RHO,     4) + " kg/m³    ".
        SET row TO row + 1.
        PRINT AT(0, row): "".
        SET row TO row + 1.

        PRINT AT(0, row):
            "  IA DEMAND    : " + ROUND(TELEM_EM_Q_DEMAND, 4) + " u/s      ".
        SET row TO row + 1.
        PRINT AT(0, row):
            "  IA SUPPLY    : " + ROUND(TELEM_EM_Q_SUPPLY, 4) + " u/s      ".
        SET row TO row + 1.

        LOCAL margin_str IS ROUND(TELEM_EM_MARGIN, 4) + " u/s".
        IF TELEM_EM_STARVING = 1 { SET margin_str TO "*** STARVING *** " + margin_str. }
        PRINT AT(0, row):
            "  MARGIN       : " + margin_str + "          ".
        SET row TO row + 1.
        PRINT AT(0, row):
            "  LA MARGIN    : " + ROUND(TELEM_EM_LOOKAHEAD_MARGIN, 4) +
            " u/s (" + EM_LOOKAHEAD_S + "s ahead)          ".
        SET row TO row + 1.
        PRINT AT(0, row): "".
        SET row TO row + 1.

        PRINT AT(0, row):
            "  Spool lag    : " + ROUND(TELEM_EM_WORST_SPOOL_LAG, 2) + " s           ".
        SET row TO row + 1.
        PRINT AT(0, row): "".
        SET row TO row + 1.

        // ------ Live cross-check: compare ffwd demand vs live REQUIREDFLOW ------
        PRINT AT(0, row): "  CROSS-CHECK (ffwd vs live REQUIREDFLOW):        ".
        SET row TO row + 1.
        SET di TO 0.
        LIST ENGINES IN _chk_eng_list.
        UNTIL di >= _chk_eng_list:LENGTH {
            LOCAL eng IS _chk_eng_list[di].
            LOCAL cr  IS eng:CONSUMEDRESOURCES.
            IF cr:HASKEY("IntakeAir") {
                LOCAL live_req IS cr["IntakeAir"]:REQUIREDFLOW.
                LOCAL live_ff  IS cr["IntakeAir"]:FUELFLOW.
                LOCAL pct IS 0.
                IF live_req > 0.0001 {
                    SET pct TO ROUND(live_ff / live_req * 100, 1).
                }
                PRINT AT(0, row):
                    "  eng[" + di + "] req=" + ROUND(live_req,4) +
                    " got=" + ROUND(live_ff,4) +
                    " sat=" + pct + "%          ".
                SET row TO row + 1.
            }
            SET di TO di + 1.
        }
    }

    WAIT 0.02.
}
