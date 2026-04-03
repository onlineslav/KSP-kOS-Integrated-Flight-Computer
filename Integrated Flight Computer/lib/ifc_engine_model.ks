@LAZYGLOBAL OFF.
// ============================================================
// ifc_engine_model.ks  -  Integrated Flight Computer
//
// Engine feedforward model:
//   - Identifies all air-breathing engines on SHIP
//   - Determines scale factor via TweakScale or thrust-ratio
//   - Simulates spool dynamics (first-order exponential)
//   - Predicts IntakeAir demand from throttle + Mach + P_atm
//   - Estimates IntakeAir supply from installed intakes
//   - Reports starvation margin and lookahead margin
//
// Public API:
//   EM_INIT()                  - call once at flight start
//   EM_TICK()                  - call every main-loop cycle
//   EM_GET_MARGIN()            - supply - demand (units/s)
//   EM_GET_LOOKAHEAD_MARGIN()  - margin at lookahead altitude
//   EM_GET_SPOOL_LAG_S()       - seconds to reach THROTTLE_CMD
//
// Ownership:
//   Reads:  THROTTLE_CMD, IFC_ACTUAL_DT (globals from state/autothrottle)
//   Writes: TELEM_EM_* globals only
//   Calls:  IFC_SET_ALERT, IFC_PUSH_EVENT (from ifc_helpers)
//
// Load order: after ifc_constants.ks, ifc_state.ks, ifc_helpers.ks
// ============================================================

// ------------------------------------------------------------
// File-scope state (persists for flight lifetime)
// ------------------------------------------------------------
LOCAL _em_db             IS LEXICON().
LOCAL _em_engine_list    IS LIST().
LOCAL _em_intake_list    IS LIST().
LOCAL _em_spool_states   IS LIST().
LOCAL _em_initialized    IS FALSE.
LOCAL _em_last_warn_ut   IS -99.
LOCAL _em_any_db_intake  IS FALSE.

// ============================================================
// PUBLIC API
// ============================================================

FUNCTION EM_INIT {
    _EM_LOAD_DB().

    SET _em_engine_list  TO LIST().
    SET _em_intake_list  TO LIST().
    SET _em_spool_states TO LIST().
    SET _em_any_db_intake TO FALSE.

    // -- Enumerate air-breathing engines (ONE call to LIST ENGINES) --
    LIST ENGINES IN _em_raw_eng_list.
    LOCAL ei IS 0.
    UNTIL ei >= _em_raw_eng_list:LENGTH {
        LOCAL eng IS _em_raw_eng_list[ei].
        LOCAL is_air IS FALSE.
        IF eng:CONSUMEDRESOURCES:HASKEY("IntakeAir") {
            SET is_air TO TRUE.
        }
        IF is_air {
            LOCAL desc IS _EM_BUILD_ENGINE_DESCRIPTOR(eng).
            _em_engine_list:ADD(desc).
            _em_spool_states:ADD(THROTTLE_CMD).
        }
        SET ei TO ei + 1.
    }

    // -- Enumerate intake parts --
    LOCAL intake_mods IS SHIP:MODULESNAMED("ModuleResourceIntake").
    LOCAL ii IS 0.
    UNTIL ii >= intake_mods:LENGTH {
        LOCAL intake_m IS intake_mods[ii].
        LOCAL desc IS _EM_BUILD_INTAKE_DESCRIPTOR(intake_m).
        _em_intake_list:ADD(desc).
        IF desc["in_db"] { SET _em_any_db_intake TO TRUE. }
        SET ii TO ii + 1.
    }

    SET TELEM_EM_DB_LOADED    TO _em_db:HASKEY("engines").
    SET TELEM_EM_ENG_COUNT    TO _em_engine_list:LENGTH.
    SET TELEM_EM_INTAKE_COUNT TO _em_intake_list:LENGTH.

    IFC_PUSH_EVENT("EM: init done — " + _em_engine_list:LENGTH + " eng, " + _em_intake_list:LENGTH + " intakes").

    IF _em_engine_list:LENGTH = 0 {
        IFC_SET_ALERT("EM: no air-breathing engines found", "WARN").
    }

    SET _em_initialized TO TRUE.
}

// ------------------------------------------------------------

FUNCTION EM_TICK {
    IF NOT _em_initialized { RETURN. }

    // -- Atmosphere --
    LOCAL mach IS 0.
    IF SHIP:SOUNDSPEED > 1 { SET mach TO SHIP:AIRSPEED / SHIP:SOUNDSPEED. }
    LOCAL p_atm  IS SHIP:BODY:ATM:ALTITUDEPRESSURE(SHIP:ALTITUDE).
    LOCAL rho    IS EM_KSL_RHO * p_atm.
    LOCAL thr    IS THROTTLE_CMD.
    LOCAL dt     IS IFC_ACTUAL_DT.

    LOCAL q_demand   IS 0.
    LOCAL q_supply   IS 0.
    LOCAL worst_lag  IS 0.

    // -- Per-engine loop --
    LOCAL i IS 0.
    UNTIL i >= _em_engine_list:LENGTH {
        LOCAL ed IS _em_engine_list[i].

        // Spool simulation
        _EM_COMPUTE_SPOOL(i, thr, dt).
        LOCAL spool IS _em_spool_states[i].

        // IntakeAir demand
        LOCAL q_eng IS _EM_DEMAND_SINGLE(ed, thr, mach, p_atm).
        SET q_demand TO q_demand + q_eng.

        // Spool lag estimate
        LOCAL delta IS ABS(thr - spool).
        IF delta > 0.01 {
            LOCAL me IS _EM_GET_ACTIVE_MODE_ENTRY(ed).
            LOCAL k_eff IS me["k_down"].
            IF thr > spool { SET k_eff TO me["k_up"]. }
            IF k_eff > 0.001 {
                LOCAL lag IS -LN(0.01 / delta) / k_eff.
                IF lag > worst_lag { SET worst_lag TO lag. }
            }
        }

        SET i TO i + 1.
    }

    // -- Per-intake loop --
    SET i TO 0.
    UNTIL i >= _em_intake_list:LENGTH {
        LOCAL id IS _em_intake_list[i].
        _EM_UPDATE_INTAKE_OPEN(id).
        SET q_supply TO q_supply + _EM_SUPPLY_SINGLE(id, mach, rho).
        SET i TO i + 1.
    }

    // Fallback if no intake is in DB: use live aggregate buffer drain rate
    IF NOT _em_any_db_intake AND q_demand > 0 AND dt > 0.001 {
        // Approximate: buffer per tick / dt
        // SHIP:INTAKEAIR:AMOUNT is current buffer in units
        SET q_supply TO SHIP:INTAKEAIR:AMOUNT / dt.
    }

    // -- Margin --
    LOCAL margin IS q_supply - q_demand.

    // -- Predictive lookahead --
    LOCAL future_alt IS SHIP:ALTITUDE + SHIP:VERTICALSPEED * EM_LOOKAHEAD_S.
    LOCAL future_p   IS SHIP:BODY:ATM:ALTITUDEPRESSURE(future_alt).
    LOCAL future_rho IS EM_KSL_RHO * future_p.
    LOCAL q_supply_la IS 0.
    SET i TO 0.
    UNTIL i >= _em_intake_list:LENGTH {
        LOCAL id IS _em_intake_list[i].
        SET q_supply_la TO q_supply_la + _EM_SUPPLY_SINGLE(id, mach, future_rho).
        SET i TO i + 1.
    }
    LOCAL la_margin IS q_supply_la - q_demand.

    // -- Starvation alert (rate-limited) --
    IF margin < EM_STARVATION_WARN_MARGIN {
        IF TIME:SECONDS - _em_last_warn_ut > EM_STARVATION_WARN_RATE_S {
            IFC_SET_ALERT("AIR STARVE " + ROUND(margin, 2) + " u/s", "WARN").
            SET _em_last_warn_ut TO TIME:SECONDS.
        }
    }

    // -- Write telemetry --
    SET TELEM_EM_Q_DEMAND         TO q_demand.
    SET TELEM_EM_Q_SUPPLY         TO q_supply.
    SET TELEM_EM_MARGIN           TO margin.
    SET TELEM_EM_LOOKAHEAD_MARGIN TO la_margin.
    SET TELEM_EM_WORST_SPOOL_LAG  TO worst_lag.
    SET TELEM_EM_MACH             TO mach.
    SET TELEM_EM_P_ATM            TO p_atm.
    SET TELEM_EM_RHO              TO rho.
    SET TELEM_EM_STARVING         TO CHOOSE 1 IF margin < 0 ELSE 0.
}

// ------------------------------------------------------------

FUNCTION EM_GET_MARGIN          { RETURN TELEM_EM_MARGIN. }
FUNCTION EM_GET_LOOKAHEAD_MARGIN { RETURN TELEM_EM_LOOKAHEAD_MARGIN. }
FUNCTION EM_GET_SPOOL_LAG_S     { RETURN TELEM_EM_WORST_SPOOL_LAG. }

// ============================================================
// INTERNAL HELPERS
// ============================================================

FUNCTION _EM_LOAD_DB {
    LOCAL db_path IS "0:/Integrated Flight Computer/data/engine_database.json".
    IF NOT EXISTS(db_path) {
        IFC_SET_ALERT("EM: DB not found at " + db_path, "WARN").
        SET _em_db TO LEXICON("engines", LEXICON(), "intakes", LEXICON()).
        RETURN.
    }
    SET _em_db TO READJSON(db_path).
    IF NOT _em_db:HASKEY("engines") OR NOT _em_db:HASKEY("intakes") {
        IFC_SET_ALERT("EM: DB missing engines/intakes keys", "WARN").
        SET _em_db TO LEXICON("engines", LEXICON(), "intakes", LEXICON()).
    }
}

// ------------------------------------------------------------
// _EM_CURVE_EVAL
// Piecewise linear interpolation between [x, y, in_t, out_t] key arrays.
// Returns 1.0 on empty curve (safe neutral value for multipliers).
// Clamps outside range to first/last key y value.
// ------------------------------------------------------------
FUNCTION _EM_CURVE_EVAL {
    PARAMETER curve_keys.
    PARAMETER cx.

    LOCAL klen IS curve_keys:LENGTH.
    IF klen = 0 { RETURN 1.0. }
    IF klen = 1 { RETURN curve_keys[0][1]. }

    IF cx <= curve_keys[0][0]          { RETURN curve_keys[0][1]. }
    IF cx >= curve_keys[klen-1][0]     { RETURN curve_keys[klen-1][1]. }

    // Find segment
    LOCAL j IS 0.
    UNTIL j >= klen - 2 {
        IF cx < curve_keys[j+1][0] { BREAK. }
        SET j TO j + 1.
    }

    LOCAL x0 IS curve_keys[j][0].
    LOCAL y0 IS curve_keys[j][1].
    LOCAL x1 IS curve_keys[j+1][0].
    LOCAL y1 IS curve_keys[j+1][1].
    LOCAL dx IS x1 - x0.
    IF ABS(dx) < 1e-9 { RETURN y0. }

    LOCAL t IS (cx - x0) / dx.
    RETURN y0 + t * (y1 - y0).
}

// ------------------------------------------------------------
// _EM_DETECT_SCALE
// Returns diameter scale factor relative to defaultDiameter.
// Tries TweakScale module first, then thrust-ratio inference, then 1.0.
// ------------------------------------------------------------
FUNCTION _EM_DETECT_SCALE {
    PARAMETER eng.
    PARAMETER db_entry.

    LOCAL default_d IS db_entry["defaultDiameter"].

    // Step 1: TweakScale module query
    LOCAL ts_mods IS SHIP:MODULESNAMED("TweakScale").
    LOCAL mi IS 0.
    UNTIL mi >= ts_mods:LENGTH {
        LOCAL ts_m IS ts_mods[mi].
        IF ts_m:PART = eng:PART {
            LOCAL field_names IS LIST("Scale", "currentScale", "Current Scale").
            LOCAL fi IS 0.
            UNTIL fi >= field_names:LENGTH {
                IF ts_m:HASFIELD(field_names[fi]) {
                    LOCAL raw IS ("" + ts_m:GETFIELD(field_names[fi])):TONUMBER(-1).
                    IF raw > 0 AND default_d > 0 {
                        RETURN CLAMP(raw / default_d, EM_SCALE_MIN, EM_SCALE_MAX).
                    }
                }
                SET fi TO fi + 1.
            }
        }
        SET mi TO mi + 1.
    }

    // Step 2: Thrust-ratio inference
    // eng:MAXTHRUST at ground = maxThrust * velCurve(0) * atmCurve(1.0)
    // Use stored vel_at_zero and atm_at_one to correct for curve offset
    LOCAL db_thrust IS db_entry["maxThrust"].
    IF db_thrust > 0 AND eng:MAXTHRUST > 0 {
        LOCAL vel0 IS db_entry["vel_at_zero"].
        LOCAL atm1 IS db_entry["atm_at_one"].
        LOCAL corrected_ref IS db_thrust * vel0 * atm1.
        IF corrected_ref > 0 {
            LOCAL ratio IS eng:MAXTHRUST / corrected_ref.
            IF ratio > 0 {
                RETURN CLAMP(SQRT(ratio), EM_SCALE_MIN, EM_SCALE_MAX).
            }
        }
    }

    RETURN 1.0.
}

// ------------------------------------------------------------
// _EM_GET_ACTIVE_MODE_ENTRY
// Returns the DB entry to use for the current engine mode.
// For single-mode: returns the top-level entry directly.
// For multimode: looks up eng:MODE in modes lexicon.
// ------------------------------------------------------------
FUNCTION _EM_GET_ACTIVE_MODE_ENTRY {
    PARAMETER ed.  // engine descriptor lexicon

    IF NOT ed["is_mm"] { RETURN ed["db_entry"]. }
    IF NOT ed["eng_ref"]:IGNITION { RETURN ed["db_entry"]. }

    // Flattened DB: mode entries are stored as "partName__ModeName"
    LOCAL mode_name IS ed["eng_ref"]:MODE.
    LOCAL flat_key  IS ed["part_name"] + "__" + mode_name.
    LOCAL eng_db    IS _em_db["engines"].

    IF eng_db:HASKEY(flat_key) { RETURN eng_db[flat_key]. }

    // Mode not found — one-shot warning, return top-level entry
    IF NOT ed["warn_issued"] {
        IFC_PUSH_EVENT("EM: unknown mode '" + mode_name + "' for " + ed["part_name"]).
        SET ed["warn_issued"] TO TRUE.
    }
    RETURN ed["db_entry"].
}

// ------------------------------------------------------------
// _EM_BUILD_ENGINE_DESCRIPTOR
// ------------------------------------------------------------
FUNCTION _EM_BUILD_ENGINE_DESCRIPTOR {
    PARAMETER eng.

    LOCAL part_name IS eng:PART:NAME.
    LOCAL eng_db    IS _em_db["engines"].
    LOCAL in_db     IS eng_db:HASKEY(part_name).
    LOCAL db_entry  IS LEXICON().
    LOCAL scale_f   IS 1.0.
    LOCAL is_mm     IS FALSE.

    IF in_db {
        SET db_entry TO eng_db[part_name].
        SET scale_f  TO _EM_DETECT_SCALE(eng, db_entry).
        SET is_mm    TO db_entry["multimode"].
    }

    RETURN LEXICON(
        "eng_ref",     eng,
        "part_name",   part_name,
        "db_entry",    db_entry,
        "scale",       scale_f,
        "in_db",       in_db,
        "is_mm",       is_mm,
        "warn_issued", FALSE
    ).
}

// ------------------------------------------------------------
// _EM_BUILD_INTAKE_DESCRIPTOR
// Accepts a ModuleResourceIntake module reference.
// ------------------------------------------------------------
FUNCTION _EM_BUILD_INTAKE_DESCRIPTOR {
    PARAMETER intake_mod.

    LOCAL prt       IS intake_mod:PART.
    LOCAL part_name IS prt:NAME.
    LOCAL int_db    IS _em_db["intakes"].
    LOCAL in_db     IS int_db:HASKEY(part_name).
    LOCAL db_entry  IS LEXICON().

    IF in_db { SET db_entry TO int_db[part_name]. }

    RETURN LEXICON(
        "part_ref",  prt,
        "mod_ref",   intake_mod,
        "part_name", part_name,
        "db_entry",  db_entry,
        "in_db",     in_db,
        "open",      TRUE
    ).
}

// ------------------------------------------------------------
// _EM_UPDATE_INTAKE_OPEN
// Reads module field to determine if intake is open.
// AMBIGUOUS: field name and value strings must be verified in-game.
// Assumed: field "Intake" returns "Open" or "Closed".
// ------------------------------------------------------------
FUNCTION _EM_UPDATE_INTAKE_OPEN {
    PARAMETER id.

    LOCAL intake_m IS id["mod_ref"].
    IF intake_m:HASFIELD("Intake") {
        LOCAL val IS ("" + intake_m:GETFIELD("Intake")):TOLOWER.
        SET id["open"] TO val <> "closed".
    } ELSE {
        // Field name not found — assume open; log once
        SET id["open"] TO TRUE.
    }
}

// ------------------------------------------------------------
// _EM_COMPUTE_SPOOL
// First-order exponential approach toward throttle_cmd.
// Updates _em_spool_states[idx] in place.
// ------------------------------------------------------------
FUNCTION _EM_COMPUTE_SPOOL {
    PARAMETER idx.
    PARAMETER throttle_cmd.
    PARAMETER dt.

    LOCAL ed      IS _em_engine_list[idx].
    LOCAL current IS _em_spool_states[idx].
    LOCAL me      IS _EM_GET_ACTIVE_MODE_ENTRY(ed).
    LOCAL k_use   IS me["k_down"].
    IF throttle_cmd > current { SET k_use TO me["k_up"]. }
    LOCAL new_val IS current + (throttle_cmd - current) * k_use * dt.
    SET _em_spool_states[idx] TO CLAMP(new_val, 0.0, 1.0).
}

// ------------------------------------------------------------
// _EM_DEMAND_SINGLE
// IntakeAir demand in units/s for one engine at given conditions.
// Returns 0 if engine is off or this mode uses no IntakeAir.
// ------------------------------------------------------------
FUNCTION _EM_DEMAND_SINGLE {
    PARAMETER ed.
    PARAMETER throttle_cmd.
    PARAMETER mach.
    PARAMETER p_atm.

    LOCAL eng IS ed["eng_ref"].
    IF eng:FLAMEOUT OR NOT eng:IGNITION { RETURN 0. }

    // Unknown engine: fall back to live runtime data
    IF NOT ed["in_db"] {
        IF NOT ed["warn_issued"] {
            IFC_PUSH_EVENT("EM: unknown eng '" + ed["part_name"] + "' — using live REQUIREDFLOW").
            SET ed["warn_issued"] TO TRUE.
        }
        LOCAL cr IS eng:CONSUMEDRESOURCES.
        IF cr:HASKEY("IntakeAir") { RETURN cr["IntakeAir"]:REQUIREDFLOW. }
        RETURN 0.
    }

    LOCAL me IS _EM_GET_ACTIVE_MODE_ENTRY(ed).

    // Skip if this mode consumes no IntakeAir (e.g. RAPIER closed-cycle)
    IF me["ratio_ia"] <= 0 { RETURN 0. }

    LOCAL scale IS ed["scale"].
    LOCAL scale2 IS scale * scale.  // thrust scales as D²

    LOCAL vel_m  IS _EM_CURVE_EVAL(me["velCurve"], mach).
    LOCAL atm_m  IS _EM_CURVE_EVAL(me["atmCurve"], p_atm).
    LOCAL thrust IS me["maxThrust"] * scale2 * throttle_cmd * vel_m * atm_m.  // kN

    // Mass flow: ṁ = F(kN)*1000 / (Isp * g0)   [kg/s]
    LOCAL isp IS me["isp_vac"].
    IF isp < 1 { RETURN 0. }
    LOCAL mdot IS (thrust * 1000.0) / (isp * EM_G0).  // kg/s

    // IntakeAir fraction by mass (density equal to LF, so mass ratio = volume ratio)
    LOCAL r_ia IS me["ratio_ia"].
    LOCAL r_lf IS me["ratio_lf"].
    LOCAL ia_frac IS r_ia / (r_ia + r_lf).
    LOCAL mdot_ia IS mdot * ia_frac.  // kg/s IntakeAir

    // Convert to units/s
    RETURN mdot_ia / EM_IA_DENSITY.
}

// ------------------------------------------------------------
// _EM_SUPPLY_SINGLE
// IntakeAir supply in units/s from one intake at given conditions.
// Returns 0 if intake is closed or not in DB.
// AMBIGUOUS: formula units (units/s vs kg/s) must be verified in-game.
// ------------------------------------------------------------
FUNCTION _EM_SUPPLY_SINGLE {
    PARAMETER id.
    PARAMETER mach.
    PARAMETER rho.

    IF NOT id["open"]  { RETURN 0. }
    IF NOT id["in_db"] { RETURN 0. }

    LOCAL de  IS id["db_entry"].
    LOCAL eff IS _EM_CURVE_EVAL(de["machCurve"], mach).
    // Formula assumed to give units/s directly.
    // If this turns out to give kg/s, divide by EM_IA_DENSITY here.
    RETURN de["area"] * de["intakeSpeed"] * rho * eff.
}
