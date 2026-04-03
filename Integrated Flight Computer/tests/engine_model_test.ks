@LAZYGLOBAL OFF.

// ============================================================
// engine_model_test.ks  -  Engine Model Standalone Test + CSV logger
//
// Purpose:
//   - Run ifc_engine_model in live flight
//   - Display key model telemetry
//   - Log per-engine validation rows to:
//       0:/Integrated Flight Computer/logs/
//
// Notes:
//   - Read-only test script: no steering/throttle control locks
//   - Designed to be tolerant of kOS suffix differences across builds
// ============================================================

LOCAL ifc_root IS "0:/Integrated Flight Computer/".
GLOBAL EMT_ROOT IS ifc_root.
GLOBAL EMT_LOG_DIR IS EMT_ROOT + "logs".
GLOBAL EMT_COUNTER_PATH IS EMT_LOG_DIR + "/engine_model_test_counter.txt".
GLOBAL EMT_LOG_FILE IS "".
GLOBAL EMT_LOG_ACTIVE IS FALSE.
GLOBAL EMT_LOG_SAMPLE_DT IS 0.10. // 10 Hz CSV logging
GLOBAL EMT_NEXT_LOG_UT IS 0.
GLOBAL EMT_SAMPLE_IDX IS 0.
GLOBAL EMT_LOG_ROWS IS 0.

RUNPATH(ifc_root + "lib/ifc_constants.ks").
RUNPATH(ifc_root + "lib/ifc_state.ks").
RUNPATH(ifc_root + "lib/ifc_helpers.ks").
RUNPATH(ifc_root + "lib/ifc_engine_model.ks").

IF NOT (DEFINED ACTIVE_AIRCRAFT) {
    GLOBAL ACTIVE_AIRCRAFT IS LEXICON("name", "EngModelTest").
}
GLOBAL THROTTLE_CMD IS SHIP:CONTROL:PILOTMAINTHROTTLE.
SET IFC_ACTUAL_DT TO 0.05.

FUNCTION EMT_CLAMP {
    PARAMETER x.
    PARAMETER lo.
    PARAMETER hi.
    IF x < lo { RETURN lo. }
    IF x > hi { RETURN hi. }
    RETURN x.
}

FUNCTION EMT_SAFE_CSV_TEXT {
    PARAMETER raw_txt.
    LOCAL s IS "" + raw_txt.
    SET s TO s:REPLACE(",", "_").
    SET s TO s:REPLACE(CHAR(10), " ").
    SET s TO s:REPLACE(CHAR(13), " ").
    RETURN s:TRIM.
}

FUNCTION EMT_GET_ENGINE_NAME {
    PARAMETER eng.
    LOCAL nm IS "".
    IF eng:HASSUFFIX("PART") {
        LOCAL p IS eng:PART.
        IF p:HASSUFFIX("NAME") { SET nm TO p:NAME. }
    }
    IF nm = "" AND eng:HASSUFFIX("NAME") { SET nm TO eng:NAME. }
    IF nm = "" { SET nm TO "UNKNOWN_ENGINE". }
    RETURN EMT_SAFE_CSV_TEXT(nm).
}

FUNCTION EMT_GET_ENGINE_MODE {
    PARAMETER eng.
    IF eng:HASSUFFIX("MODE") { RETURN EMT_SAFE_CSV_TEXT(eng:MODE). }
    RETURN "".
}

FUNCTION EMT_GET_INTAKEAIR_BUFFER {
    IF NOT SHIP:HASSUFFIX("INTAKEAIR") { RETURN 0. }
    LOCAL ia_buf IS SHIP:INTAKEAIR.
    IF ia_buf:HASSUFFIX("AMOUNT") { RETURN ia_buf:AMOUNT. }
    RETURN ia_buf.
}

FUNCTION EMT_BUILD_LOG_FILE {
    IF NOT EXISTS(EMT_LOG_DIR) { CREATEDIR(EMT_LOG_DIR). }

    LOCAL seq IS 1.
    IF EXISTS(EMT_COUNTER_PATH) {
        LOCAL counter_str IS OPEN(EMT_COUNTER_PATH):READALL:STRING:TRIM.
        IF counter_str:LENGTH > 0 {
            LOCAL parsed IS counter_str:TONUMBER(0).
            IF parsed >= 1 { SET seq TO ROUND(parsed). }
        }
    }
    IF EXISTS(EMT_COUNTER_PATH) { DELETEPATH(EMT_COUNTER_PATH). }
    LOG (seq + 1) TO EMT_COUNTER_PATH.

    LOCAL seq_str IS "" + seq.
    UNTIL seq_str:LENGTH >= 8 { SET seq_str TO "0" + seq_str. }
    RETURN EMT_LOG_DIR + "/engine_model_validation_" + seq_str + ".csv".
}

FUNCTION EMT_WRITE_LOG_HEADER {
    LOCAL header IS
        "ut_s,sample_idx,engine_idx,engine_name,engine_mode,engine_multimode,engine_ignition,engine_flameout,engine_thrust_kn,engine_avail_thrust_kn,engine_maxthrust_kn,model_engine_thrust_kn,thrust_error_kn,model_engine_q_demand_u_s,intake_req_flow_u_s,intake_got_flow_u_s,intake_sat_pct,model_spool_01,throttle_cmd_01,pilot_throttle_01,mach,p_atm_atm,rho_kg_m3,q_demand_u_s,q_supply_u_s,margin_u_s,lookahead_margin_u_s,worst_spool_lag_s,starving_flag,ship_intakeair_buffer_u,ship_thrust_kn,ship_avail_thrust_kn,altitude_m,vertical_speed_m_s,airspeed_m_s,sound_speed_m_s,ifc_dt_s,db_loaded,model_eng_count,model_intake_count,vessel_name".
    LOG header TO EMT_LOG_FILE.
}

FUNCTION EMT_GET_PILOT_THROTTLE {
    IF SHIP:HASSUFFIX("CONTROL") {
        LOCAL c IS SHIP:CONTROL.
        IF c:HASSUFFIX("PILOTMAINTHROTTLE") { RETURN EMT_CLAMP(c:PILOTMAINTHROTTLE, 0, 1). }
        IF c:HASSUFFIX("MAINTHROTTLE") { RETURN EMT_CLAMP(c:MAINTHROTTLE, 0, 1). }
    }
    RETURN EMT_CLAMP(THROTTLE_CMD, 0, 1).
}

FUNCTION EMT_GET_SHIP_THRUST {
    IF SHIP:HASSUFFIX("THRUST") { RETURN SHIP:THRUST. }
    RETURN -1.
}

FUNCTION EMT_GET_SHIP_AVAIL_THRUST {
    IF SHIP:HASSUFFIX("AVAILABLETHRUST") { RETURN SHIP:AVAILABLETHRUST. }
    RETURN -1.
}

FUNCTION EMT_GET_SHIP_AIRSPEED {
    IF SHIP:HASSUFFIX("AIRSPEED") { RETURN SHIP:AIRSPEED. }
    RETURN -1.
}

FUNCTION EMT_GET_SHIP_SOUNDSPEED {
    IF SHIP:HASSUFFIX("SOUNDSPEED") { RETURN SHIP:SOUNDSPEED. }
    RETURN -1.
}

FUNCTION EMT_GET_ENG_THRUST {
    PARAMETER eng.
    IF eng:HASSUFFIX("THRUST") { RETURN eng:THRUST. }
    RETURN -1.
}

FUNCTION EMT_GET_ENG_AVAIL_THRUST {
    PARAMETER eng.
    IF eng:HASSUFFIX("AVAILABLETHRUST") { RETURN eng:AVAILABLETHRUST. }
    RETURN -1.
}

FUNCTION EMT_GET_ENG_MAXTHRUST {
    PARAMETER eng.
    IF eng:HASSUFFIX("MAXTHRUST") { RETURN eng:MAXTHRUST. }
    RETURN -1.
}

FUNCTION EMT_GET_ENG_MM01 {
    PARAMETER eng.
    IF eng:HASSUFFIX("MULTIMODE") { RETURN CHOOSE 1 IF eng:MULTIMODE ELSE 0. }
    RETURN -1.
}

FUNCTION EMT_GET_ENG_IGN01 {
    PARAMETER eng.
    IF eng:HASSUFFIX("IGNITION") { RETURN CHOOSE 1 IF eng:IGNITION ELSE 0. }
    RETURN -1.
}

FUNCTION EMT_GET_ENG_FLAMEOUT01 {
    PARAMETER eng.
    IF eng:HASSUFFIX("FLAMEOUT") { RETURN CHOOSE 1 IF eng:FLAMEOUT ELSE 0. }
    RETURN -1.
}

FUNCTION EMT_LOG_SAMPLE {
    PARAMETER dt.

    LOCAL now IS TIME:SECONDS.
    LOCAL pilot_thr IS EMT_GET_PILOT_THROTTLE().
    LOCAL ship_thr IS EMT_GET_SHIP_THRUST().
    LOCAL ship_avail IS EMT_GET_SHIP_AVAIL_THRUST().
    LOCAL ia_buf IS EMT_GET_INTAKEAIR_BUFFER().
    LOCAL aspd IS EMT_GET_SHIP_AIRSPEED().
    LOCAL sspd IS EMT_GET_SHIP_SOUNDSPEED().
    LOCAL db_loaded_i IS CHOOSE 1 IF TELEM_EM_DB_LOADED ELSE 0.
    LOCAL vessel_name IS EMT_SAFE_CSV_TEXT(SHIP:NAME).

    LOCAL eng_list IS SHIP:ENGINES.
    LOCAL i IS 0.
    UNTIL i >= eng_list:LENGTH {
        LOCAL eng IS eng_list[i].
        LOCAL cr IS LEXICON().
        IF eng:HASSUFFIX("CONSUMEDRESOURCES") {
            SET cr TO eng:CONSUMEDRESOURCES.
        }

        LOCAL req IS -1.
        LOCAL got IS -1.
        LOCAL sat IS -1.
        IF cr:HASKEY("IntakeAir") {
            LOCAL ia IS cr["IntakeAir"].
            IF ia:HASSUFFIX("REQUIREDFLOW") { SET req TO ia:REQUIREDFLOW. }
            IF ia:HASSUFFIX("FUELFLOW") { SET got TO ia:FUELFLOW. }
            IF req > 0.0001 AND got >= 0 {
                SET sat TO got / req * 100.
            }
        }

        LOCAL spool IS -1.
        LOCAL model_t IS -1.
        LOCAL model_q IS -1.
        LOCAL live_t IS EMT_GET_ENG_THRUST(eng).
        IF i < TELEM_EM_ENG_COUNT { SET spool TO EM_GET_SPOOL_AT(i). }
        IF i < TELEM_EM_ENG_COUNT { SET model_t TO EM_GET_MODEL_THRUST_AT(i). }
        IF i < TELEM_EM_ENG_COUNT { SET model_q TO EM_GET_MODEL_Q_DEMAND_AT(i). }
        LOCAL thrust_err IS -999.
        IF live_t >= 0 AND model_t >= 0 { SET thrust_err TO live_t - model_t. }

        LOCAL row IS
            now + "," +
            EMT_SAMPLE_IDX + "," +
            i + "," +
            EMT_GET_ENGINE_NAME(eng) + "," +
            EMT_GET_ENGINE_MODE(eng) + "," +
            EMT_GET_ENG_MM01(eng) + "," +
            EMT_GET_ENG_IGN01(eng) + "," +
            EMT_GET_ENG_FLAMEOUT01(eng) + "," +
            live_t + "," +
            EMT_GET_ENG_AVAIL_THRUST(eng) + "," +
            EMT_GET_ENG_MAXTHRUST(eng) + "," +
            model_t + "," +
            thrust_err + "," +
            model_q + "," +
            req + "," +
            got + "," +
            sat + "," +
            spool + "," +
            THROTTLE_CMD + "," +
            pilot_thr + "," +
            TELEM_EM_MACH + "," +
            TELEM_EM_P_ATM + "," +
            TELEM_EM_RHO + "," +
            TELEM_EM_Q_DEMAND + "," +
            TELEM_EM_Q_SUPPLY + "," +
            TELEM_EM_MARGIN + "," +
            TELEM_EM_LOOKAHEAD_MARGIN + "," +
            TELEM_EM_WORST_SPOOL_LAG + "," +
            TELEM_EM_STARVING + "," +
            ia_buf + "," +
            ship_thr + "," +
            ship_avail + "," +
            SHIP:ALTITUDE + "," +
            SHIP:VERTICALSPEED + "," +
            aspd + "," +
            sspd + "," +
            dt + "," +
            db_loaded_i + "," +
            TELEM_EM_ENG_COUNT + "," +
            TELEM_EM_INTAKE_COUNT + "," +
            vessel_name.

        LOG row TO EMT_LOG_FILE.
        SET EMT_LOG_ROWS TO EMT_LOG_ROWS + 1.
        SET i TO i + 1.
    }

    SET EMT_SAMPLE_IDX TO EMT_SAMPLE_IDX + 1.
}

FUNCTION EMT_PRINT_ENGINE_CROSSCHECK {
    PRINT "  CROSS-CHECK (ffwd vs live REQUIREDFLOW):".
    LOCAL eng_list IS SHIP:ENGINES.
    LOCAL i IS 0.
    UNTIL i >= eng_list:LENGTH {
        LOCAL eng IS eng_list[i].
        LOCAL cr IS LEXICON().
        IF eng:HASSUFFIX("CONSUMEDRESOURCES") {
            SET cr TO eng:CONSUMEDRESOURCES.
        }
        IF cr:HASKEY("IntakeAir") {
            LOCAL ia IS cr["IntakeAir"].
            LOCAL live_req IS 0.
            LOCAL live_ff IS 0.
            IF ia:HASSUFFIX("REQUIREDFLOW") { SET live_req TO ia:REQUIREDFLOW. }
            IF ia:HASSUFFIX("FUELFLOW") { SET live_ff TO ia:FUELFLOW. }
            LOCAL pct IS 0.
            IF live_req > 0.0001 {
                SET pct TO ROUND(live_ff / live_req * 100, 1).
            }
            PRINT "  eng[" + i + "] req=" + ROUND(live_req,4) +
                  " got=" + ROUND(live_ff,4) +
                  " sat=" + pct + "%".
        }
        SET i TO i + 1.
    }
}

FUNCTION EMT_PRINT_SCREEN {
    CLEARSCREEN.
    PRINT "======================================================".
    PRINT " ENGINE MODEL TEST  -  " + SHIP:NAME.
    PRINT "======================================================".
    PRINT "".
    PRINT "  [THROTTLE] adjust demand  |  [ABORT] exit".
    PRINT "------------------------------------------------------".
    PRINT "Log file     : " + EMT_LOG_FILE.
    PRINT "Log rows     : " + EMT_LOG_ROWS.
    PRINT "Log dt       : " + EMT_LOG_SAMPLE_DT + " s".
    PRINT "".
    PRINT "DB loaded    : " + TELEM_EM_DB_LOADED.
    PRINT "Engines found: " + TELEM_EM_ENG_COUNT.
    PRINT "Intakes found: " + TELEM_EM_INTAKE_COUNT.
    PRINT "------------------------------------------------------".
    PRINT "  Throttle cmd : " + ROUND(THROTTLE_CMD * 100, 1) + " %".
    PRINT "  Pilot thr    : " + ROUND(EMT_GET_PILOT_THROTTLE() * 100, 1) + " %".
    PRINT "  Mach         : " + ROUND(TELEM_EM_MACH, 3).
    PRINT "  P_atm        : " + ROUND(TELEM_EM_P_ATM, 5) + " atm".
    PRINT "  Rho          : " + ROUND(TELEM_EM_RHO, 4) + " kg/m^3".
    PRINT "".
    PRINT "  IA DEMAND    : " + ROUND(TELEM_EM_Q_DEMAND, 4) + " u/s".
    PRINT "  IA SUPPLY    : " + ROUND(TELEM_EM_Q_SUPPLY, 4) + " u/s".
    LOCAL margin_str IS ROUND(TELEM_EM_MARGIN, 4) + " u/s".
    IF TELEM_EM_STARVING = 1 { SET margin_str TO "*** STARVING *** " + margin_str. }
    PRINT "  MARGIN       : " + margin_str.
    PRINT "  LA MARGIN    : " + ROUND(TELEM_EM_LOOKAHEAD_MARGIN, 4) +
          " u/s (" + EM_LOOKAHEAD_S + "s ahead)".
    PRINT "  Spool lag    : " + ROUND(TELEM_EM_WORST_SPOOL_LAG, 2) + " s".
    PRINT "  IA buffer    : " + ROUND(EMT_GET_INTAKEAIR_BUFFER(), 4) + " u".
    PRINT "  Ship thrust  : " + ROUND(EMT_GET_SHIP_THRUST(), 2) + " kN".
    IF TELEM_EM_ENG_COUNT > 0 AND SHIP:ENGINES:LENGTH > 0 {
        LOCAL live_t0 IS EMT_GET_ENG_THRUST(SHIP:ENGINES[0]).
        LOCAL model_t0 IS EM_GET_MODEL_THRUST_AT(0).
        PRINT "  Eng0 live/model: " + ROUND(live_t0,2) + " / " + ROUND(model_t0,2) +
              " kN (err " + ROUND(live_t0 - model_t0,2) + ")".
    }
    PRINT "".
    EMT_PRINT_ENGINE_CROSSCHECK().
}

ON ABORT {
    UNLOCK THROTTLE.
    CLEARSCREEN.
    PRINT "Engine model test aborted.".
    PRINT "Log file: " + EMT_LOG_FILE.
    PRINT "Rows written: " + EMT_LOG_ROWS.
}

IFC_INIT_STATE().
EM_INIT().

SET EMT_LOG_FILE TO EMT_BUILD_LOG_FILE().
EMT_WRITE_LOG_HEADER().
SET EMT_LOG_ACTIVE TO TRUE.
SET EMT_NEXT_LOG_UT TO TIME:SECONDS.

LOCAL UPDATE_HZ IS 10.
LOCAL update_dt IS 1.0 / UPDATE_HZ.
LOCAL last_disp_t IS 0.
LOCAL last_tick_t IS TIME:SECONDS.

LOCAL running IS TRUE.
UNTIL NOT running {
    SET THROTTLE_CMD TO EMT_GET_PILOT_THROTTLE().

    LOCAL now IS TIME:SECONDS.
    LOCAL dt IS now - last_tick_t.
    SET last_tick_t TO now.
    SET IFC_ACTUAL_DT TO EMT_CLAMP(dt, 0.01, 0.5).

    EM_TICK().

    IF EMT_LOG_ACTIVE AND now >= EMT_NEXT_LOG_UT {
        EMT_LOG_SAMPLE(IFC_ACTUAL_DT).
        SET EMT_NEXT_LOG_UT TO now + EMT_LOG_SAMPLE_DT.
    }

    IF now - last_disp_t >= update_dt {
        SET last_disp_t TO now.
        EMT_PRINT_SCREEN().
    }

    WAIT 0.02.
}
