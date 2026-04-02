@LAZYGLOBAL OFF.

// ============================================================
// engine_test.ks - IFC standalone engine/intake characterization
//
// Goal:
//   Run repeatable throttle step tests on a static stand and log
//   enough telemetry to estimate spool-up/spool-down dynamics and
//   build feed-forward models.
//
// Recommended setup:
//   - Tag test engine part: engine_test_engine
//   - Tag paired intake part: engine_test_intake
//
// Fallback behavior:
//   - If engine tag is missing, first SHIP:ENGINES entry is used.
//   - If intake tag is missing, first part with an intake module
//     is used (best effort).
//
// Output:
//   0:/Integrated Flight Computer/engine test logs/engine_test_log_XXXXXXXX.csv
// ============================================================

GLOBAL ET_ROOT IS "0:/Integrated Flight Computer/".
GLOBAL ET_LOG_DIR IS ET_ROOT + "engine test logs".
GLOBAL ET_COUNTER_PATH IS ET_LOG_DIR + "/counter.txt".

GLOBAL ET_ENGINE_TAG IS "engine_test_engine".
GLOBAL ET_INTAKE_TAG IS "engine_test_intake".

GLOBAL ET_STAGE_ON_ARM IS TRUE.
GLOBAL ET_PRETEST_IDLE_HOLD_S IS 2.0.
GLOBAL ET_SAMPLE_DT IS 0.10. // 10 Hz

// Per-phase hold durations (s).
// Increased from the first-run defaults so each step has more time to settle.
GLOBAL ET_HOLD_IDLE_BASELINE_S IS 12.0.
GLOBAL ET_HOLD_UP_FULL_1_S IS 24.0.
GLOBAL ET_HOLD_DOWN_IDLE_1_S IS 24.0.
GLOBAL ET_HOLD_UP_HALF_S IS 24.0.
GLOBAL ET_HOLD_DOWN_IDLE_2_S IS 20.0.
GLOBAL ET_HOLD_UP_FULL_2_S IS 24.0.
GLOBAL ET_HOLD_DOWN_IDLE_3_S IS 30.0.

GLOBAL ET_CMD_THROTTLE IS 0.

GLOBAL ET_ENG_FUEL_FLOW_FIELDS IS LIST(
  "Fuel Flow",
  "fuelFlow",
  "fuel flow",
  "requestedFuelFlow",
  "maxFuelFlow"
).

GLOBAL ET_ENG_THRUST_FIELDS IS LIST(
  "finalThrust",
  "FinalThrust",
  "final thrust",
  "requestedThrust",
  "maxThrust"
).

GLOBAL ET_ENG_THROTTLE_FIELDS IS LIST(
  "currentThrottle",
  "current throttle",
  "requestedThrottle",
  "throttle",
  "thrustPercentage"
).

GLOBAL ET_ENG_SPOOL_FIELDS IS LIST(
  "normalizedThrust",
  "normThrust",
  "spool",
  "EngineSpool",
  "spoolUp",
  "engineSpool"
).

GLOBAL ET_INT_AIRFLOW_FIELDS IS LIST(
  "airFlow",
  "airflow",
  "air flow",
  "finalAirFlow",
  "final air flow",
  "intakeFlow",
  "intake flow"
).

GLOBAL ET_INT_SPEED_FIELDS IS LIST(
  "intakeSpeed",
  "intakespeed",
  "speed",
  "mach"
).

GLOBAL ET_INT_AREA_FIELDS IS LIST(
  "area",
  "Area",
  "intakeArea",
  "intake area"
).

FUNCTION ET_CLAMP {
  PARAMETER x, lo, hi.
  IF x < lo { RETURN lo. }
  IF x > hi { RETURN hi. }
  RETURN x.
}

FUNCTION ET_SANITIZE_TXT {
  PARAMETER raw_txt.
  LOCAL s IS "" + raw_txt.
  SET s TO s:REPLACE(",", "_").
  SET s TO s:REPLACE(CHAR(10), " ").
  SET s TO s:REPLACE(CHAR(13), " ").
  RETURN s:TRIM.
}

FUNCTION ET_ENTRY_NAME {
  PARAMETER entry.
  IF entry = 0 { RETURN "". }
  IF entry:HASSUFFIX("NAME") { RETURN "" + entry:NAME. }
  RETURN "" + entry.
}

FUNCTION ET_PART_MODULE_ENTRIES {
  PARAMETER p.
  IF p = 0 { RETURN LIST(). }
  IF p:HASSUFFIX("MODULES") { RETURN p:MODULES. }
  IF p:HASSUFFIX("ALLMODULES") { RETURN p:ALLMODULES. }
  RETURN LIST().
}

FUNCTION ET_PART_HAS_MODULE_NAME {
  PARAMETER p, target_name.
  IF p = 0 { RETURN FALSE. }
  IF target_name = "" { RETURN FALSE. }

  LOCAL entries IS ET_PART_MODULE_ENTRIES(p).
  LOCAL i IS 0.
  UNTIL i >= entries:LENGTH {
    IF ET_ENTRY_NAME(entries[i]) = target_name { RETURN TRUE. }
    SET i TO i + 1.
  }
  RETURN FALSE.
}

FUNCTION ET_RESOLVE_MODULE {
  PARAMETER p, module_name.
  IF p = 0 { RETURN 0. }
  IF module_name = "" { RETURN 0. }
  IF NOT p:HASSUFFIX("GETMODULE") { RETURN 0. }
  IF NOT ET_PART_HAS_MODULE_NAME(p, module_name) { RETURN 0. }
  RETURN p:GETMODULE(module_name).
}

FUNCTION ET_FIND_MODULE_NAME_CONTAINS {
  PARAMETER p, needle.
  IF p = 0 { RETURN "". }
  LOCAL entries IS ET_PART_MODULE_ENTRIES(p).
  LOCAL i IS 0.
  UNTIL i >= entries:LENGTH {
    LOCAL nm IS ET_ENTRY_NAME(entries[i]).
    IF nm:TOLOWER():CONTAINS(needle:TOLOWER()) { RETURN nm. }
    SET i TO i + 1.
  }
  RETURN "".
}

FUNCTION ET_FIND_PART_BY_UID {
  PARAMETER uid_val.
  LOCAL all_parts IS SHIP:PARTS.
  LOCAL i IS 0.
  UNTIL i >= all_parts:LENGTH {
    LOCAL p IS all_parts[i].
    IF p <> 0 AND p:HASSUFFIX("UID") {
      IF p:UID = uid_val { RETURN p. }
    }
    SET i TO i + 1.
  }
  RETURN 0.
}

FUNCTION ET_FIND_PART_BY_TAG {
  PARAMETER tag_name.
  IF NOT SHIP:HASSUFFIX("PARTSTAGGED") { RETURN 0. }
  LOCAL tagged IS SHIP:PARTSTAGGED(tag_name).
  IF tagged:LENGTH <= 0 { RETURN 0. }
  RETURN tagged[0].
}

FUNCTION ET_PART_TITLE {
  PARAMETER p.
  IF p = 0 { RETURN "NONE". }
  IF p:HASSUFFIX("TITLE") { RETURN ET_SANITIZE_TXT(p:TITLE). }
  IF p:HASSUFFIX("NAME") { RETURN ET_SANITIZE_TXT(p:NAME). }
  IF p:HASSUFFIX("UID") { RETURN "UID_" + p:UID. }
  RETURN "UNKNOWN_PART".
}

FUNCTION ET_PART_NAME {
  PARAMETER p.
  IF p = 0 { RETURN "NONE". }
  IF p:HASSUFFIX("NAME") { RETURN ET_SANITIZE_TXT(p:NAME). }
  RETURN ET_PART_TITLE(p).
}

FUNCTION ET_PART_UID_TXT {
  PARAMETER p.
  IF p = 0 { RETURN "NONE". }
  IF p:HASSUFFIX("UID") { RETURN "" + p:UID. }
  RETURN "UNKNOWN_UID".
}

FUNCTION ET_CURRENT_THROTTLE {
  IF SHIP:HASSUFFIX("CONTROL") {
    LOCAL ctrl IS SHIP:CONTROL.
    IF ctrl:HASSUFFIX("MAINTHROTTLE") {
      RETURN ET_CLAMP(ctrl:MAINTHROTTLE, 0, 1).
    }
    IF ctrl:HASSUFFIX("PILOTMAINTHROTTLE") {
      RETURN ET_CLAMP(ctrl:PILOTMAINTHROTTLE, 0, 1).
    }
  }
  RETURN ET_CLAMP(ET_CMD_THROTTLE, 0, 1).
}

FUNCTION ET_BUILD_LOG_FILE {
  IF NOT EXISTS(ET_LOG_DIR) { CREATEDIR(ET_LOG_DIR). }

  LOCAL seq IS 1.
  IF EXISTS(ET_COUNTER_PATH) {
    LOCAL counter_str IS OPEN(ET_COUNTER_PATH):READALL:STRING:TRIM.
    IF counter_str:LENGTH > 0 {
      LOCAL parsed IS counter_str:TONUMBER(0).
      IF parsed >= 1 { SET seq TO ROUND(parsed). }
    }
  }
  IF EXISTS(ET_COUNTER_PATH) { DELETEPATH(ET_COUNTER_PATH). }
  LOG (seq + 1) TO ET_COUNTER_PATH.

  LOCAL seq_str IS "" + seq.
  UNTIL seq_str:LENGTH >= 8 { SET seq_str TO "0" + seq_str. }
  RETURN ET_LOG_DIR + "/engine_test_log_" + seq_str + ".csv".
}

FUNCTION ET_READ_FIELD_NUM {
  PARAMETER module_obj, field_name, fallback.
  IF module_obj = 0 { RETURN fallback. }
  IF field_name = "" { RETURN fallback. }
  IF NOT module_obj:HASSUFFIX("GETFIELD") { RETURN fallback. }
  IF module_obj:HASSUFFIX("HASFIELD") AND NOT module_obj:HASFIELD(field_name) { RETURN fallback. }
  LOCAL raw IS module_obj:GETFIELD(field_name).
  RETURN ("" + raw):TONUMBER(fallback).
}

FUNCTION ET_READ_FIRST_FIELD_NUM {
  PARAMETER module_obj, field_names, fallback.
  IF module_obj = 0 { RETURN fallback. }
  IF NOT module_obj:HASSUFFIX("HASFIELD") { RETURN fallback. }
  LOCAL i IS 0.
  UNTIL i >= field_names:LENGTH {
    LOCAL f IS field_names[i].
    IF module_obj:HASFIELD(f) {
      RETURN ET_READ_FIELD_NUM(module_obj, f, fallback).
    }
    SET i TO i + 1.
  }
  RETURN fallback.
}

FUNCTION ET_PART_RESOURCE_AMOUNT {
  PARAMETER p, resource_name.
  IF p = 0 OR NOT p:HASSUFFIX("RESOURCES") { RETURN -1. }
  LOCAL rs IS p:RESOURCES.
  LOCAL i IS 0.
  UNTIL i >= rs:LENGTH {
    LOCAL res_entry IS rs[i].
    IF res_entry <> 0 AND res_entry:HASSUFFIX("NAME") {
      IF res_entry:NAME = resource_name {
        IF res_entry:HASSUFFIX("AMOUNT") { RETURN res_entry:AMOUNT. }
      }
    }
    SET i TO i + 1.
  }
  RETURN -1.
}

FUNCTION ET_PART_RESOURCE_MAX {
  PARAMETER p, resource_name.
  IF p = 0 OR NOT p:HASSUFFIX("RESOURCES") { RETURN -1. }
  LOCAL rs IS p:RESOURCES.
  LOCAL i IS 0.
  UNTIL i >= rs:LENGTH {
    LOCAL res_entry IS rs[i].
    IF res_entry <> 0 AND res_entry:HASSUFFIX("NAME") {
      IF res_entry:NAME = resource_name {
        IF res_entry:HASSUFFIX("MAX") { RETURN res_entry:MAX. }
        IF res_entry:HASSUFFIX("MAXAMOUNT") { RETURN res_entry:MAXAMOUNT. }
        IF res_entry:HASSUFFIX("CAPACITY") { RETURN res_entry:CAPACITY. }
      }
    }
    SET i TO i + 1.
  }
  RETURN -1.
}

FUNCTION ET_PRESSURE_ATM {
  IF SHIP:HASSUFFIX("BODY") AND SHIP:BODY:HASSUFFIX("ATM") {
    RETURN SHIP:BODY:ATM:ALTITUDEPRESSURE(SHIP:ALTITUDE).
  }
  RETURN 0.
}

FUNCTION ET_LOG_MODULE_FIELD_LIST {
  PARAMETER log_file, module_obj, label_txt.
  IF module_obj = 0 {
    LOG "# " + label_txt + "_fields=NONE" TO log_file.
    RETURN.
  }
  IF NOT module_obj:HASSUFFIX("ALLFIELDS") {
    LOG "# " + label_txt + "_fields=UNKNOWN" TO log_file.
    RETURN.
  }
  LOCAL names IS module_obj:ALLFIELDS.
  LOCAL s IS "".
  LOCAL i IS 0.
  UNTIL i >= names:LENGTH {
    LOCAL nm IS ET_SANITIZE_TXT(names[i]).
    IF s = "" { SET s TO nm. }
    ELSE { SET s TO s + "|" + nm. }
    SET i TO i + 1.
  }
  LOG "# " + label_txt + "_fields=" + s TO log_file.
}

FUNCTION ET_FIND_ENGINE_ENTRY {
  LOCAL all_engs IS SHIP:ENGINES.
  IF all_engs:LENGTH <= 0 { RETURN 0. }

  LOCAL tagged_part IS ET_FIND_PART_BY_TAG(ET_ENGINE_TAG).
  IF tagged_part <> 0 AND tagged_part:HASSUFFIX("UID") {
    LOCAL i IS 0.
    UNTIL i >= all_engs:LENGTH {
      LOCAL eng IS all_engs[i].
      IF eng <> 0 AND eng:HASSUFFIX("UID") {
        IF eng:UID = tagged_part:UID {
          RETURN LEXICON(
            "engine", eng,
            "part", tagged_part,
            "source", "tag"
          ).
        }
      }
      SET i TO i + 1.
    }
  }

  LOCAL fallback_eng IS all_engs[0].
  LOCAL fallback_part IS 0.
  IF fallback_eng:HASSUFFIX("UID") {
    SET fallback_part TO ET_FIND_PART_BY_UID(fallback_eng:UID).
  }
  RETURN LEXICON(
    "engine", fallback_eng,
    "part", fallback_part,
    "source", "ship_engines_0"
  ).
}

FUNCTION ET_FIND_INTAKE_PART {
  LOCAL tagged_part IS ET_FIND_PART_BY_TAG(ET_INTAKE_TAG).
  IF tagged_part <> 0 { RETURN tagged_part. }

  LOCAL all_parts IS SHIP:PARTS.
  LOCAL i IS 0.
  UNTIL i >= all_parts:LENGTH {
    LOCAL p IS all_parts[i].
    IF p <> 0 {
      IF ET_PART_HAS_MODULE_NAME(p, "ModuleResourceIntake") { RETURN p. }
      LOCAL intake_nm IS ET_FIND_MODULE_NAME_CONTAINS(p, "intake").
      IF intake_nm <> "" { RETURN p. }
    }
    SET i TO i + 1.
  }
  RETURN 0.
}

FUNCTION ET_LOG_METADATA {
  PARAMETER log_file, engine_part, intake_part, engine_module_name, intake_module_name, source_txt.

  LOG "# engine_test_version=1" TO log_file.
  LOG "# craft_name=" + ET_SANITIZE_TXT(SHIP:NAME) TO log_file.
  LOG "# start_ut_s=" + ROUND(TIME:SECONDS, 3) TO log_file.
  LOG "# engine_source=" + ET_SANITIZE_TXT(source_txt) TO log_file.
  LOG "# engine_tag=" + ET_SANITIZE_TXT(ET_ENGINE_TAG) TO log_file.
  LOG "# intake_tag=" + ET_SANITIZE_TXT(ET_INTAKE_TAG) TO log_file.
  LOG "# engine_part_title=" + ET_PART_TITLE(engine_part) TO log_file.
  LOG "# engine_part_name=" + ET_PART_NAME(engine_part) TO log_file.
  LOG "# engine_part_uid=" + ET_PART_UID_TXT(engine_part) TO log_file.
  LOG "# intake_part_title=" + ET_PART_TITLE(intake_part) TO log_file.
  LOG "# intake_part_name=" + ET_PART_NAME(intake_part) TO log_file.
  LOG "# intake_part_uid=" + ET_PART_UID_TXT(intake_part) TO log_file.
  LOG "# engine_module_name=" + ET_SANITIZE_TXT(engine_module_name) TO log_file.
  LOG "# intake_module_name=" + ET_SANITIZE_TXT(intake_module_name) TO log_file.
}

FUNCTION ET_LOG_HEADER {
  PARAMETER log_file.
  LOG "t_s,phase_idx,phase_name,phase_t_s,dt_s,cmd_throttle,act_throttle,pressure_atm,engine_ignition,engine_flameout,engine_thrust_kn,engine_avail_thrust_kn,engine_max_thrust_kn,engine_massflow_tps,engine_massflow_kgps,engine_isp_s,engine_ispat_s,engine_thrustlimit,eng_mod_fuel_flow,eng_mod_thrust,eng_mod_throttle,eng_mod_spool,intake_air_amt,intake_air_max,int_mod_airflow,int_mod_speed,int_mod_area,ship_mass_t,ship_thrust_kn,ship_avail_thrust_kn,ship_vertspd_mps,ship_groundspeed_mps,ship_airspeed_mps,ship_mach,ship_alt_m,ship_agl_m,ship_dyn_pressure_pa,ship_status" TO log_file.
}

FUNCTION ET_PHASE_PROFILE {
  RETURN LIST(
    LEXICON("name", "idle_baseline", "cmd", 0.00, "hold_s", ET_HOLD_IDLE_BASELINE_S),
    LEXICON("name", "up_full_1",    "cmd", 1.00, "hold_s", ET_HOLD_UP_FULL_1_S),
    LEXICON("name", "down_idle_1",  "cmd", 0.00, "hold_s", ET_HOLD_DOWN_IDLE_1_S),
    LEXICON("name", "up_half",      "cmd", 0.50, "hold_s", ET_HOLD_UP_HALF_S),
    LEXICON("name", "down_idle_2",  "cmd", 0.00, "hold_s", ET_HOLD_DOWN_IDLE_2_S),
    LEXICON("name", "up_full_2",    "cmd", 1.00, "hold_s", ET_HOLD_UP_FULL_2_S),
    LEXICON("name", "down_idle_3",  "cmd", 0.00, "hold_s", ET_HOLD_DOWN_IDLE_3_S)
  ).
}

FUNCTION RUN_ENGINE_TEST {
  CLEARSCREEN.
  PRINT "========================================".
  PRINT "IFC ENGINE TEST - SPOOL CHARACTERIZATION".
  PRINT "========================================".
  PRINT "".

  LOCAL engine_entry IS ET_FIND_ENGINE_ENTRY().
  IF engine_entry = 0 {
    PRINT "ERROR: no engines found on vessel.".
    RETURN.
  }

  LOCAL engine_obj IS engine_entry["engine"].
  LOCAL engine_part IS engine_entry["part"].
  LOCAL source_txt IS engine_entry["source"].
  LOCAL intake_part IS ET_FIND_INTAKE_PART().

  LOCAL engine_module_name IS ET_FIND_MODULE_NAME_CONTAINS(engine_part, "moduleengines").
  IF engine_module_name = "" {
    SET engine_module_name TO ET_FIND_MODULE_NAME_CONTAINS(engine_part, "engines").
  }
  LOCAL intake_module_name IS "".
  IF intake_part <> 0 {
    IF ET_PART_HAS_MODULE_NAME(intake_part, "ModuleResourceIntake") {
      SET intake_module_name TO "ModuleResourceIntake".
    } ELSE {
      SET intake_module_name TO ET_FIND_MODULE_NAME_CONTAINS(intake_part, "intake").
    }
  }

  LOCAL engine_module_obj IS ET_RESOLVE_MODULE(engine_part, engine_module_name).
  LOCAL intake_module_obj IS ET_RESOLVE_MODULE(intake_part, intake_module_name).

  LOCAL log_file IS ET_BUILD_LOG_FILE().

  PRINT "Craft: " + ET_SANITIZE_TXT(SHIP:NAME).
  PRINT "Engine part: " + ET_PART_TITLE(engine_part).
  PRINT "Engine name: " + ET_PART_NAME(engine_part).
  PRINT "Engine source: " + source_txt.
  IF intake_part <> 0 {
    PRINT "Intake part: " + ET_PART_TITLE(intake_part).
    PRINT "Intake name: " + ET_PART_NAME(intake_part).
  } ELSE {
    PRINT "Intake part: NONE (logging intake module fields disabled)".
  }
  PRINT "Log file: " + log_file.
  PRINT "".
  PRINT "Type A to arm test, or Q to abort.".

  TERMINAL:INPUT:CLEAR().
  LOCAL arm_sel IS "".
  UNTIL arm_sel = "a" OR arm_sel = "A" OR arm_sel = "q" OR arm_sel = "Q" {
    SET arm_sel TO TERMINAL:INPUT:GETCHAR().
  }
  IF arm_sel = "q" OR arm_sel = "Q" {
    PRINT "Engine test canceled.".
    RETURN.
  }

  IF ET_STAGE_ON_ARM {
    PRINT "Staging...".
    STAGE.
    WAIT 0.5.
  }

  ET_LOG_METADATA(log_file, engine_part, intake_part, engine_module_name, intake_module_name, source_txt).
  ET_LOG_MODULE_FIELD_LIST(log_file, engine_module_obj, "engine_module").
  ET_LOG_MODULE_FIELD_LIST(log_file, intake_module_obj, "intake_module").
  ET_LOG_HEADER(log_file).

  BRAKES ON.
  SET ET_CMD_THROTTLE TO 0.
  LOCK THROTTLE TO ET_CMD_THROTTLE.
  WAIT ET_PRETEST_IDLE_HOLD_S.

  LOCAL test_running IS TRUE.
  ON ABORT {
    SET test_running TO FALSE.
    SET ET_CMD_THROTTLE TO 0.
    UNLOCK THROTTLE.
    PRINT "ABORT received: engine test stopped." AT(0, 0).
    PRESERVE.
  }

  LOCAL profile IS ET_PHASE_PROFILE().
  LOCAL run_start_ut IS TIME:SECONDS.
  LOCAL sample_last_ut IS TIME:SECONDS.

  PRINT "".
  PRINT "Running test profile...".
  PRINT "[ABORT] to stop and unlock throttle.".

  LOCAL p_idx IS 0.
  UNTIL p_idx >= profile:LENGTH OR NOT test_running {
    LOCAL step_info IS profile[p_idx].
    LOCAL phase_name IS step_info["name"].
    LOCAL cmd_target IS ET_CLAMP(step_info["cmd"], 0, 1).
    LOCAL hold_s IS step_info["hold_s"].
    LOCAL phase_start_ut IS TIME:SECONDS.

    SET ET_CMD_THROTTLE TO cmd_target.

    LOCAL phase_line IS "Phase " + (p_idx + 1) + "/" + profile:LENGTH + ": " + phase_name
      + "  cmd=" + ROUND(cmd_target, 3) + "  hold=" + ROUND(hold_s, 1) + "s".
    PRINT phase_line AT(0, 4).

    UNTIL (TIME:SECONDS - phase_start_ut) >= hold_s OR NOT test_running {
      LOCAL now IS TIME:SECONDS.
      LOCAL elapsed_since_sample IS now - sample_last_ut.
      LOCAL wait_s IS ET_SAMPLE_DT - elapsed_since_sample.
      IF wait_s > 0 { WAIT wait_s. }

      SET now TO TIME:SECONDS.
      LOCAL dt_s IS MAX(now - sample_last_ut, 0.001).
      SET sample_last_ut TO now.

      LOCAL run_t_s IS now - run_start_ut.
      LOCAL phase_t_s IS now - phase_start_ut.
      LOCAL pressure_atm IS ET_PRESSURE_ATM().

      LOCAL eng_ignition IS 0.
      IF engine_obj <> 0 AND engine_obj:HASSUFFIX("IGNITION") {
        SET eng_ignition TO CHOOSE 1 IF engine_obj:IGNITION ELSE 0.
      }

      LOCAL eng_flameout IS 0.
      IF engine_obj <> 0 AND engine_obj:HASSUFFIX("FLAMEOUT") {
        SET eng_flameout TO CHOOSE 1 IF engine_obj:FLAMEOUT ELSE 0.
      }

      LOCAL eng_thrust IS -1.
      IF engine_obj <> 0 AND engine_obj:HASSUFFIX("THRUST") {
        SET eng_thrust TO engine_obj:THRUST.
      }

      LOCAL eng_avail_thrust IS -1.
      IF engine_obj <> 0 AND engine_obj:HASSUFFIX("AVAILABLETHRUST") {
        SET eng_avail_thrust TO engine_obj:AVAILABLETHRUST.
      }

      LOCAL eng_max_thrust IS -1.
      IF engine_obj <> 0 AND engine_obj:HASSUFFIX("MAXTHRUST") {
        SET eng_max_thrust TO engine_obj:MAXTHRUST.
      }

      LOCAL eng_mdot_tps IS -1.
      IF engine_obj <> 0 AND engine_obj:HASSUFFIX("MASSFLOW") {
        SET eng_mdot_tps TO engine_obj:MASSFLOW.
      }

      LOCAL eng_mdot_kgps IS -1.
      IF eng_mdot_tps >= 0 { SET eng_mdot_kgps TO eng_mdot_tps * 1000.0. }

      LOCAL eng_isp_s IS -1.
      IF engine_obj <> 0 AND engine_obj:HASSUFFIX("ISP") {
        SET eng_isp_s TO engine_obj:ISP.
      }

      LOCAL eng_ispat_s IS -1.
      IF engine_obj <> 0 AND engine_obj:HASSUFFIX("ISPAT") {
        SET eng_ispat_s TO engine_obj:ISPAT(pressure_atm).
      }

      LOCAL eng_thrust_limit IS -1.
      IF engine_obj <> 0 AND engine_obj:HASSUFFIX("THRUSTLIMIT") {
        SET eng_thrust_limit TO engine_obj:THRUSTLIMIT.
      }

      LOCAL eng_mod_fuel_flow IS ET_READ_FIRST_FIELD_NUM(engine_module_obj, ET_ENG_FUEL_FLOW_FIELDS, -1).
      LOCAL eng_mod_thrust IS ET_READ_FIRST_FIELD_NUM(engine_module_obj, ET_ENG_THRUST_FIELDS, -1).
      LOCAL eng_mod_throttle IS ET_READ_FIRST_FIELD_NUM(engine_module_obj, ET_ENG_THROTTLE_FIELDS, -1).
      LOCAL eng_mod_spool IS ET_READ_FIRST_FIELD_NUM(engine_module_obj, ET_ENG_SPOOL_FIELDS, -1).

      LOCAL intake_air_amt IS ET_PART_RESOURCE_AMOUNT(intake_part, "IntakeAir").
      LOCAL intake_air_max IS ET_PART_RESOURCE_MAX(intake_part, "IntakeAir").
      LOCAL int_mod_airflow IS ET_READ_FIRST_FIELD_NUM(intake_module_obj, ET_INT_AIRFLOW_FIELDS, -1).
      LOCAL int_mod_speed IS ET_READ_FIRST_FIELD_NUM(intake_module_obj, ET_INT_SPEED_FIELDS, -1).
      LOCAL int_mod_area IS ET_READ_FIRST_FIELD_NUM(intake_module_obj, ET_INT_AREA_FIELDS, -1).

      LOCAL ship_q IS 0.
      IF SHIP:HASSUFFIX("Q") { SET ship_q TO SHIP:Q. }
      LOCAL ship_mach IS -1.
      IF SHIP:HASSUFFIX("MACH") { SET ship_mach TO SHIP:MACH. }

      LOCAL row IS LIST(
        ROUND(run_t_s, 3),
        p_idx + 1,
        ET_SANITIZE_TXT(phase_name),
        ROUND(phase_t_s, 3),
        ROUND(dt_s, 4),
        ROUND(ET_CMD_THROTTLE, 4),
        ROUND(ET_CURRENT_THROTTLE(), 4),
        ROUND(pressure_atm, 6),
        eng_ignition,
        eng_flameout,
        ROUND(eng_thrust, 4),
        ROUND(eng_avail_thrust, 4),
        ROUND(eng_max_thrust, 4),
        ROUND(eng_mdot_tps, 6),
        ROUND(eng_mdot_kgps, 3),
        ROUND(eng_isp_s, 3),
        ROUND(eng_ispat_s, 3),
        ROUND(eng_thrust_limit, 4),
        ROUND(eng_mod_fuel_flow, 6),
        ROUND(eng_mod_thrust, 6),
        ROUND(eng_mod_throttle, 6),
        ROUND(eng_mod_spool, 6),
        ROUND(intake_air_amt, 6),
        ROUND(intake_air_max, 6),
        ROUND(int_mod_airflow, 6),
        ROUND(int_mod_speed, 6),
        ROUND(int_mod_area, 6),
        ROUND(SHIP:MASS, 6),
        ROUND(SHIP:THRUST, 4),
        ROUND(SHIP:AVAILABLETHRUST, 4),
        ROUND(SHIP:VERTICALSPEED, 4),
        ROUND(SHIP:GROUNDSPEED, 4),
        ROUND(SHIP:AIRSPEED, 4),
        ROUND(ship_mach, 5),
        ROUND(SHIP:ALTITUDE, 3),
        ROUND(ALT:RADAR, 3),
        ROUND(ship_q, 4),
        ET_SANITIZE_TXT(SHIP:STATUS)
      ).

      LOG row:JOIN(",") TO log_file.
    }

    SET p_idx TO p_idx + 1.
  }

  SET ET_CMD_THROTTLE TO 0.
  WAIT 1.0.
  UNLOCK THROTTLE.

  LOCAL total_s IS TIME:SECONDS - run_start_ut.
  IF test_running {
    LOG "# result=COMPLETE" TO log_file.
  } ELSE {
    LOG "# result=ABORTED" TO log_file.
  }
  LOG "# end_ut_s=" + ROUND(TIME:SECONDS, 3) TO log_file.
  LOG "# duration_s=" + ROUND(total_s, 3) TO log_file.

  CLEARSCREEN.
  PRINT "ENGINE TEST FINISHED".
  LOCAL result_txt IS "ABORTED".
  IF test_running { SET result_txt TO "COMPLETE". }
  PRINT "Result: " + result_txt.
  PRINT "Duration: " + ROUND(total_s, 1) + " s".
  PRINT "Log: " + log_file.
}

RUN_ENGINE_TEST().
