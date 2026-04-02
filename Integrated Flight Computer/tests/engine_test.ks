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

// Manual geometry metadata overrides (set > 0 to log explicit diameters/scale).
// Keep at -1 to leave unspecified.
GLOBAL ET_ENGINE_DIAMETER_M_MANUAL IS -1.
GLOBAL ET_INTAKE_DIAMETER_M_MANUAL IS -1.
GLOBAL ET_SCALE_FACTOR_MANUAL IS -1.

// Phase progression gate:
// - stay in each phase for at least ET_PHASE_MIN_HOLD_S
// - then advance once all steady-state checks pass for ET_STEADY_HOLD_S
GLOBAL ET_PHASE_MIN_HOLD_S IS 10.0.
GLOBAL ET_STEADY_WINDOW_S IS 4.0.
GLOBAL ET_STEADY_HOLD_S IS 4.0.
// Safety timeout to prevent infinite waits if a channel never settles.
GLOBAL ET_PHASE_MAX_WAIT_S IS 120.0.
GLOBAL ET_PROP_SIGNAL_GRACE_S IS 6.0.

// Steady-state thresholds.
GLOBAL ET_STEADY_THRUST_SPREAD_KN IS 0.25.
GLOBAL ET_STEADY_FUEL_FLOW_SPREAD IS 0.0015.

GLOBAL ET_CMD_THROTTLE IS 0.

// Resolved engine module field names (auto-discovered at runtime).
GLOBAL ET_FIELD_ENG_FUEL_FLOW IS "".
GLOBAL ET_FIELD_ENG_THRUST IS "".
GLOBAL ET_FIELD_ENG_PROP_REQ_MET IS "".
GLOBAL ET_FIELD_ENG_PROP_REQ_MET_MODULE IS "".
GLOBAL ET_FIELD_ENG_THROTTLE IS "".

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

GLOBAL ET_TWEAKSCALE_FIELDS IS LIST(
  "currentScale",
  "CurrentScale",
  "current scale",
  "Current Scale",
  "tweakScale",
  "TweakScale",
  "scale",
  "Scale",
  "defaultScale",
  "DefaultScale",
  "size",
  "Size"
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

FUNCTION ET_CLEAN_FIELD_NAME {
  PARAMETER raw_name.
  LOCAL s IS ("" + raw_name):TRIM.
  IF s = "" { RETURN "". }

  SET s TO s:REPLACE("(settable)", "").
  SET s TO s:REPLACE("(readonly)", "").
  SET s TO s:REPLACE("(read-only)", "").
  SET s TO s:TRIM.

  IF s:CONTAINS("_ is ") {
    LOCAL p IS s:FIND("_ is ").
    IF p >= 0 {
      SET s TO s:SUBSTRING(0, p).
    }
  }

  IF s:LENGTH > 0 {
    LOCAL last_ch IS s:SUBSTRING(s:LENGTH - 1, 1).
    IF last_ch = "_" {
      SET s TO s:SUBSTRING(0, s:LENGTH - 1).
    }
  }

  RETURN s:TRIM.
}

FUNCTION ET_FIELD_CANDIDATES {
  PARAMETER raw_name.
  LOCAL out IS LIST().
  LOCAL original IS ("" + raw_name):TRIM.
  LOCAL cleaned IS ET_CLEAN_FIELD_NAME(original).

  IF original <> "" { out:ADD(original). }
  IF cleaned <> "" AND cleaned <> original { out:ADD(cleaned). }
  IF cleaned <> "" {
    LOCAL deperiod IS cleaned:REPLACE(".", ""):TRIM.
    IF deperiod <> "" AND deperiod <> cleaned { out:ADD(deperiod). }
  }
  RETURN out.
}

FUNCTION ET_READ_FIELD_NUM {
  PARAMETER module_obj, field_name, fallback.
  IF module_obj = 0 { RETURN fallback. }
  IF field_name = "" { RETURN fallback. }
  IF NOT module_obj:HASSUFFIX("GETFIELD") { RETURN fallback. }
  LOCAL candidates IS ET_FIELD_CANDIDATES(field_name).
  LOCAL i IS 0.
  UNTIL i >= candidates:LENGTH {
    LOCAL candidate IS candidates[i].
    IF candidate <> "" {
      LOCAL can_read IS FALSE.
      // Candidate[0] is the exact string sourced from ALLFIELDS.
      // Some modules reject HASFIELD() for that decorated label but
      // still allow GETFIELD() reads.
      IF i = 0 {
        SET can_read TO TRUE.
      } ELSE {
        SET can_read TO (NOT module_obj:HASSUFFIX("HASFIELD")) OR module_obj:HASFIELD(candidate).
      }
      IF can_read {
        LOCAL raw IS module_obj:GETFIELD(candidate).
        LOCAL parsed IS ET_PARSE_NUMERIC_TEXT(raw, fallback).
        IF parsed <> fallback { RETURN parsed. }
      }
    }
    SET i TO i + 1.
  }
  RETURN fallback.
}

FUNCTION ET_READ_FIELD_TEXT {
  PARAMETER module_obj, field_name, fallback.
  IF module_obj = 0 { RETURN fallback. }
  IF field_name = "" { RETURN fallback. }
  IF NOT module_obj:HASSUFFIX("GETFIELD") { RETURN fallback. }
  LOCAL candidates IS ET_FIELD_CANDIDATES(field_name).
  LOCAL i IS 0.
  UNTIL i >= candidates:LENGTH {
    LOCAL candidate IS candidates[i].
    IF candidate <> "" {
      LOCAL can_read IS FALSE.
      IF i = 0 {
        SET can_read TO TRUE.
      } ELSE {
        SET can_read TO (NOT module_obj:HASSUFFIX("HASFIELD")) OR module_obj:HASFIELD(candidate).
      }
      IF can_read {
        RETURN "" + module_obj:GETFIELD(candidate).
      }
    }
    SET i TO i + 1.
  }
  RETURN fallback.
}

FUNCTION ET_TEXT_IS_TRUE {
  PARAMETER raw_text.
  LOCAL s IS ("" + raw_text):TOLOWER:TRIM.
  RETURN s = "true" OR s = "1" OR s = "yes" OR s = "on".
}

FUNCTION ET_TEXT_IS_FALSE {
  PARAMETER raw_text.
  LOCAL s IS ("" + raw_text):TOLOWER:TRIM.
  RETURN s = "false" OR s = "0" OR s = "no" OR s = "off".
}

FUNCTION ET_READ_FIELD_BOOL01 {
  PARAMETER module_obj, field_name, fallback.
  IF module_obj = 0 { RETURN fallback. }
  IF field_name = "" { RETURN fallback. }
  LOCAL txt IS ET_READ_FIELD_TEXT(module_obj, field_name, "").
  IF txt = "" { RETURN fallback. }
  IF ET_TEXT_IS_TRUE(txt) { RETURN 1. }
  IF ET_TEXT_IS_FALSE(txt) { RETURN 0. }

  // Many KSP module fields expose percentages like "100.00%".
  // Normalize those to 0..1 so they can be used as steady-state gates.
  LOCAL cleaned IS txt:REPLACE("%", ""):TRIM.
  LOCAL parsed IS cleaned:TONUMBER(fallback).
  IF parsed = fallback { RETURN fallback. }

  IF txt:CONTAINS("%") OR parsed > 1 {
    SET parsed TO parsed / 100.0.
  }
  RETURN ET_CLAMP(parsed, 0, 1).
}

FUNCTION ET_FIELD_NAME_CONTAINS {
  PARAMETER field_name, needle.
  IF field_name = "" { RETURN FALSE. }
  IF needle = "" { RETURN FALSE. }
  RETURN field_name:TOLOWER:CONTAINS(needle:TOLOWER).
}

FUNCTION ET_NORMALIZE_FIELD_KEY {
  PARAMETER raw_text.
  LOCAL s IS ("" + raw_text):TOLOWER.
  SET s TO s:REPLACE(" ", "").
  SET s TO s:REPLACE(".", "").
  SET s TO s:REPLACE("_", "").
  SET s TO s:REPLACE("-", "").
  SET s TO s:REPLACE("(", "").
  SET s TO s:REPLACE(")", "").
  SET s TO s:REPLACE("/", "").
  RETURN s.
}

FUNCTION ET_FIND_ENGINE_FIELD_BINDINGS {
  PARAMETER module_obj.
  SET ET_FIELD_ENG_FUEL_FLOW TO "".
  SET ET_FIELD_ENG_THRUST TO "".
  SET ET_FIELD_ENG_PROP_REQ_MET TO "".
  SET ET_FIELD_ENG_THROTTLE TO "".

  IF module_obj = 0 OR NOT module_obj:HASSUFFIX("ALLFIELDS") { RETURN. }

  LOCAL names IS module_obj:ALLFIELDS.
  LOCAL i IS 0.
  UNTIL i >= names:LENGTH {
    LOCAL field_name IS "" + names[i].
    LOCAL field_lc IS field_name:TOLOWER.
    LOCAL field_norm IS ET_NORMALIZE_FIELD_KEY(field_name).

    IF ET_FIELD_ENG_FUEL_FLOW = "" {
      IF field_lc:CONTAINS("fuel flow") OR field_norm:CONTAINS("fuelflow") {
        SET ET_FIELD_ENG_FUEL_FLOW TO field_name.
      }
    }

    IF ET_FIELD_ENG_THRUST = "" {
      IF field_norm:CONTAINS("thrust") AND NOT field_norm:CONTAINS("limiter") {
        SET ET_FIELD_ENG_THRUST TO field_name.
      }
    }

    IF ET_FIELD_ENG_PROP_REQ_MET = "" {
      LOCAL has_req IS field_norm:CONTAINS("require") OR field_norm:CONTAINS("req").
      LOCAL has_met IS field_norm:CONTAINS("met") OR field_norm:CONTAINS("satisfied").
      LOCAL has_prop IS field_norm:CONTAINS("prop") OR field_norm:CONTAINS("propellant").
      IF has_req AND has_met AND has_prop {
        SET ET_FIELD_ENG_PROP_REQ_MET TO field_name.
      }
    }

    IF ET_FIELD_ENG_THROTTLE = "" {
      IF field_lc:CONTAINS("throttle") AND NOT field_lc:CONTAINS("limiter") {
        SET ET_FIELD_ENG_THROTTLE TO field_name.
      }
    }

    SET i TO i + 1.
  }

  // Fallback: if no explicit prop*requirement*met field was found,
  // but a requirement/met field exists, use it.
  IF ET_FIELD_ENG_PROP_REQ_MET = "" {
    SET i TO 0.
    UNTIL i >= names:LENGTH {
      LOCAL fallback_name IS "" + names[i].
      LOCAL fallback_norm IS ET_NORMALIZE_FIELD_KEY(fallback_name).
      LOCAL fallback_hit IS FALSE.
      IF fallback_norm:CONTAINS("propreqmet") { SET fallback_hit TO TRUE. }
      IF fallback_norm:CONTAINS("proprequirementmet") { SET fallback_hit TO TRUE. }
      IF fallback_norm:CONTAINS("propellantreqmet") { SET fallback_hit TO TRUE. }
      IF NOT fallback_hit {
        IF fallback_norm:CONTAINS("prop") AND fallback_norm:CONTAINS("req") AND fallback_norm:CONTAINS("met") {
          SET fallback_hit TO TRUE.
        }
      }
      IF fallback_hit {
        SET ET_FIELD_ENG_PROP_REQ_MET TO fallback_name.
        BREAK.
      }
      SET i TO i + 1.
    }
  }
}

FUNCTION ET_FIELD_LOOKS_PROP_REQ_MET {
  PARAMETER field_name.
  IF field_name = "" { RETURN FALSE. }

  LOCAL n IS ET_NORMALIZE_FIELD_KEY(field_name).
  LOCAL has_prop IS n:CONTAINS("prop") OR n:CONTAINS("propellant").
  LOCAL has_req IS n:CONTAINS("require") OR n:CONTAINS("req").
  LOCAL has_met IS n:CONTAINS("met") OR n:CONTAINS("satisfied").

  IF has_prop AND has_req AND has_met { RETURN TRUE. }
  IF n:CONTAINS("propreqmet") { RETURN TRUE. }
  IF n:CONTAINS("proprequirementmet") { RETURN TRUE. }
  IF n:CONTAINS("propellantreqmet") { RETURN TRUE. }
  RETURN FALSE.
}

FUNCTION ET_FIND_PROP_REQ_BINDING_ON_PART {
  PARAMETER p.
  IF p = 0 { RETURN LEXICON("module_name", "", "field_name", ""). }

  LOCAL modules IS ET_PART_MODULE_ENTRIES(p).
  LOCAL best_module IS "".
  LOCAL best_field IS "".
  LOCAL best_score IS -1.

  LOCAL i IS 0.
  UNTIL i >= modules:LENGTH {
    LOCAL module_name IS ET_ENTRY_NAME(modules[i]).
    IF module_name <> "" {
      LOCAL module_obj IS ET_RESOLVE_MODULE(p, module_name).
      IF module_obj <> 0 AND module_obj:HASSUFFIX("ALLFIELDS") {
        LOCAL fields IS module_obj:ALLFIELDS.
        LOCAL j IS 0.
        UNTIL j >= fields:LENGTH {
          LOCAL field_name IS "" + fields[j].
          IF ET_FIELD_LOOKS_PROP_REQ_MET(field_name) {
            RETURN LEXICON("module_name", module_name, "field_name", field_name).
          }

          // Weaker score-based fallback if literal match isn't found.
          LOCAL n IS ET_NORMALIZE_FIELD_KEY(field_name).
          LOCAL score IS 0.
          IF n:CONTAINS("prop") OR n:CONTAINS("propellant") { SET score TO score + 2. }
          IF n:CONTAINS("require") OR n:CONTAINS("req") { SET score TO score + 2. }
          IF n:CONTAINS("met") OR n:CONTAINS("satisfied") { SET score TO score + 2. }
          IF n:CONTAINS("ratio") OR n:CONTAINS("percent") { SET score TO score + 1. }

          IF score > best_score {
            SET best_score TO score.
            SET best_module TO module_name.
            SET best_field TO field_name.
          }
          SET j TO j + 1.
        }
      }
    }
    SET i TO i + 1.
  }

  IF best_score >= 4 {
    RETURN LEXICON("module_name", best_module, "field_name", best_field).
  }
  RETURN LEXICON("module_name", "", "field_name", "").
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

FUNCTION ET_FIND_FIELD_NAME_CONTAINS {
  PARAMETER module_obj, needle.
  IF module_obj = 0 { RETURN "". }
  IF needle = "" { RETURN "". }
  IF NOT module_obj:HASSUFFIX("ALLFIELDS") { RETURN "". }

  LOCAL fields IS module_obj:ALLFIELDS.
  LOCAL i IS 0.
  UNTIL i >= fields:LENGTH {
    LOCAL f_name IS "" + fields[i].
    IF f_name:TOLOWER:CONTAINS(needle:TOLOWER) { RETURN f_name. }
    SET i TO i + 1.
  }
  RETURN "".
}

FUNCTION ET_PARSE_NUMERIC_TEXT {
  PARAMETER raw_text, fallback.
  LOCAL s IS ("" + raw_text):TRIM.
  IF s = "" { RETURN fallback. }

  LOCAL direct_val IS s:TONUMBER(fallback).
  IF direct_val <> fallback { RETURN direct_val. }

  // Common formatting seen in PAW/module fields (e.g. "1.875x", "2.5m", "100.00%").
  SET s TO s:TOLOWER.
  SET s TO s:REPLACE("%", "").
  SET s TO s:REPLACE("x", "").
  SET s TO s:REPLACE("m", "").
  SET s TO s:REPLACE("scale", "").
  SET s TO s:REPLACE("current", "").
  SET s TO s:REPLACE(":", "").
  SET s TO s:TRIM.

  RETURN s:TONUMBER(fallback).
}

FUNCTION ET_RESOLVE_TWEAKSCALE_INFO {
  PARAMETER p.
  IF p = 0 {
    RETURN LEXICON("module_name", "", "field_name", "", "value", -1).
  }

  LOCAL module_name IS ET_FIND_MODULE_NAME_CONTAINS(p, "tweakscale").
  IF module_name = "" {
    RETURN LEXICON("module_name", "", "field_name", "", "value", -1).
  }

  LOCAL module_obj IS ET_RESOLVE_MODULE(p, module_name).
  IF module_obj = 0 {
    RETURN LEXICON("module_name", module_name, "field_name", "", "value", -1).
  }

  LOCAL field_name IS "".
  LOCAL i IS 0.
  IF module_obj:HASSUFFIX("HASFIELD") {
    UNTIL i >= ET_TWEAKSCALE_FIELDS:LENGTH {
      LOCAL candidate IS ET_TWEAKSCALE_FIELDS[i].
      IF module_obj:HASFIELD(candidate) {
        SET field_name TO candidate.
        BREAK.
      }
      SET i TO i + 1.
    }
  }

  IF field_name = "" { SET field_name TO ET_FIND_FIELD_NAME_CONTAINS(module_obj, "current scale"). }
  IF field_name = "" { SET field_name TO ET_FIND_FIELD_NAME_CONTAINS(module_obj, "currentscale"). }
  IF field_name = "" { SET field_name TO ET_FIND_FIELD_NAME_CONTAINS(module_obj, "tweakscale"). }
  IF field_name = "" { SET field_name TO ET_FIND_FIELD_NAME_CONTAINS(module_obj, "scale"). }

  LOCAL scale_val IS -1.
  IF field_name <> "" {
    LOCAL raw_scale_txt IS ET_READ_FIELD_TEXT(module_obj, field_name, "").
    SET scale_val TO ET_PARSE_NUMERIC_TEXT(raw_scale_txt, -1).
    IF scale_val < 0 {
      SET scale_val TO ET_READ_FIELD_NUM(module_obj, field_name, -1).
    }
  }

  RETURN LEXICON(
    "module_name", module_name,
    "field_name", field_name,
    "value", scale_val
  ).
}

FUNCTION ET_CLIP_LIST_LEN {
  PARAMETER target_list, max_len.
  UNTIL target_list:LENGTH <= max_len {
    target_list:REMOVE(0).
  }
}

FUNCTION ET_LIST_SPREAD {
  PARAMETER target_list.
  IF target_list:LENGTH <= 0 { RETURN -1. }
  LOCAL min_v IS target_list[0].
  LOCAL max_v IS target_list[0].
  LOCAL i IS 1.
  UNTIL i >= target_list:LENGTH {
    LOCAL sample_val IS target_list[i].
    IF sample_val < min_v { SET min_v TO sample_val. }
    IF sample_val > max_v { SET max_v TO sample_val. }
    SET i TO i + 1.
  }
  RETURN max_v - min_v.
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
  PARAMETER log_file, engine_part, intake_part, engine_module_name, intake_module_name, source_txt, engine_scale_info, intake_scale_info.

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
  LOG "# engine_diameter_m_manual=" + ROUND(ET_ENGINE_DIAMETER_M_MANUAL, 6) TO log_file.
  LOG "# intake_diameter_m_manual=" + ROUND(ET_INTAKE_DIAMETER_M_MANUAL, 6) TO log_file.
  LOG "# scale_factor_manual=" + ROUND(ET_SCALE_FACTOR_MANUAL, 6) TO log_file.
  LOG "# engine_tweakscale_module_name=" + ET_SANITIZE_TXT(engine_scale_info["module_name"]) TO log_file.
  LOG "# engine_tweakscale_field_name=" + ET_SANITIZE_TXT(engine_scale_info["field_name"]) TO log_file.
  LOG "# engine_tweakscale_value=" + ROUND(engine_scale_info["value"], 6) TO log_file.
  LOG "# intake_tweakscale_module_name=" + ET_SANITIZE_TXT(intake_scale_info["module_name"]) TO log_file.
  LOG "# intake_tweakscale_field_name=" + ET_SANITIZE_TXT(intake_scale_info["field_name"]) TO log_file.
  LOG "# intake_tweakscale_value=" + ROUND(intake_scale_info["value"], 6) TO log_file.
  LOG "# engine_field_fuel_flow=" + ET_SANITIZE_TXT(ET_FIELD_ENG_FUEL_FLOW) TO log_file.
  LOG "# engine_field_thrust=" + ET_SANITIZE_TXT(ET_FIELD_ENG_THRUST) TO log_file.
  LOG "# engine_field_prop_req_met=" + ET_SANITIZE_TXT(ET_FIELD_ENG_PROP_REQ_MET) TO log_file.
  LOG "# engine_field_prop_req_module=" + ET_SANITIZE_TXT(ET_FIELD_ENG_PROP_REQ_MET_MODULE) TO log_file.
  LOG "# engine_field_throttle=" + ET_SANITIZE_TXT(ET_FIELD_ENG_THROTTLE) TO log_file.
  LOG "# phase_min_hold_s=" + ROUND(ET_PHASE_MIN_HOLD_S, 3) TO log_file.
  LOG "# steady_window_s=" + ROUND(ET_STEADY_WINDOW_S, 3) TO log_file.
  LOG "# steady_hold_s=" + ROUND(ET_STEADY_HOLD_S, 3) TO log_file.
  LOG "# phase_max_wait_s=" + ROUND(ET_PHASE_MAX_WAIT_S, 3) TO log_file.
  LOG "# steady_thrust_spread_kn=" + ROUND(ET_STEADY_THRUST_SPREAD_KN, 6) TO log_file.
  LOG "# steady_fuel_flow_spread=" + ROUND(ET_STEADY_FUEL_FLOW_SPREAD, 6) TO log_file.
}

FUNCTION ET_LOG_HEADER {
  PARAMETER log_file.
  LOG "t_s,phase_idx,phase_name,phase_t_s,dt_s,cmd_throttle,act_throttle,pressure_atm,engine_ignition,engine_flameout,engine_thrust_kn,engine_avail_thrust_kn,engine_max_thrust_kn,engine_massflow_tps,engine_massflow_kgps,engine_isp_s,engine_ispat_s,engine_thrustlimit,eng_mod_fuel_flow,eng_mod_thrust,eng_mod_prop_req_met,eng_mod_throttle,eng_mod_spool,phase_steady_thrust_ok,phase_steady_fuel_ok,phase_steady_prop_ok,phase_steady_all_ok,intake_air_amt,intake_air_max,int_mod_airflow,int_mod_speed,int_mod_area,ship_mass_t,ship_thrust_kn,ship_avail_thrust_kn,ship_vertspd_mps,ship_groundspeed_mps,ship_airspeed_mps,ship_mach,ship_alt_m,ship_agl_m,ship_dyn_pressure_pa,ship_status" TO log_file.
}

FUNCTION ET_PHASE_PROFILE {
  RETURN LIST(
    LEXICON("name", "idle_baseline", "cmd", 0.00),
    LEXICON("name", "up_full_1",    "cmd", 1.00),
    LEXICON("name", "down_idle_1",  "cmd", 0.00),
    LEXICON("name", "up_half",      "cmd", 0.50),
    LEXICON("name", "down_idle_2",  "cmd", 0.00),
    LEXICON("name", "up_full_2",    "cmd", 1.00),
    LEXICON("name", "down_idle_3",  "cmd", 0.00)
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
  ET_FIND_ENGINE_FIELD_BINDINGS(engine_module_obj).
  SET ET_FIELD_ENG_PROP_REQ_MET_MODULE TO engine_module_name.

  LOCAL prop_req_binding IS ET_FIND_PROP_REQ_BINDING_ON_PART(engine_part).
  LOCAL prop_req_module_name IS ET_FIELD_ENG_PROP_REQ_MET_MODULE.
  LOCAL prop_req_module_obj IS engine_module_obj.
  IF prop_req_binding["field_name"] <> "" {
    SET ET_FIELD_ENG_PROP_REQ_MET TO prop_req_binding["field_name"].
    SET prop_req_module_name TO prop_req_binding["module_name"].
    SET ET_FIELD_ENG_PROP_REQ_MET_MODULE TO prop_req_module_name.
    SET prop_req_module_obj TO ET_RESOLVE_MODULE(engine_part, prop_req_module_name).
  }

  LOCAL engine_scale_info IS ET_RESOLVE_TWEAKSCALE_INFO(engine_part).
  LOCAL intake_scale_info IS ET_RESOLVE_TWEAKSCALE_INFO(intake_part).

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
  PRINT "Engine field fuel flow: " + ET_SANITIZE_TXT(ET_FIELD_ENG_FUEL_FLOW).
  PRINT "Engine field thrust: " + ET_SANITIZE_TXT(ET_FIELD_ENG_THRUST).
  PRINT "Engine field prop req: " + ET_SANITIZE_TXT(ET_FIELD_ENG_PROP_REQ_MET).
  PRINT "Engine prop req module: " + ET_SANITIZE_TXT(ET_FIELD_ENG_PROP_REQ_MET_MODULE).
  PRINT "Manual engine diameter (m): " + ROUND(ET_ENGINE_DIAMETER_M_MANUAL, 6).
  PRINT "Manual intake diameter (m): " + ROUND(ET_INTAKE_DIAMETER_M_MANUAL, 6).
  PRINT "Manual scale factor: " + ROUND(ET_SCALE_FACTOR_MANUAL, 6).
  PRINT "Engine TweakScale: " + ROUND(engine_scale_info["value"], 6).
  PRINT "Intake TweakScale: " + ROUND(intake_scale_info["value"], 6).
  IF ET_FIELD_ENG_FUEL_FLOW = "" {
    PRINT "NOTE: fuel-flow field not found pre-arm (will retry after stage).".
  }
  IF ET_FIELD_ENG_THRUST = "" {
    PRINT "NOTE: thrust field not found pre-arm (will retry after stage).".
  }
  IF ET_FIELD_ENG_PROP_REQ_MET = "" {
    PRINT "NOTE: prop requirement field not found pre-arm (will retry after stage).".
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

  // Some engine module fields are only present after the engine is active.
  ET_FIND_ENGINE_FIELD_BINDINGS(engine_module_obj).
  SET ET_FIELD_ENG_PROP_REQ_MET_MODULE TO engine_module_name.
  SET prop_req_module_name TO ET_FIELD_ENG_PROP_REQ_MET_MODULE.
  SET prop_req_module_obj TO engine_module_obj.
  SET prop_req_binding TO ET_FIND_PROP_REQ_BINDING_ON_PART(engine_part).
  IF prop_req_binding["field_name"] <> "" {
    SET ET_FIELD_ENG_PROP_REQ_MET TO prop_req_binding["field_name"].
    SET prop_req_module_name TO prop_req_binding["module_name"].
    SET ET_FIELD_ENG_PROP_REQ_MET_MODULE TO prop_req_module_name.
    SET prop_req_module_obj TO ET_RESOLVE_MODULE(engine_part, prop_req_module_name).
  }

  PRINT "Post-stage prop req field: " + ET_SANITIZE_TXT(ET_FIELD_ENG_PROP_REQ_MET).
  PRINT "Post-stage prop req module: " + ET_SANITIZE_TXT(ET_FIELD_ENG_PROP_REQ_MET_MODULE).
  IF ET_FIELD_ENG_PROP_REQ_MET = "" {
    PRINT "WARNING: prop requirement field not found (prop steady gate disabled).".
  }

  ET_LOG_METADATA(log_file, engine_part, intake_part, engine_module_name, intake_module_name, source_txt, engine_scale_info, intake_scale_info).
  LOCAL engine_tweakscale_obj IS ET_RESOLVE_MODULE(engine_part, engine_scale_info["module_name"]).
  LOCAL intake_tweakscale_obj IS ET_RESOLVE_MODULE(intake_part, intake_scale_info["module_name"]).
  ET_LOG_MODULE_FIELD_LIST(log_file, engine_module_obj, "engine_module").
  ET_LOG_MODULE_FIELD_LIST(log_file, prop_req_module_obj, "prop_req_module").
  ET_LOG_MODULE_FIELD_LIST(log_file, intake_module_obj, "intake_module").
  ET_LOG_MODULE_FIELD_LIST(log_file, engine_tweakscale_obj, "engine_tweakscale_module").
  ET_LOG_MODULE_FIELD_LIST(log_file, intake_tweakscale_obj, "intake_tweakscale_module").
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
    LOCAL phase_start_ut IS TIME:SECONDS.
    LOCAL phase_window_n IS MAX(3, ROUND(ET_STEADY_WINDOW_S / ET_SAMPLE_DT, 0)).
    LOCAL thrust_hist IS LIST().
    LOCAL fuel_hist IS LIST().
    LOCAL prop_hist IS LIST().
    LOCAL require_prop_signal IS ET_FIELD_ENG_PROP_REQ_MET <> "".
    LOCAL prop_signal_seen IS FALSE.
    LOCAL prop_signal_disabled_logged IS FALSE.
    LOCAL steady_start_ut IS -1.
    LOCAL phase_done IS FALSE.
    LOCAL phase_done_reason IS "".

    SET ET_CMD_THROTTLE TO cmd_target.

    LOCAL phase_line IS "Phase " + (p_idx + 1) + "/" + profile:LENGTH + ": " + phase_name
      + "  cmd=" + ROUND(cmd_target, 3)
      + "  min=" + ROUND(ET_PHASE_MIN_HOLD_S, 1) + "s + steady".
    PRINT phase_line AT(0, 4).

    UNTIL phase_done OR NOT test_running {
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

      // Last-chance dynamic binding: some engines expose prop-requirement
      // fields only after throttle and airflow are established.
      IF ET_FIELD_ENG_PROP_REQ_MET = "" {
        SET prop_req_binding TO ET_FIND_PROP_REQ_BINDING_ON_PART(engine_part).
        IF prop_req_binding["field_name"] <> "" {
          SET ET_FIELD_ENG_PROP_REQ_MET TO prop_req_binding["field_name"].
          SET prop_req_module_name TO prop_req_binding["module_name"].
          SET ET_FIELD_ENG_PROP_REQ_MET_MODULE TO prop_req_module_name.
          SET prop_req_module_obj TO ET_RESOLVE_MODULE(engine_part, prop_req_module_name).
        }
      }

      LOCAL eng_mod_fuel_flow IS ET_READ_FIELD_NUM(engine_module_obj, ET_FIELD_ENG_FUEL_FLOW, -1).
      LOCAL eng_mod_thrust IS ET_READ_FIELD_NUM(engine_module_obj, ET_FIELD_ENG_THRUST, -1).
      LOCAL eng_mod_prop_req_met IS ET_READ_FIELD_BOOL01(prop_req_module_obj, ET_FIELD_ENG_PROP_REQ_MET, -1).
      LOCAL eng_mod_throttle IS ET_READ_FIELD_BOOL01(engine_module_obj, ET_FIELD_ENG_THROTTLE, -1).
      LOCAL eng_mod_spool IS ET_READ_FIRST_FIELD_NUM(engine_module_obj, ET_ENG_SPOOL_FIELDS, -1).
      IF eng_mod_prop_req_met >= 0 { SET prop_signal_seen TO TRUE. }

      LOCAL intake_air_amt IS ET_PART_RESOURCE_AMOUNT(intake_part, "IntakeAir").
      LOCAL intake_air_max IS ET_PART_RESOURCE_MAX(intake_part, "IntakeAir").
      LOCAL int_mod_airflow IS ET_READ_FIRST_FIELD_NUM(intake_module_obj, ET_INT_AIRFLOW_FIELDS, -1).
      LOCAL int_mod_speed IS ET_READ_FIRST_FIELD_NUM(intake_module_obj, ET_INT_SPEED_FIELDS, -1).
      LOCAL int_mod_area IS ET_READ_FIRST_FIELD_NUM(intake_module_obj, ET_INT_AREA_FIELDS, -1).

      LOCAL steady_thrust_sig IS eng_thrust.
      IF eng_mod_thrust >= 0 { SET steady_thrust_sig TO eng_mod_thrust. }
      LOCAL steady_fuel_sig IS eng_mod_fuel_flow.
      IF steady_fuel_sig < 0 AND eng_mdot_tps >= 0 { SET steady_fuel_sig TO eng_mdot_tps. }
      LOCAL steady_prop_sig IS eng_mod_prop_req_met.

      IF steady_thrust_sig >= 0 {
        thrust_hist:ADD(steady_thrust_sig).
        ET_CLIP_LIST_LEN(thrust_hist, phase_window_n).
      }
      IF steady_fuel_sig >= 0 {
        fuel_hist:ADD(steady_fuel_sig).
        ET_CLIP_LIST_LEN(fuel_hist, phase_window_n).
      }
      IF steady_prop_sig >= 0 {
        prop_hist:ADD(steady_prop_sig).
        ET_CLIP_LIST_LEN(prop_hist, phase_window_n).
      }

      LOCAL thrust_spread IS ET_LIST_SPREAD(thrust_hist).
      LOCAL fuel_spread IS ET_LIST_SPREAD(fuel_hist).
      LOCAL prop_spread IS ET_LIST_SPREAD(prop_hist).
      LOCAL steady_thrust_ok IS FALSE.
      LOCAL steady_fuel_ok IS FALSE.
      LOCAL steady_prop_ok IS TRUE.

      IF thrust_hist:LENGTH >= phase_window_n AND thrust_spread >= 0 AND thrust_spread <= ET_STEADY_THRUST_SPREAD_KN {
        SET steady_thrust_ok TO TRUE.
      }
      IF fuel_hist:LENGTH >= phase_window_n AND fuel_spread >= 0 AND fuel_spread <= ET_STEADY_FUEL_FLOW_SPREAD {
        SET steady_fuel_ok TO TRUE.
      }
      IF require_prop_signal {
        IF NOT prop_signal_seen AND phase_t_s >= ET_PROP_SIGNAL_GRACE_S {
          SET require_prop_signal TO FALSE.
          IF NOT prop_signal_disabled_logged {
            LOG "# note phase=" + phase_name + " prop_signal_unreadable_disable_gate_at_s=" + ROUND(phase_t_s, 2) TO log_file.
            SET prop_signal_disabled_logged TO TRUE.
          }
        }
      }
      IF require_prop_signal {
        SET steady_prop_ok TO FALSE.
        IF prop_hist:LENGTH >= phase_window_n AND prop_spread >= 0 AND prop_spread <= 0.01 {
          SET steady_prop_ok TO TRUE.
        }
      }

      LOCAL steady_all_ok IS steady_thrust_ok AND steady_fuel_ok AND steady_prop_ok.
      IF steady_all_ok {
        IF steady_start_ut < 0 { SET steady_start_ut TO now. }
      } ELSE {
        SET steady_start_ut TO -1.
      }

      IF phase_t_s >= ET_PHASE_MIN_HOLD_S AND steady_start_ut >= 0 AND (now - steady_start_ut) >= ET_STEADY_HOLD_S {
        SET phase_done TO TRUE.
        SET phase_done_reason TO "steady".
      }
      IF NOT phase_done AND ET_PHASE_MAX_WAIT_S > 0 AND phase_t_s >= ET_PHASE_MAX_WAIT_S {
        SET phase_done TO TRUE.
        SET phase_done_reason TO "timeout".
      }

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
        ROUND(eng_mod_prop_req_met, 0),
        ROUND(eng_mod_throttle, 6),
        ROUND(eng_mod_spool, 6),
        CHOOSE 1 IF steady_thrust_ok ELSE 0,
        CHOOSE 1 IF steady_fuel_ok ELSE 0,
        CHOOSE 1 IF steady_prop_ok ELSE 0,
        CHOOSE 1 IF steady_all_ok ELSE 0,
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

    IF phase_done_reason = "" {
      SET phase_done_reason TO "aborted".
    }
    LOCAL phase_done_msg IS "# phase_done idx=" + (p_idx + 1)
      + " name=" + ET_SANITIZE_TXT(phase_name)
      + " reason=" + phase_done_reason
      + " phase_t_s=" + ROUND(TIME:SECONDS - phase_start_ut, 3).
    LOG phase_done_msg TO log_file.
    PRINT "Phase done: " + phase_name + " (" + phase_done_reason + ")" AT(0, 5).

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
