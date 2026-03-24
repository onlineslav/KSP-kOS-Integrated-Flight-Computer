@LAZYGLOBAL OFF.

// ============================================================
// ifc_autospoiler.ks  -  Integrated Flight Computer
//
// Autospoiler module:
// - Finds spoiler-tagged parts once at startup
// - Resolves each part's deploy-angle field
// - Commands spoiler deploy angle from overspeed in CRUISE/APPROACH
// - Applies throttle-idle decel-assist gating and speed-based angle caps
// ============================================================

FUNCTION AS_RESET {
  SET AS_DISCOVERED TO FALSE.
  SET AS_AVAILABLE TO FALSE.
  SET AS_WARNED_NO_PARTS TO FALSE.
  SET AS_SPOILER_BINDINGS TO LIST().
  SET AS_CMD_DEG TO 0.
  SET AS_LAST_CAP_DEG TO 0.
  SET TELEM_AS_CMD_DEG TO 0.
  SET TELEM_AS_CAP_DEG TO 0.
  SET TELEM_AS_RAW_DEG TO 0.
  SET TELEM_AS_ERR_MPS TO 0.
  SET TELEM_AS_ACTIVE TO 0.
}

FUNCTION _AS_ENABLED {
  LOCAL en IS AS_ENABLED_DEFAULT.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("as_enabled") AND ACTIVE_AIRCRAFT["as_enabled"] >= 0 {
    SET en TO ACTIVE_AIRCRAFT["as_enabled"] <> 0.
  }
  RETURN en.
}

FUNCTION _AS_TAG {
  LOCAL tag IS AS_SPOILER_TAG_DEFAULT.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("spoiler_tag") {
    LOCAL cfg_tag IS ACTIVE_AIRCRAFT["spoiler_tag"].
    IF cfg_tag <> "" {
      SET tag TO cfg_tag.
    }
  }
  RETURN tag.
}

FUNCTION _AS_PART_LABEL {
  PARAMETER p.
  IF p = 0 { RETURN "<null part>". }

  LOCAL lbl IS "part".
  IF p:HASSUFFIX("TITLE") {
    SET lbl TO p:TITLE.
  } ELSE IF p:HASSUFFIX("NAME") {
    SET lbl TO p:NAME.
  }
  IF p:HASSUFFIX("TAG") AND p:TAG <> "" {
    SET lbl TO lbl + " [tag=" + p:TAG + "]".
  }
  RETURN lbl.
}

FUNCTION _AS_IS_SPOILER_MODULE {
  PARAMETER mod_name.
  IF mod_name = "" { RETURN FALSE. }
  LOCAL n IS mod_name:TOLOWER.
  IF n = "modulecontrolsurface" { RETURN TRUE. }
  IF n = "moduleaerosurface" { RETURN TRUE. }
  IF n = "moduleairbrake" { RETURN TRUE. }
  IF n:FIND("controlsurface") >= 0 { RETURN TRUE. }
  IF n:FIND("airbrake") >= 0 { RETURN TRUE. }
  IF n:FIND("air brake") >= 0 { RETURN TRUE. }
  RETURN FALSE.
}

FUNCTION _AS_MODULE_NAME_CANDIDATES {
  RETURN LIST(
    "ModuleControlSurface",
    "ModuleAeroSurface",
    "ModuleAirbrake",
    "modulecontrolsurface",
    "moduleaerosurface",
    "moduleairbrake"
  ).
}

FUNCTION _AS_PART_HAS_MODULE_NAME {
  PARAMETER p, wanted_name.
  IF p = 0 { RETURN FALSE. }
  IF wanted_name = "" { RETURN FALSE. }
  IF NOT p:HASSUFFIX("ALLMODULES") { RETURN FALSE. }

  LOCAL wanted_lc IS wanted_name:TOLOWER.
  LOCAL allmods IS p:ALLMODULES.
  LOCAL i IS 0.
  UNTIL i >= allmods:LENGTH {
    LOCAL entry IS allmods[i].
    SET i TO i + 1.
    IF entry <> 0 {
      LOCAL entry_name IS "".
      IF entry:HASSUFFIX("NAME") AND entry:NAME <> "" {
        SET entry_name TO entry:NAME.
      } ELSE {
        SET entry_name TO "" + entry.
      }
      IF entry_name:TOLOWER = wanted_lc { RETURN TRUE. }
    }
  }
  RETURN FALSE.
}

FUNCTION _AS_ENTRY_NAME {
  PARAMETER module_entry.
  IF module_entry = 0 { RETURN "". }
  IF module_entry:HASSUFFIX("NAME") AND module_entry:NAME <> "" {
    RETURN module_entry:NAME.
  }
  RETURN "" + module_entry.
}

FUNCTION _AS_RESOLVE_MODULE_OBJECT {
  PARAMETER p, module_entry, module_name.

  IF module_entry <> 0 AND module_entry:HASSUFFIX("HASFIELD") {
    RETURN module_entry.
  }

  IF p <> 0 AND p:HASSUFFIX("GETMODULE") AND module_name <> "" AND _AS_PART_HAS_MODULE_NAME(p, module_name) {
    LOCAL probe_mod IS p:GETMODULE(module_name).
    IF probe_mod <> 0 { RETURN probe_mod. }
  }

  RETURN module_entry.
}

FUNCTION _AS_RESOLVE_DEPLOY_FIELD {
  PARAMETER module_obj.
  IF module_obj = 0 { RETURN "". }
  IF NOT module_obj:HASSUFFIX("HASFIELD") { RETURN "". }

  LOCAL candidates IS LIST(
    "deployAngle",
    "Deploy Angle",
    "deploy angle",
    "deployangle",
    "maxDeployAngle",
    "maxdeployangle"
  ).

  LOCAL i IS 0.
  UNTIL i >= candidates:LENGTH {
    LOCAL f IS candidates[i].
    IF module_obj:HASFIELD(f) { RETURN f. }
    SET i TO i + 1.
  }
  RETURN "".
}

FUNCTION _AS_RESOLVE_DEPLOY_STATE_FIELD {
  PARAMETER module_obj.
  IF module_obj = 0 { RETURN "". }
  IF NOT module_obj:HASSUFFIX("HASFIELD") { RETURN "". }

  LOCAL candidates IS LIST(
    "deploy",
    "Deploy",
    "deployed",
    "Deployed",
    "isDeployed",
    "isdeployed"
  ).

  LOCAL idx IS 0.
  UNTIL idx >= candidates:LENGTH {
    LOCAL state_field IS candidates[idx].
    IF module_obj:HASFIELD(state_field) { RETURN state_field. }
    SET idx TO idx + 1.
  }
  RETURN "".
}

FUNCTION _AS_RESOLVE_READY_DEFLECTION_FIELD {
  PARAMETER module_obj.
  IF module_obj = 0 { RETURN "". }
  IF NOT module_obj:HASSUFFIX("HASFIELD") { RETURN "". }

  LOCAL candidates IS LIST(
    "maxDeployAngle",
    "maxdeployangle"
  ).

  LOCAL idx IS 0.
  UNTIL idx >= candidates:LENGTH {
    LOCAL ready_field IS candidates[idx].
    IF module_obj:HASFIELD(ready_field) { RETURN ready_field. }
    SET idx TO idx + 1.
  }
  RETURN "".
}

FUNCTION _AS_SET_MODULE_FIELD {
  PARAMETER module_obj, field_name, value_num.
  IF module_obj = 0 { RETURN FALSE. }
  IF NOT module_obj:HASSUFFIX("SETFIELD") { RETURN FALSE. }
  IF module_obj:HASSUFFIX("HASFIELD") AND NOT module_obj:HASFIELD(field_name) { RETURN FALSE. }

  module_obj:SETFIELD(field_name, ROUND(value_num, 3)).
  RETURN TRUE.
}

FUNCTION _AS_SET_MODULE_STATE_FIELD {
  PARAMETER module_obj, field_name, deploy_now.
  IF module_obj = 0 { RETURN FALSE. }
  IF field_name = "" { RETURN FALSE. }
  IF NOT module_obj:HASSUFFIX("SETFIELD") { RETURN FALSE. }
  IF module_obj:HASSUFFIX("HASFIELD") AND NOT module_obj:HASFIELD(field_name) { RETURN FALSE. }

  IF deploy_now {
    module_obj:SETFIELD(field_name, "True").
  } ELSE {
    module_obj:SETFIELD(field_name, "False").
  }
  RETURN TRUE.
}

FUNCTION _AS_DEBUG_DUMP_TAGGED_PART {
  PARAMETER p.
  RETURN.
}

FUNCTION _AS_APPLY_CMD {
  PARAMETER cmd_deg.
  IF NOT AS_AVAILABLE { RETURN. }
  IF AS_SPOILER_BINDINGS:LENGTH = 0 { RETURN. }
  LOCAL deploy_now IS cmd_deg > 0.05.
  LOCAL ready_deflection_deg IS AC_PARAM("as_max_deflection_deg", AS_MAX_DEFLECTION_DEG, 0).
  IF ready_deflection_deg < 0 { SET ready_deflection_deg TO 0. }

  LOCAL i IS 0.
  UNTIL i >= AS_SPOILER_BINDINGS:LENGTH {
    LOCAL b IS AS_SPOILER_BINDINGS[i].
    IF b:HASKEY("state_field") AND b["state_field"] <> "" {
      _AS_SET_MODULE_STATE_FIELD(b["mod"], b["state_field"], deploy_now).
    }
    IF deploy_now {
      _AS_SET_MODULE_FIELD(b["mod"], b["field"], cmd_deg).
    } ELSE {
      _AS_SET_MODULE_FIELD(b["mod"], b["field"], 0).
    }

    IF b:HASKEY("ready_field") AND b["ready_field"] <> "" {
      IF NOT deploy_now OR b["ready_field"] <> b["field"] {
        _AS_SET_MODULE_FIELD(b["mod"], b["ready_field"], ready_deflection_deg).
      }
    }
    SET i TO i + 1.
  }
}

FUNCTION AS_DISCOVER_PARTS {
  IF AS_DISCOVERED { RETURN. }
  SET AS_DISCOVERED TO TRUE.
  SET AS_AVAILABLE TO FALSE.
  SET AS_SPOILER_BINDINGS TO LIST().

  IF NOT _AS_ENABLED() {
    IFC_SET_ALERT("AS disabled (as_enabled = 0)").
    RETURN.
  }

  LOCAL tag IS _AS_TAG().
  LOCAL tag_lc IS tag:TOLOWER.
  LOCAL tagged_count IS 0.
  LOCAL bound_count IS 0.
  LOCAL skipped_count IS 0.
  LOCAL parts IS SHIP:PARTS.

  LOCAL i IS 0.
  UNTIL i >= parts:LENGTH {
    LOCAL p IS parts[i].
    SET i TO i + 1.

    LOCAL is_tagged IS FALSE.
    IF p <> 0 AND p:HASSUFFIX("TAG") AND p:TAG <> "" AND p:TAG:TOLOWER = tag_lc {
      SET is_tagged TO TRUE.
    }

    IF is_tagged {
      SET tagged_count TO tagged_count + 1.

      IF NOT p:HASSUFFIX("MODULES") {
        SET skipped_count TO skipped_count + 1.
      } ELSE {
        LOCAL mods IS p:MODULES.
        LOCAL bound IS FALSE.

        // First try direct module lookup by well-known KSP module names.
        IF p:HASSUFFIX("GETMODULE") {
          LOCAL names IS _AS_MODULE_NAME_CANDIDATES().
          LOCAL k IS 0.
          UNTIL k >= names:LENGTH {
            LOCAL nm IS names[k].
            SET k TO k + 1.
            IF _AS_PART_HAS_MODULE_NAME(p, nm) {
              LOCAL probe_mod IS p:GETMODULE(nm).
              LOCAL probe_field IS _AS_RESOLVE_DEPLOY_FIELD(probe_mod).
              IF probe_field <> "" {
                LOCAL probe_state_field IS _AS_RESOLVE_DEPLOY_STATE_FIELD(probe_mod).
                LOCAL probe_ready_field IS _AS_RESOLVE_READY_DEFLECTION_FIELD(probe_mod).
                LOCAL probe_deploy_label IS "<none>".
                LOCAL probe_ready_label IS "<none>".
                IF probe_state_field <> "" { SET probe_deploy_label TO probe_state_field. }
                IF probe_ready_field <> "" { SET probe_ready_label TO probe_ready_field. }
                AS_SPOILER_BINDINGS:ADD(LEXICON(
                  "part", p,
                  "mod", probe_mod,
                  "module_name", nm,
                  "field", probe_field,
                  "state_field", probe_state_field,
                  "ready_field", probe_ready_field
                )).
                SET bound_count TO bound_count + 1.
                SET bound TO TRUE.
                BREAK.
              }
            }
          }
        }

        LOCAL j IS 0.
        UNTIL j >= mods:LENGTH OR bound {
          LOCAL m IS mods[j].
          SET j TO j + 1.

          IF m <> 0 {
            LOCAL mod_name IS _AS_ENTRY_NAME(m).
            LOCAL module_obj IS _AS_RESOLVE_MODULE_OBJECT(p, m, mod_name).

            IF _AS_IS_SPOILER_MODULE(mod_name) {
              LOCAL field_name IS _AS_RESOLVE_DEPLOY_FIELD(module_obj).
              IF field_name <> "" {
                LOCAL state_field IS _AS_RESOLVE_DEPLOY_STATE_FIELD(module_obj).
                LOCAL ready_field IS _AS_RESOLVE_READY_DEFLECTION_FIELD(module_obj).
                LOCAL deploy_label IS "<none>".
                LOCAL ready_label IS "<none>".
                IF state_field <> "" { SET deploy_label TO state_field. }
                IF ready_field <> "" { SET ready_label TO ready_field. }
                AS_SPOILER_BINDINGS:ADD(LEXICON(
                  "part", p,
                  "mod", module_obj,
                  "module_name", mod_name,
                  "field", field_name,
                  "state_field", state_field,
                  "ready_field", ready_field
                )).

                SET bound_count TO bound_count + 1.
                SET bound TO TRUE.
                BREAK.
              }
            }
          }
        }

        IF NOT bound {
          SET skipped_count TO skipped_count + 1.
          _AS_DEBUG_DUMP_TAGGED_PART(p).
        }
      }
    }
  }

  IF bound_count > 0 {
    SET AS_AVAILABLE TO TRUE.
  } ELSE {
    IF NOT AS_WARNED_NO_PARTS {
      IFC_SET_ALERT("AS disabled: no tagged spoiler parts for '" + tag + "'", "WARN").
      SET AS_WARNED_NO_PARTS TO TRUE.
    }
  }
}

FUNCTION _AS_PHASE_CAP_DEG {
  PARAMETER ias, phase_key.

  LOCAL s_lo IS 0.
  LOCAL s_hi IS 1.
  LOCAL cap_lo IS 0.
  LOCAL cap_hi IS 0.

  IF phase_key = "CRUISE" {
    SET s_lo   TO AC_PARAM("as_crz_speed_lo",    AS_CRZ_SPEED_LO,    0).
    SET s_hi   TO AC_PARAM("as_crz_speed_hi",    AS_CRZ_SPEED_HI,    0.001).
    SET cap_lo TO AC_PARAM("as_crz_cap_deg_lo",  AS_CRZ_CAP_DEG_LO,  0).
    SET cap_hi TO AC_PARAM("as_crz_cap_deg_hi",  AS_CRZ_CAP_DEG_HI,  0).
  } ELSE {
    SET s_lo   TO AC_PARAM("as_app_speed_lo",    AS_APP_SPEED_LO,    0).
    SET s_hi   TO AC_PARAM("as_app_speed_hi",    AS_APP_SPEED_HI,    0.001).
    SET cap_lo TO AC_PARAM("as_app_cap_deg_lo",  AS_APP_CAP_DEG_LO,  0).
    SET cap_hi TO AC_PARAM("as_app_cap_deg_hi",  AS_APP_CAP_DEG_HI,  0).
  }

  IF s_hi <= s_lo { SET s_hi TO s_lo + 0.001. }
  SET cap_lo TO MAX(cap_lo, 0).
  SET cap_hi TO MAX(cap_hi, 0).

  IF ias <= s_lo { RETURN cap_lo. }
  IF ias >= s_hi { RETURN cap_hi. }

  LOCAL frac IS (ias - s_lo) / (s_hi - s_lo).
  RETURN cap_lo + (cap_hi - cap_lo) * frac.
}

FUNCTION AS_RUN {
  PARAMETER v_tgt, phase_key.

  IF NOT _AS_ENABLED() {
    AS_RELEASE().
    RETURN.
  }

  IF NOT AS_DISCOVERED { AS_DISCOVER_PARTS(). }
  IF NOT AS_AVAILABLE {
    SET TELEM_AS_ACTIVE TO 0.
    RETURN.
  }

  LOCAL ias IS GET_IAS().
  LOCAL overspeed IS ias - v_tgt.
  LOCAL deadband IS AC_PARAM("as_err_deadband_mps", AS_ERR_DEADBAND_MPS, 0).
  LOCAL full_err IS AC_PARAM("as_err_full_mps", AS_ERR_FULL_MPS, deadband + 0.001).
  IF full_err <= deadband { SET full_err TO deadband + 0.001. }

  LOCAL thr_gate IS AC_PARAM("as_thr_idle_gate", AS_THR_IDLE_GATE, 0).
  LOCAL cap_deg IS _AS_PHASE_CAP_DEG(ias, phase_key).

  LOCAL raw_deg IS 0.
  IF overspeed > deadband AND THROTTLE_CMD <= thr_gate {
    LOCAL frac IS (overspeed - deadband) / (full_err - deadband).
    SET raw_deg TO CLAMP(frac, 0, 1) * cap_deg.
  }

  LOCAL slew_dps IS AC_PARAM("as_angle_slew_dps", AS_ANGLE_SLEW_DPS, 0.001).
  LOCAL cmd_deg IS MOVE_TOWARD(AS_CMD_DEG, raw_deg, slew_dps * IFC_ACTUAL_DT).
  SET AS_CMD_DEG TO cmd_deg.
  SET AS_LAST_CAP_DEG TO cap_deg.
  _AS_APPLY_CMD(cmd_deg).

  SET TELEM_AS_CMD_DEG TO cmd_deg.
  SET TELEM_AS_CAP_DEG TO cap_deg.
  SET TELEM_AS_RAW_DEG TO raw_deg.
  SET TELEM_AS_ERR_MPS TO overspeed.
  IF cmd_deg > 0.05 {
    SET TELEM_AS_ACTIVE TO 1.
  } ELSE {
    SET TELEM_AS_ACTIVE TO 0.
  }
}

FUNCTION AS_RELEASE {
  IF NOT AS_DISCOVERED { RETURN. }
  IF NOT AS_AVAILABLE {
    SET AS_CMD_DEG TO 0.
    SET TELEM_AS_CMD_DEG TO 0.
    SET TELEM_AS_RAW_DEG TO 0.
    SET TELEM_AS_ERR_MPS TO 0.
    SET TELEM_AS_ACTIVE TO 0.
    RETURN.
  }

  SET AS_CMD_DEG TO 0.
  _AS_APPLY_CMD(0).
  SET TELEM_AS_CMD_DEG TO 0.
  SET TELEM_AS_RAW_DEG TO 0.
  SET TELEM_AS_ERR_MPS TO 0.
  SET TELEM_AS_ACTIVE TO 0.
}

// Force touchdown spoiler deployment to maximum configured deploy angle.
// Used by autoland when touchdown spoilers are commanded.
FUNCTION AS_DEPLOY_MAX_TOUCHDOWN {
  IF NOT _AS_ENABLED() { RETURN. }

  IF NOT AS_DISCOVERED { AS_DISCOVER_PARTS(). }
  IF NOT AS_AVAILABLE { RETURN. }

  LOCAL max_deg IS AC_PARAM("as_max_deflection_deg", AS_MAX_DEFLECTION_DEG, 0).
  SET max_deg TO MAX(max_deg, 0).

  SET AS_CMD_DEG TO max_deg.
  SET AS_LAST_CAP_DEG TO max_deg.
  _AS_APPLY_CMD(max_deg).

  SET TELEM_AS_CMD_DEG TO max_deg.
  SET TELEM_AS_CAP_DEG TO max_deg.
  SET TELEM_AS_RAW_DEG TO max_deg.
  SET TELEM_AS_ACTIVE TO 1.
}
