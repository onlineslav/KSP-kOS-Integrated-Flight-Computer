@LAZYGLOBAL OFF.

// ============================================================
// cf1_j_cfg.ks  -  Integrated Flight Computer aircraft config
//
// Comment format (standardized):
//   // [unit]   Description   (global)
//
// Global column:
//   (value) when a global fallback exists; blank otherwise.
// ============================================================

FUNCTION BUILD_AIRCRAFT_CONFIG {
  RETURN LEXICON(

    // ========================================================
    // 1) Identity + Airframe Envelope
    // ========================================================
    "name", "C-1000F SkyDiva",                   // [string]          Aircraft display name shown in IFC status and logs.                              
//  "notes", "X10-H Spaceplane",              // [string]          Free-text note shown at IFC startup for quick context.                           UNUSED

//  "vs0", 92.0,                              // [m/s]             Landing-configuration stall speed estimate used for protection/scheduling logic. UNUSED
    "a_crit", 18.0,                            // [deg]             FAR critical AoA. 0 disables AoA-ratio based protections until tuned.            
    "tailstrike_pitch_max_deg", 7.0,         // [deg]             Absolute pitch cap near runway to protect tail.                                  (20.0)

    // AtmosphereAutopilot moderator limits (per-aircraft overrides).
    "aa_max_aoa", 18,                         // [deg]             Max commanded AoA in FBW director.                                               (12)
    "aa_max_g", 3.5,                           // [g]               Max normal load factor in FBW.                                                   (3.5)
    "aa_max_sideslip", -1,                    // [deg]             Max allowable sideslip in FBW.                                                   (5)
    "aa_max_side_g", -1,                      // [g]               Max lateral load factor in FBW.                                                  (1.5)
    "aa_max_bank", -1,                        // [deg]             Max bank angle allowed by FBW.                                                   (35)

    // ========================================================
    // 2) Hardware / Action-Group Mapping
    // ========================================================
    "has_nws", TRUE,                          // [bool]            TRUE when the aircraft has usable nose-wheel steering.                           
    "ag_flaps_step_up", 9,                    // [AG#]             FAR flap detent step-up action group (1..10, 0 disables).                        
    "ag_flaps_step_down", 10,                 // [AG#]             FAR flap detent step-down action group (1..10, 0 disables).                      
    "ag_spoilers", 7,                         // [AG#]             Touchdown spoiler/airbrake deploy action group (1..10, 0 disables).              
    "ag_thrust_rev", 8,                       // [AG#]             Thrust reverser action group used in rollout (1..10, 0 disables).                
    "ag_drogue", 0,                           // [AG#]             Drogue chute deploy action group (1..10, 0 disables).    

    // Spoiler arming and autospoiler tuning (from X11-D baseline).
    "spoiler_tag", "ifc_spoiler",             // [string]          Part tag used by autospoiler discovery.
    "as_enabled", 1,                          // [bool]            1 enables autospoiler, 0 disables.
    "as_thr_idle_gate", 0.08,                 // [0..1]            Deploy assist only when THROTTLE_CMD <= gate.
    "as_err_deadband_mps", 1.5,               // [m/s]             Overspeed deadband before spoiler response starts.
    "as_err_full_mps", 18.0,                  // [m/s]             Overspeed that commands full capped deflection.
    "as_angle_slew_dps", 25.0,                // [deg/s]           Deploy-angle slew limit.
    "as_max_deflection_deg", 70,              // [deg]             Deploy angle written when autospoiler is not actively deploying.
    "as_speed_lo", 100,                         // [m/s]             Low speed point for cap schedule.
    "as_speed_hi", 250,                        // [m/s]             High speed point for cap schedule.
    "as_cap_deg_lo", 55,                       // [deg]             Cap at/below as_speed_lo.
    "as_cap_deg_hi", 25,                       // [deg]             Cap at/above as_speed_hi.
    "ag_spoilers_arm", 0,                     // [AG#]             AG to arm spoilers in-flight (0 disables).
    "app_spoiler_arm_km", 0,                  // [km]              Distance from threshold to arm spoilers in flight (0 disables).                        

    // ========================================================
    // 3) Takeoff
    // ========================================================
    "v_r", 85.0,                             // [m/s]             Rotation speed target.                                                           (70)
    "v2", 105.0,                              // [m/s]             Climb safety speed target after liftoff.                                         (80)
    "takeoff_pitch_tgt", 6.0,                // [deg]             Initial pitch target used through rotation.                                      (12)
    "takeoff_pitch_slew_dps", 1,            // [deg/s]           Slew limit applied to rotation pitch target changes.                             (3.0)

    // On-wheels rotation control shaping.
    "takeoff_rotate_pitch_kp", -1,          // [cmd/deg]         Pitch command gain versus pitch error while weight-on-wheels.                    (0.08)
    "takeoff_rotate_pitch_ff", -1,          // [cmd]             Feed-forward back-pressure to initiate smooth rotation.                          (0.08)
    "takeoff_rotate_pitch_min_cmd", -1,     // [cmd]             Minimum nose-up command while below rotation target.                             (0.12)
    "takeoff_rotate_pitch_max_cmd", -1,     // [cmd]             Maximum nose-up command while on wheels to limit strike risk.                    (0.45)
    "takeoff_rotate_pitch_slew_per_s", -1,   // [cmd/s]           Slew limit for pitch command during rotation.                                    (1.6)

    "takeoff_climb_fpa", 8.0,                 // [deg]             Climb-out flight-path-angle command after rotation.                              (8.0)
    "takeoff_throttle", 1.0,                  // [0..1]            Takeoff throttle command during roll/rotation.                                   
    "takeoff_done_agl", 250.0,                // [m AGL]           AGL threshold to exit takeoff phase and hand to next mode.                       (300)
    "takeoff_airborne_agl", -1,              // [m AGL]           Minimum AGL used in airborne detection gate.                                     (3)
    "takeoff_airborne_min_vs", -1,           // [m/s]             Minimum vertical speed used in airborne detection gate.                          (0.5)

    // Preflight spool and optional autostage.
    "takeoff_autostage", -1,                   // [bool]            Enables pre-roll auto-stage retry logic when thrust is missing.                  (TRUE)
    "takeoff_stage_max_attempts", -1,          // [count]           Max staging attempts before takeoff abort/fail branch.                           (1)
    "takeoff_stage_retry_s", -1,             // [s]               Delay between auto-stage attempts.                                               (1.0)
    "takeoff_engine_spool_timeout_s", 8,   // [s]               Max wait for engines to reach valid thrust before giving up.                     (8.0)
    "takeoff_spool_thrust_frac", -1,       // [0..1]            Required fraction of available thrust before brake release.                      (0.95)
    "takeoff_spool_steady_dknps", -1,        // [kN/s]            Allowed |d(thrust)/dt| to declare thrust steady.                                 (3.0)
    "takeoff_spool_steady_hold_s", -1,       // [s]               Time thrust must remain steady before launch roll starts.                        (0.6)
    "takeoff_flap_settle_s", -1,             // [s]               Hold time after final flap command before brake release.                         (2.0)
    "takeoff_min_avail_thrust", -1,          // [kN]              Minimum available thrust to count engines as lit.                                (5.0)

    // Ground steering and rudder assist.
    "takeoff_loc_kp", -1,                  // [deg/m]           Centerline correction gain from lateral LOC error.                               (0.020)
    "takeoff_loc_guard_m", -1,             // [m]               LOC error clamp used before steering conversion.                                 (120.0)
    "takeoff_steer_max_corr", -1,           // [deg]             Max heading correction via wheel steering channel.                               (10.0)
    "takeoff_steer_hdg_rate_kd", -1,        // [deg/(deg/s)]     Heading-rate damping to suppress runway weave.                                   (0.20)
//  "takeoff_dir_max_corr", 6.0,              // [deg]             Max heading correction passed to director channel.                               (6.0) UNUSED
    "takeoff_yaw_start_ias", -1,             // [m/s]             IAS where rudder-assist blending begins.                                         (20.0)
    "takeoff_yaw_full_ias", -1,             // [m/s]             IAS where rudder-assist reaches full authority.                                  (90.0)
    "takeoff_yaw_min_scale", -1,            // [0..1]            Minimum rudder gain floor at low IAS during rollout.                             (0.25)
    "takeoff_yaw_kp", -1,                  // [cmd/deg]         Rudder gain versus heading error for runway tracking.                            (0.025)
    "takeoff_yaw_kd", -1,                  // [cmd/(deg/s)]     Rudder damping gain versus heading rate.                                         (0.00)
    "takeoff_yaw_boost_err_deg", -1,        // [deg]             Heading error where boost term reaches full contribution.                        (0.50)
    "takeoff_yaw_boost_max", -1,            // [unitless]        Maximum extra yaw gain added by error-based boost.                               (0.80)
    "takeoff_yaw_max_cmd", -1,              // [cmd]             Absolute rudder command clamp during takeoff.                                    (0.30)
    "takeoff_yaw_slew_per_s", -1,            // [cmd/s]           Slew limit for yaw command to avoid snap inputs.                                 (2.0)
    "takeoff_yaw_sign", -1,                   // [sign]            Control-sign convention for this airframe (+1 or -1).                            

    // Climb speed-protection behavior after liftoff.
    "takeoff_climb_min_throttle", -1,       // [0..1]            Throttle floor while in climb speed-protection logic.                            (0.78)
    "takeoff_climb_spd_thr_gain", -1,      // [thr/(m/s)]       Throttle trim gain versus speed error relative to V2.                            (0.010)
    "takeoff_climb_fpa_spd_gain", -1,       // [deg/(m/s)]       FPA reduction per m/s below V2 to protect energy.                                (0.08)
    "takeoff_climb_fpa_min", -1,             // [deg]             Minimum allowed climb FPA under speed protection.                                (3.0)
    "takeoff_climb_fpa_slew_dps", -1,        // [deg/s]           Slew rate for rotate-to-climb FPA transition.                                    (1.6)
    "takeoff_aoa_protect_frac", -1,         // [fraction]        AoA warning threshold as fraction of a_crit.                                     (0.85)
    "takeoff_aoa_fpa_gain", -1,             // [deg/deg]         FPA pull-down gain per degree AoA above warning threshold.                       (0.90)

    // ========================================================
    // 4) Approach
    // ========================================================
    "v_app", 88.0,                           // [m/s]             Target IAS from intercept through stabilized approach.       100m/s surf                    
    "v_ref", 78.0,                           // [m/s]             Reference IAS near threshold crossing.                                           

    // Enroute-to-intercept speed-gate overrides.
    "app_spd_enroute_target", -1,          // [m/s]             Enroute IAS target before intercept gate arms (<=0 uses CRUISE_SPD_MPS/current IAS). (-1.0)
    "app_spd_intercept_arm_dist_m", 45000.0,  // [m]               Distance-to-threshold gate to arm intercept-speed decel.                         (30000.0)
    "app_spd_intercept_arm_alt_m", 1200.0,    // [m ASL]           Altitude gate to arm intercept-speed decel regardless of range.                  (2600.0)
    "app_spd_intercept_release_factor", 1.25, // [unitless]        Hysteresis multiplier for releasing/holding intercept-speed gate state.          (1.25)

    // Intercept and short-final speed scheduler.
    "app_spd_intercept_gain", 5.0,           // [unitless]        Intercept add gain: Vint = Vapp + clamp((Vapp-Vref)*gain, min, max).             (0.60)
    "app_spd_intercept_min_add", 15.0,         // [m/s]             Minimum speed margin added during intercept segment.                             (4.0)
    "app_spd_intercept_max_add", 15.0,        // [m/s]             Maximum speed margin added during intercept segment.                             (9.0)
    "app_short_final_agl", -1,             // [m AGL]           AGL where schedule starts blending toward short-final behavior.                  (60.0)
    "app_speed_tgt_slew_per_s", -1,          // [(m/s)/s]         Slew limit for commanded approach speed target.                                  (0.8)
    "app_short_final_cap", -1,                 // [bool]            Forces short-final cap when capture flags are noisy.                             (TRUE)

    // Gear extension policy on approach.
    "gear_down_agl", 300,                     // [m AGL]           Auto-gear deploy trigger height; set 0 for manual gear management.               
    "gear_max_extend_ias", 120,               // [m/s]             IAS limit for automatic gear extension.                                          (120)

    // Flap schedule and limits.
    "flaps_initial_detent", 0,                // [detent]          Expected flap detent when IFC engages.                                           
    "flaps_detent_up", 0,                     // [detent]          Flap detent index for clean/up configuration.                                    
    "flaps_detent_climb", 1,                  // [detent]          Flap detent index for climb configuration.                                       
    "flaps_detent_takeoff", 1,                // [detent]          Flap detent index for takeoff configuration.                                     
    "flaps_detent_approach", 2,               // [detent]          Flap detent index for approach configuration.                                    
    "flaps_detent_landing", 3,                // [detent]          Flap detent index for landing configuration.                                     
    "flaps_max_detent", 3,                    // [detent]          Highest valid flap detent index.                                                 
    "vfe_climb", 170,                         // [m/s]             Max IAS permitted in climb flap detent.                                          
    "vfe_approach", 130,                      // [m/s]             Max IAS permitted in approach flap detent.                                       
    "vfe_landing", 110,                       // [m/s]             Max IAS permitted in landing flap detent.                                        
    "flaps_climb_km", 45,                     // [km]              Distance cue for climb-detent selection.                                         
    "flaps_approach_km", 30,                  // [km]              Distance cue for approach-detent selection.                                      
    "flaps_landing_km", 8,                   // [km]              Distance cue for landing-detent selection.                                       

    // ========================================================
    // 5) Landing (Flare / Touchdown / Rollout)
    // ========================================================
    // Flare geometry and command shaping.
    "flare_gear_tag", "",         // [string]          Part tag used as main-gear height reference.                                     ("ifc_maingear")
    "flare_ctrl_h_offset_max_m", -1,        // [m]               Max captured control-height offset (runway height minus gear height).            (30.0)
    "flare_agl", 20,                          // [m]               Runway-relative trigger height to enter flare phase.                             (25)
    "flare_entry_vs_min", -1,               // [m/s]             Minimum flare-entry sink retained from approach.                                  (-6.0)
    "flare_touchdown_vs", -0.40,              // [m/s]             Target sink rate at touchdown.                                                   (-0.3)
    "flare_cmd_fpa_min", -1,                // [deg]             Lower clamp on flare FPA command.                                                (-6.0)
    "flare_cmd_fpa_max", 3.5,                 // [deg]             Upper clamp on flare FPA command before AoA compensation.                        (4.0)
    "flare_cmd_rate_min_dps", -1,            // [deg/s]           Minimum flare command rate at low speed.                                         (0.8)
    "flare_cmd_rate_max_dps", -1,            // [deg/s]           Maximum flare command rate at high speed.                                        (2.2)
    "flare_roundout_start_h_m", 5.0,         // [m]               Height where roundout blend begins.                                              (8.0)
    "flare_roundout_end_h_m", 0.5,            // [m]               Height where roundout blend completes near touchdown.                            (0.8)
    "flare_roundout_curve", -1,             // [unitless]        Roundout curve exponent shaping float vs settle behavior.                        (1.0)
    "flare_roundout_ttg_start_s", 2.0,         // [s]               Time-to-ground where roundout blend starts.                                       (3.0)
    "flare_roundout_ttg_end_s", 0.5,           // [s]               Time-to-ground where roundout blend reaches full effect.                          (0.8)
    "flare_disable_speed_bleed", 0,           // [bool]            TRUE disables extra sink augmentation from speed-above-Vref term.                (TRUE)
    "flare_min_throttle", -1,               // [0..1]            Throttle floor in flare prior to low-altitude blend/recovery.                    (0.0)
    "flare_min_throttle_agl_blend", -1,      // [m]               Height where throttle floor blends down near touchdown.                          (8.0)

    // Flare authority monitor and recovery.
    "flare_authority_vs_err_trigger", -1,    // [m/s]             VS error trigger for authority-limited detection.                                (0.8)
    "flare_authority_pitch_err_trigger", -1, // [deg]             Pitch tracking error trigger for authority-limited detection.                    (2.0)
    "flare_authority_fpa_err_trigger", -1,   // [deg]             FPA tracking error trigger for authority-limited detection.                      (1.0)
    "flare_authority_detect_s", -1,          // [s]               Required persistence before authority-limited latch.                             (0.35)
    "flare_authority_recovery_gain", -1,    // [unitless]        Gain for flare recovery branch after authority-limited latch.                    (0.20)
    "flare_balloon_vs_trigger", -1,           // [m/s]             Upward VS threshold that latches anti-balloon supervision.                        (0.2)
    "flare_balloon_clear_vs", -1,             // [m/s]             VS threshold that clears anti-balloon latch once descending.                      (-0.2)
    "flare_balloon_min_h_m", -1,              // [m]               Control-height floor below which anti-balloon latch is ignored.                   (3.0)
    "flare_balloon_gamma_down_deg", -1,       // [deg]             Minimum nose-down gamma command while anti-balloon is active.                     (-3.0)

    // Flare TECS tuning.
    "flare_tecs_et_kp", -1,                   // [thr/(m^2/s^2)]   Throttle proportional gain on total-energy error.                                (0.00008)
    "flare_tecs_et_ki", -1,                   // [thr/(m^2/s^2*s)] Throttle integrator gain on total-energy error.                                  (0.00002)
    "flare_tecs_eb_kp", -1,                // [unitless]        Gamma channel proportional gain on energy-balance error.                         (0.59)
    "flare_tecs_eb_ki", -1,                  // [1/s]             Gamma channel integrator gain on energy-balance error.                           (0.063)
    "flare_tecs_et_int_lim", -1,              // [(m^2/s^2)*s]     Integrator clamp for total-energy channel.                                       (8000)
    "flare_tecs_eb_int_lim", -1,            // [(m^2/s^2)*s]     Integrator clamp for energy-balance channel.                                     (5000)
    "flare_tecs_thr_trim", -1,             // [0..1]            Baseline throttle trim in flare TECS.                                            
    "flare_tecs_thr_bal_k", -1,          // [thr/(m^2/s^2)]   Throttle bias coupling from balance-error channel.                               (0.00008)
    "flare_tecs_thr_slew_per_s", -1,          // [thr/s]           Slew limit on TECS throttle command.                                             (1.2)
    "flare_tecs_climb_vs_gate", -1,         // [m/s]             VS gate that forces throttle floor when climbing in flare.                       (0.2)

    // Touchdown commit debounce.
    "touchdown_confirm_s", -1,              // [s]               Confirm time required before touchdown state commits.                            (0.12)
    "touchdown_confirm_max_abs_vs", -1,      // [m/s]             Max |VS| allowed while confirming touchdown contact.                             (2.5)

    // Rollout and bounce recovery behavior.
    "rollout_brake_max_ias", 70,             // [m/s]             Above this IAS, wheel brakes are withheld to avoid instability.                  (70)
    "rollout_yaw_assist_ias", 95,            // [m/s]             IAS below which rollout yaw assist is active.                                    (95)
    "rollout_roll_assist_ias", 95,             // [m/s]             IAS below which rollout roll assist is active (0 disables).                      (95)
    "rollout_steer_min_blend", -1,          // [0..1]            Minimum steering blend floor retained at high speed.                             (0.10)
    "rollout_yaw_sign", -1,                   // [sign]            Control-sign convention for rollout yaw command (+1 or -1).                      
    "rollout_yaw_kp", -1,                  // [cmd/deg]         Rollout yaw gain versus heading error.                                           (0.05)
    "rollout_yaw_slew_per_s", -1,            // [cmd/s]           Slew limit for rollout yaw command.                                              (2.5)
    "rollout_yaw_fade_ias", -1,               // [m/s]             IAS where yaw-assist output fades toward zero.                                   (20)
    "rollout_yaw_max_cmd", -1,              // [cmd]             Max magnitude of rollout yaw command.                                            (0.50)
    "rollout_touchdown_settle_s", -1,       // [s]               Post-touchdown settle delay before full rollout logic.                           (0.20)
    "touchdown_nose_hold_s", -1,            // [s]               Hold main-gear touchdown pitch before commanded nose-lowering begins.            (0.15)
    "touchdown_nose_lower_rate_dps", -1,     // [deg/s]           Max commanded de-rotation rate toward rollout_nose_target_pitch_deg.             (2.0)
    "bounce_recovery_agl_m", -1,             // [m]               Max AGL for bounce-recovery logic to engage.                                     (2.5)
    "bounce_recovery_min_vs", -1,            // [m/s]             Minimum positive VS that flags a bounce condition.                               (0.6)
    "bounce_recovery_confirm_s", -1,        // [s]               Required bounce persistence before recovery mode latches.                        (0.30)
    "bounce_recovery_max_s", -1,             // [s]               Max time bounce recovery remains active before fallback.                         (6.0)
    "rollout_nose_hold_cmd", 0,            // [cmd]             Initial nose-up hold command during early rollout.                               (0.0)
    "rollout_nose_release_ias", -1,           // [m/s]             IAS where nose-hold behavior is released.                                        (35)
    "rollout_nose_hold_min_s", -1,           // [s]               Minimum hold time before allowing nose-release logic.                            (1.0)
//  "rollout_nose_min_ref_deg", 4.0,          // [deg]             Minimum pitch reference while nose-hold is active.                               (2.0) UNUSED
    "rollout_nose_target_pitch_deg", -1,     // [deg]             Target pitch once nose transitions toward runway attitude.                       (0.0)
    "rollout_nose_target_slew_dps", 3.0,     // [deg/s]           Slew rate for nose target pitch transition.                                      (1.2)
    "rollout_pitch_hold_kp", -1,            // [cmd/deg]         Pitch-hold gain during rollout attitude control.                                 (0.08)
    "rollout_pitch_max_cmd", -1,            // [cmd]             Max upward pitch command in rollout controller.                                  (0.35)
    "rollout_pitch_max_down_cmd", 0.25,       // [cmd]             Max downward pitch command in rollout controller.                                (0.18)
    "rollout_pitch_slew_per_s", 3.0,          // [cmd/s]           Slew limit for rollout pitch command output.                                     (1.2)

    // ========================================================
    // 6) Ascent
    // ========================================================
    "ascent_q_target", -1,                    // [Pa]              Dynamic-pressure corridor center target.                                         (30000)
    "ascent_q_max", -1,                       // [Pa]              Upper dynamic-pressure limit for structural protection.                          (60000)
    "ascent_q_min", -1,                       // [Pa]              Lower dynamic-pressure bound to avoid over-lofting.                              (8000)
    "ascent_heat_limit", -1,                  // [Pa*m/s]          Heating proxy limit used by ascent energy management.                            (3.5e8)
    "ascent_k_prop", -1,                      // [unitless]        Propellant equivalency coefficient for ascent planner.                           (0.5)
    "ascent_aoa_limit", -1,                   // [deg]             Ascent AoA clamp.                                                                (12)
    "ascent_regime_mach", -1,                 // [Mach]            Transition Mach between ascent thermal/prop regimes.                             (4.5)
    "ascent_zoom_target_m", -1,               // [m]               Apoapsis target for zoom/climb transition.                                       (45000)
    "ascent_apoapsis_target_m", -1
  ).
}
