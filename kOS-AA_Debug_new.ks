// ============================================================
// kOS-AA_Debug_new.ks  -  AA-assisted takeoff
//
// Phases:
//   1. Pre-flight  - brakes on, throttle full, AA limits, user go/no-go
//   2. Takeoff     - stage engines, release brakes
//   3. Takeoff roll - FBW stabilisation, monitor FAR lift, rotate at Vr
//   4. Rotation    - AA Director targets climb attitude
//   5. Climb-out   - AA Cruise (vertspeed), gear up
// ============================================================

clearscreen.
core:part:getmodule("kOSProcessor"):doevent("Open Terminal").

// ── Parameters (tune per aircraft) ──────────────────────────
local LIFT_ROTATE_FACTOR  is 0.2.   // rotate when lift >= this fraction of weight
local ROTATE_DURATION     is 3.0.   // seconds to ramp pitch from 0 → 1.0 at Vr
local ROTATE_MIN_PITCH    is 15.    // deg nose-up to hand off to Cruise; also FPA target
local CRUISE_HEADING     is 90.    // deg magnetic
local MAX_CLIMB_ANGLE    is 20.    // deg max pitch in cruise
local GEAR_UP_AGL        is 30.    // m AGL to retract gear

// ── Addon availability check ─────────────────────────────────
if not addons:available("AA") {
    print "ERROR: AtmosphereAutopilot addon not available!".
    print "Aborting.".
} else if not addons:available("FAR") {
    print "ERROR: kOS-Ferram addon not available!".
    print "Aborting.".
} else {
    runTakeoff().
}

function runTakeoff {

    local aa is addons:aa.
    set config:suppressautopilot to false.  // ensure kOS control is not suppressed

    // ── Phase 1: Pre-flight ──────────────────────────────────
    print "=== PRE-FLIGHT ===".

    // Brakes on
    brakes on.
    print "  Brakes ON".

    // Configure AA flight limits (requires FBW active briefly)
    set aa:fbw to true.
    wait 0.1.
    set aa:moderateaoa      to true.
    set aa:maxaoa           to 15.
    set aa:moderateg        to true.
    set aa:maxg             to 4.
    set aa:moderatesideslip to true.
    set aa:maxsideslip      to 5.
    set aa:moderatesideg    to true.
    set aa:maxsideg         to 2.
    set aa:fbw to false.
    wait 0.1.
    print "  AA limits set.".

    // Pre-configure cruise setpoints for after rotation.
    // FPA mode: hold ROTATE_MIN_PITCH flight path angle on climb-out.
    set aa:heading       to CRUISE_HEADING.
    set aa:fpangle       to ROTATE_MIN_PITCH.  // activates FlightPathAngle height mode
    set aa:maxclimbangle to MAX_CLIMB_ANGLE.
    print "  Cruise pre-configured: hdg=" + CRUISE_HEADING +
          "  FPA=" + ROTATE_MIN_PITCH + " deg".

    // ── Control surface test ─────────────────────────────────
    print " ".
    print "=== CONTROL SURFACE TEST ===".
    local prevSAS is sas.
    sas off.                         // SAS must be off or it fights the inputs
    wait 0.1.
    print "  Pitch UP...".
    set ship:control:pitch to  1.0.  wait 0.5.
    print "  Pitch DOWN...".
    set ship:control:pitch to -1.0.  wait 0.5.
    set ship:control:pitch to  0.0.  wait 0.2.
    print "  Roll RIGHT...".
    set ship:control:roll  to  1.0.  wait 0.5.
    print "  Roll LEFT...".
    set ship:control:roll  to -1.0.  wait 0.5.
    set ship:control:roll  to  0.0.  wait 0.2.
    print "  Yaw RIGHT...".
    set ship:control:yaw   to  1.0.  wait 0.5.
    print "  Yaw LEFT...".
    set ship:control:yaw   to -1.0.  wait 0.5.
    set ship:control:neutralize to true.
    set sas to prevSAS.              // restore SAS state
    print "  Controls OK.".

    // ── User go / no-go ─────────────────────────────────────
    print " ".
    print "Ready to launch? (y / n)".
    local ch is "".
    until ch = "y" or ch = "Y" {
        set ch to terminal:input:getchar().
        if ch = "n" or ch = "N" {
            print "Launch aborted.".
            unlock throttle.
            brakes on.
            return.
        }
    }
    print "GO!".

    // ── Phase 2: Takeoff ─────────────────────────────────────
    print " ".
    print "=== TAKEOFF ===".
    set ship:control:mainthrottle to 1.  // immediate raw set
    lock throttle to 1.0.               // lock it - takes effect next physics tick
    wait 0.                             // yield one tick so lock applies
    print "  Throttle FULL".
    stage.                               // light the engines
    print "  Engines lit.".
    wait 0.5.                            // brief pause for thrust to build

    brakes off.
    // FBW on for ground roll stabilisation. FBW and lock steering conflict,
    // so heading hold is handled by FBW's own yaw stabilisation here.
    set aa:fbw to true.
    print "  Brakes OFF  -  rolling  (FBW active)".

    // ── Phase 3: Takeoff roll - wait for Vr ──────────────────
    // Lift from FAR AEROFORCE (kN, SHIP:RAW frame) dotted with
    // the up vector gives the vertical (lift) component.
    // Rotate when lift >= LIFT_ROTATE_FACTOR * weight.
    print " ".
    print "=== TAKEOFF ROLL ===".
    local weightKN is ship:mass * 9.80665.
    local vrThreshKN is LIFT_ROTATE_FACTOR * weightKN.
    print "  Weight " + round(weightKN, 1) + " kN  |  Vr at " +
          round(LIFT_ROTATE_FACTOR * 100) + "% = " + round(vrThreshKN, 1) + " kN".

    until vdot(addons:far:aeroforce, ship:up:vector) >= vrThreshKN {
        local liftKN is vdot(addons:far:aeroforce, ship:up:vector).
        print "  Lift: " + round(liftKN, 1) + " / " + round(vrThreshKN, 1) +
              " kN    IAS: " + round(ship:airspeed, 1) + " m/s     " at (0, 15).
        wait 0.
    }
    print "  Vr - ROTATE!                                             ".

    // ── Phase 4: Rotation - raw pitch ramp until 10° nose-up ────
    // FBW is still active; raw pitch input tells FBW "pitch up at this rate".
    // Stop ramping and neutralise once the target pitch is reached.
    print " ".
    print "=== ROTATION ===".
    // pitchAboveHorizon: angle of nose above horizontal, reliable at any heading.
    local function pitchAboveHorizon {
        return 90 - vectorangle(ship:facing:forevector, ship:up:vector).
    }

    local rotStart is time:seconds.
    until pitchAboveHorizon() >= ROTATE_MIN_PITCH {
        set ship:control:pitch to min((time:seconds - rotStart) / ROTATE_DURATION, 1.0).
        wait 0.
    }
    set ship:control:neutralize to true.
    lock throttle to 1.0.   // re-assert: neutralize clears mainthrottle
    print "  " + round(pitchAboveHorizon(), 1) + " deg - handing off to Cruise".

    // ── Phase 5: Climb-out ───────────────────────────────────
    print " ".
    print "=== CLIMB-OUT ===".
    set aa:heading to CRUISE_HEADING.
    set aa:cruise to true.
    print "  Cruise ON  (FPA=" + ROTATE_MIN_PITCH + " deg  hdg " + CRUISE_HEADING + " deg)".

    until alt:radar >= GEAR_UP_AGL {
        print "  AGL=" + round(alt:radar, 1) + " / " + GEAR_UP_AGL +
              " m    Vz=" + round(ship:verticalspeed, 1) + " m/s     " at (0, 16).
        wait 0.
    }
    gear off.
    print "  Gear UP.                                              ".

    print " ".
    print "=== TAKEOFF COMPLETE ===".
    print "  Climbing on hdg " + CRUISE_HEADING + " at FPA=" + ROTATE_MIN_PITCH + " deg.".
    print "  Switch aa:altitude to target when ready for alt hold.".
    print "  (Script holding throttle - ctrl+c to release control)".
    wait until false.   // keep script alive so lock throttle stays active
}