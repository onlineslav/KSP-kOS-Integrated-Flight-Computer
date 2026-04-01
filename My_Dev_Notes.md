# To-Do

- Add maximum maneuvering speed as an aircraft parameter.
- Add all of Van's runways to nav beacons with sensible approaches.
- Reorganize aircraft cfg files by phase order:
  - Preflight
  - Takeoff
  - Cruise
  - Approach
  - Landing
- Group VFE / extension-speed limits clearly in cfg files.
- Ensure every aircraft has a default cruise speed for cruise phases.
- Show target vs actual values in FMS terminal/GUI (heading, altitude, speed, etc.).
- During preflight, find parts with "Disable Containment" and set containment off.
- Fix fix-skipping logic: only skip a fix when it is truly behind (10 deg cone).

# Ideas
- smart waypoint "capture radius" that is based on the speed of the aircraft or something? So that when it goes from one leg to another, it activates the next leg in time so that it is able to get onto the course properly (currently it's just set at 1.5km)
- each ILS has a default GS, as well as a lower limit GS (if terrain is in the way), but GS can be set manually for each approach (so that spaceplanes that glide at high flight path angles can utilize them)
- Add a bank angle limiter that fights AA - see if it can work
- Have a "kOS augmented" manual FBW mode
  - Has automatic crash prevention
- ensure brakes cannot activate until nosegear landed (incl debounce - maybe 0.5s)
- if aircraft dips below some altitude (AGL) floor while on ILS, climb to regain glideslope as opposed to simply holding alt until GS recaptured
- Modulate spoiler deflection using both speed error and aerodynamic pressure (q).
- Add autobrakes that modulate braking percentage dynamically.
- Add a manual-control button that keeps IFC running without fully exiting.
- Decide whether climb/descent should be explicit phases or transitional states.
- Add a per-aircraft max landing weight, and automatically dump fuel to get below that weight.
- Display current AoA in the terminal
- stall protection? if aircraft approaching Vs, disregard commanded speed and increase to safe minimum speed
- also add minimum flap speeds? like below a certain speed, the aircraft must have a certain degree of flaps (for AoA margin)
- add what the default values actually are to aircraft cfg template (and all other ones)
- be able to enable/disable what actually gets logged - I feel like a lot of stuff that gets logged was only useful for debugging the code and useless now that a lot of it is working
  - furthermore, actual grouping of logged values, so grouping of control surface deflections (rudders, ailerons, etc.)
  - it would be useful to know what currently has control of the vessel: AA cruise, AA kOS, what AA mode is on, etc.
- - ability to skip current leg in flight plan (while in flight) or edit flight plan while in flight. Should have a way of indicating what is the current active next waypoint.
- able to list nearest waypoint to airport
- should be able to select which type of speed (Mach or IAS)

- be able to change top-of-descent fpa (while planning and during flight)

- ground handling mode?
  - some aircraft don't have steerable nose gear. would be good to use diff braking and/or diff thrust to turn the aircraft on the ground. Note: this would be a feature of the "enhanced manual mode" 

- give autospoilers a decel m/s^2 limit based off of the AA g-limit. Allow them whatever deflection they want (under max deflect angle) as long as it doesn't breach the g-limit.

# Planned

## IFC Terminal/GUIs

- Be able to edit flight plan while in flight - add or remove waypoints
  - Also able to change speeds and altitudes of legs in flightplan.
- Improve waypoint skip logic:
  - only allow auto waypoint skip for approach logic, and only allow waypoint skip if...
- Allow both Mach and IAS in cruise GUI
- Be able to select initial approach fixes as waypoint for cruise destination

## IFC Behaviour
- Auto-descent in order to get to next waypoint at correct altitude (including approach plates)
  - Say you have: Takeoff, Cruise to approach IAF, approach - If the IAF is at a much lower altitude, I want the aircraft to descend early enough so that it reaches the IAF correctly.
    - Could specify a cruise->descent maximum negative vert speed in cfgs.
- Handle the case correctly where if waypoint 1->2->3 form an acute triangle (so to say), that 3 won't get skipped once AC gets to waypoint 2 just because 2 is "behind" the aircraft.

# Augmented Manual Operation (AMO) Mode

## Modules

### Ground Steering Assist
This module is a feature of Augmented Manual Operation (AMO) Mode. When a flight plan is not active, this is the default state the aircraft should be in. There will be other modules that will be usable in this mode, but this is the first one.

If an aircraft does not have nose steering, differential braking and differential thrust should be used, so the 'A' and 'D' keys, which normally steers the aircraft with the nosewheel (in addition to the rudder), will create diffenertial steering. Note: the thrust level of the steering engines is set by the throttle - the opposite engine should be idle.

### Radar-based Terrain Following Mode
This mode allows the aircraft to be fully manually operatable, however pilot-inputs are blended with terrain following, that way the aircraft can still bank and turn, but the autopilot also tries to keep the same radar altitude.

### Radar-based SLAM navigation (plotting course around terrain (horizontally or vertically))

# Bugs
- Thrust reverser should throttle zero before disengaging (and wait for spool down) at end of landing so that it doesn't start accelerating the plane fwd when reversers are disengaged

- [x] Performance-improvements commit appears to have introduced broad regressions.
- Moderate-G mode may deactivate unexpectedly (possibly during kOS director <-> cruise controller handoff).
- Aircraft does not bank correctly during approach.
- [x] Autothrottle response appears too slow (roughly 1 Hz behavior).
- Localizer intercept offset: aircraft is about 130 m right of centerline at intercept.
- On localizer capture/alive, guidance can remain in waypoint mode too long before proper ILS capture.
- Guidance mode toggling loop/spam observed:
  - kOS Director Enabled
  - Standard Fly-By-Wire Enabled
  - Standard Fly-By-Wire Disabled
  - kOS Director Enabled
- [x] During approach/cruise transitions, altitude control sometimes uses FPA hold when altitude-hold behavior is expected.
