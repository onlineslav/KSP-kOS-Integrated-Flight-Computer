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

# Planned

- Build FMS/IFC as a central control system for both:
  - Terrestrial flights
  - Spaceplane flights to LEO
- Expand flight plan leg types:
  - Takeoff
  - Navigate to marker/waypoint/coordinates
  - Spaceplane suborbital insertion (user-defined apoapsis and tunables)
  - Spaceplane re-entry program (guidance to waypoint/marker with controlled lift/drag)
  - Approach
  - Land
- Support full plan lifecycle:
  - Save
  - Load
  - Edit
  - Delete
  - Reorder

# Bugs

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
