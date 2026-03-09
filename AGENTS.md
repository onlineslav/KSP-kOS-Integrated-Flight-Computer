I’m building kOS code for a custom FAR-based SSTO spaceplane in Kerbal Space Program. I want you to help me write and iteratively refine two kOS files:

1. `x_10_d_boot.ks`
2. `X_10_D.ks`

## File roles

### `x_10_d_boot.ks`
This is the bootfile that is loaded onto the craft. Its job should be minimal and robust:
- initialize any needed settings
- print a short startup message
- run or compile `X_10_D.ks`
- hand control off cleanly to the main program
- avoid putting major flight logic here unless absolutely necessary

### `X_10_D.ks`
This is the main flight-control program. It should contain the actual flight logic, reusable functions, phase handling, and any state machine or mode switching.

## Overall goal

This craft is a custom SSTO spaceplane named X-10 D. It is intended to:
- take off from the runway like an aircraft
- perform a controlled atmospheric ascent
- transition through high-speed / high-altitude flight
- pitch into a proper ascent profile
- continue to apoapsis building
- coast if needed
- circularize into orbit

I do NOT want a toy example. I want a practical, structured kOS flight program architecture suitable for a real reusable spaceplane.

## Important design context

This is a spaceplane, not a vertical rocket, so the logic should reflect aircraft-like takeoff and atmospheric flight first, then orbital ascent later.

The program should be designed around multiple flight phases / programs, such as:

- preflight / initialization
- takeoff roll
- rotation
- initial climb
- low-altitude acceleration
- transonic / supersonic climb
- high-altitude acceleration
- pull-up / ascent to apoapsis target
- coast phase
- circularization burn
- optional orbit achieved / shutdown / hold attitude mode

I want the code designed so these are explicit modes or state transitions, not a pile of unstructured IF statements.

## What I want from you

Please help me generate clean, modular kOS code with:
- clear structure
- readable function and variable names
- comments that explain why things are being done
- tunable constants grouped near the top
- no unnecessary cleverness
- maintainability and debuggability prioritized over brevity

## Architecture requirements

Please structure `X_10_D.ks` around a clear state machine or phase manager.

For example, something in the spirit of:
- current flight phase variable
- one function per phase
- transition checks between phases
- shared helper functions for telemetry, steering, throttle logic, safety checks, etc.

I want a design that is easy to extend later with things like:
- abort modes
- re-entry
- landing autopilot
- ascent profile tuning
- engine mode switching
- flap / gear automation
- manual override

## Specific program behavior goals

### 1. Takeoff program
I want a proper runway takeoff routine that can:
- hold runway heading
- manage throttle / engine spool-up if relevant
- detect takeoff speed or rotation conditions
- rotate to a target pitch smoothly, not violently
- retract gear at an appropriate time after positive climb
- establish a stable climb

### 2. Atmospheric ascent program
After takeoff, I want the plane to:
- maintain sensible climb behavior
- avoid extreme AoA
- manage pitch gradually
- keep the craft stable through increasing speed
- optionally use q / dynamic pressure / altitude / speed thresholds for phase transitions

Because this is a spaceplane, I want the ascent logic to respect aerodynamic flight rather than immediately pitching like a rocket.

### 3. Orbital insertion / circularization
Eventually, I want the craft to:
- target a desired apoapsis
- stop active ascent when apoapsis target is achieved
- coast if needed
- calculate and perform a circularization burn
- end in stable orbit

## Coding style constraints

Please follow these constraints:
- write idiomatic kOS, not pseudocode
- avoid unsupported language features
- keep syntax compatible with kOS
- do not assume external libraries unless clearly stated
- prefer small helper functions over giant monolithic blocks
- use logging / PRINT output for debugging important transitions
- include safe guards for missing conditions or unexpected states where reasonable

## Output format I want from you

Please produce:
1. a brief architecture explanation
2. the full contents of `x_10_d_boot.ks`
3. the full contents of `X_10_D.ks`

## Additional implementation guidance

Please include or think about:
- tunable constants for takeoff speed, rotation pitch, climb pitch, apoapsis target, circularization margin, etc.
- helper functions for telemetry readouts
- phase transition logic based on altitude / speed / vertical speed / apoapsis / periapsis / time to apoapsis
- simple, stable steering behavior
- avoiding excessive oscillation
- comments marking places where craft-specific tuning will likely be needed

## Very important

If you are unsure about a kOS API name or syntax, do not invent random abstractions. Prefer simple, known kOS patterns.

Do not compress everything into a single loop without structure.

Do not write this as a generic rocket ascent autopilot. It must feel like an SSTO spaceplane ascent program.

Also, if some details are craft-specific and should be left as constants or TODOs, that is fine — but the overall structure should be complete and usable.