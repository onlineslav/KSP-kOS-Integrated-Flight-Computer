print "Basics #28".
wait 3.
sas off.

local targvel is 0.

lock steering to up.

set throttlePID to PIDLoop(1.02, 10.3, 0.03, 0, 1).
set throttlePID:setpoint to targvel.
set wanted_throttle to 1.
lock throttle to wanted_throttle.

stage.
//set throttlePID:SETPOINT to -2.
wait until alt:radar >= 10.
set now to time:seconds.

until time:seconds >= now + 15 {
  set wanted_throttle to throttlePID:UPDATE(time:seconds, ship:verticalspeed).
  print "  PID Throttle : " + round(throttle,2) + "   " at (0,6).
  print "vertical speed : " + round(ship:verticalspeed,2) + " m/s      " at (0,7).
  print "     alt:radar : " + round(alt:radar,2) + " m      " at (0,8).
}

set throttlePID:SETPOINT to -2.
until ship:status = "landed" {
  set wanted_throttle to throttlePID:UPDATE(time:seconds, ship:verticalspeed).
  print "  PID Throttle : " + round(throttle,2) + "   " at (0,6).
  print "vertical speed : " + round(ship:verticalspeed,2) + " m/s      " at (0,7).
  print "     alt:radar : " + round(alt:radar,2) + " m      " at (0,8).
}

wait 0.
sas on.
set ship:control:pilotmainthrottle to 0.
unlock throttle.