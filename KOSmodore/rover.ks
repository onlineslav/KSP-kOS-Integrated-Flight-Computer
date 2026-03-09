

//Rover Things

//run once "/KOSmodore/main.ks".
global speedPID to PIDLoop(0.4, 0, 0, 0, 1).
global turnPID TO PIDLOOP(0.3,0,0.1,-1,1).

global controlrover to 0.      // 0 = nessun rover da conrollare
                               // 1 = rover da mandare ad un posto specifico
							   // 2 = rover segue track

// stati del rover che va verso una posizione

global roverstate to 0.        // 0 = stato di nulla
							   // 1 = stato di puntamento iniziale lento
							   // 2 = dritto per due metri per raddrizzare le ruote
							   // 3 = veloce fino a 30 metri dall'arrivo
							   // 4 = frenata in caso di sbandamento che riconduce a 1
							   // 5 = piano gli ultimi metri
							   
local twompos to LatLng(0,0).  //posizione ausiliaria per andare dritto due metri e raddrizzare le ruote prima di aumentare la velocità

global TCou to 1.              //indice che scorre il  track. parte da 1 perchè 0 è lo header

function brakes_on {
	//print "BON".
	brakes on.
}

function brakes_off {
	//print "BOFF".
	brakes off.
}

function conroverstate
{
	parameter f.

	if f = 0 {
		//Speed
		global speedPID to PIDLoop(6, 0, 0, 0, 1).
		set speedPID:setPoint to 0.
	
		//Direction
		global turnPID TO PIDLOOP(0.3,0,0,-1,1).
		set turnPID:setPoint to 0.
	}
	
	if f = 1 {
		//Speed
		global speedPID to PIDLoop(6, 0, 0, 0, 1).
		set speedPID:setPoint to 1.
	
		//Direction
		global turnPID TO PIDLOOP(0.3,0,0,-1,1).
		set turnPID:setPoint to 0.
	}

	if f = 2 {		
		// Speed
		global speedPID to PIDLoop(0.4, 0, 0, 0, 1).
		set speedPID:setPoint to 4.
	
		// Direction	
		global turnPID TO PIDLOOP(0.01,0,0,-.8,.8).
		set turnPID:setPoint to 0.
	}

	if f = 3 {		
		// Speed
		global speedPID to PIDLoop(0.4, 0, 0, 0, 1).
		set speedPID:setPoint to 15.
	
		// Direction	
		global turnPID TO PIDLOOP(0.01,0,0,-.8,.8).
		set turnPID:setPoint to 0.
	}
}



function ConRoverPos {
	
	parameter LPos.     // loaded Pos
	
    //necessita in partenza roverstate = 1
	set wanted_throttle to SpeedPID:UPDATE(time:seconds,ship:velocity:surface:mag).
	set wanted_angle to TurnPID:UPDATE(TIME:SECONDS,LPos:bearing).
	set ship:control:wheelsteer to wanted_angle.
	
	if roverstate = 1 {
		conroverstate(1).
		if LPos:distance < 5 {
			set controlrover to 0.
		}
		if abs(LPos:bearing) < 8{
			set roverstate to 2.
			set twompos to LatLng(ship:geoposition:lat, ship:geoposition:lng).
			
		}
	}
	
	if roverstate = 2 {
		conroverstate(1).
		if abs(LPos:bearing) > 20{
			set roverstate to 4.
		}
		if twomPos:distance > 2 {
			set roverstate to 3.
		}
		if LPos:distance < 5 {
			set controlrover to 0.
		}
	}
	
	if roverstate = 3 {
		conroverstate(3).
		if abs(LPos:bearing) > 20{
			set roverstate to 4.
		}
		if LPos:distance < 40 {
			conroverstate(2).
			brakes on.
		
			if ship:velocity:surface:mag < 4 {
				brakes off.
				set roverstate to 5.
			}
		}
	}
	
	if roverstate = 4{
		conroverstate(0).
		brakes on.
		
		if ship:velocity:surface:mag < 1 {
			brakes off.
			set roverstate to 1.
		}
	}
	
	if roverstate = 5{
		conroverstate(2).
		if abs(LPos:bearing) > 20{
			set roverstate to 4.
		}
		if LPos:distance < 5 {
			conroverstate(0).
			brakes on.
		
			if ship:velocity:surface:mag < 1 {
				brakes off.
				set controlrover to 0.
			}
			
		}
	}
	
	if controlrover = 0 {
		brakes on.
		unlock wheelThrottle.
		set ship:control:wheelsteer to 0.
		unlock WHEELSTEERING.
	}

	if Npage = 20 {
		FROM {               //porta il cursore dove non lo vede nessuno
			local xx is 1.
		}
		UNTIL xx = 18 STEP {
			set xx to xx+1.
		} 
		DO {
			print " ".
		}
		print "Distance to target: " + round(LPos:distance, 2) + (" m   ") at (0,0).
		print "Bearing to target: " + round(LPos:bearing, 2) + ("°   ") at (0,1).
		print "State: " + roverstate at (0,2).	
		print "Speed: " + ship:velocity:surface:mag at (0,3).
		
	}
}

function ConRoverTrack {
	parameter LT.      //LoadedTrack
	 
	//print  "l: " + LT:LENGTH + ", TCou = " + Tcou at (0,5).
	//print Tcou + "° pos: " + LT[TCou][1] +", " + LT[TCou][2] at (0,6).
	
	//set endpos to LatLng(LT[TCou][1], LT[TCou][2]).
	//clearscreen.
	
	
	wait 0.1.
	//set TCou to TCou + 1.
	if TCou <  LT:LENGTH {
		//print "Tcou: " + TCou at(0,5).
		conroverPos(LatLng(LT[TCou][1], LT[TCou][2])).
		if controlrover = 0 {    // lo header non mi interessa (-1)
			set controlrover to 2.
			set TCou to TCou + 1.
			
			set speedPID:setPoint to 1.
			set turnPID:setPoint to 0.
		
			global wanted_throttle to 0. // for now.
			lock wheelThrottle to wanted_throttle.
			global wanted_angle to 0.
			lock WHEELSTEERING to wanted_angle.
		}
	}
	else {
		set controlrover to 0.
		set TCou to 1.
		brakes on.
		unlock wheelThrottle.
		set ship:control:wheelsteer to 0.
		unlock WHEELSTEERING.
	}
}



