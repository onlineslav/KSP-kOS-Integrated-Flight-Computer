//-------------------------------------------------|
//                                                 |
//  kOS-Computer - by Sinucep - 2025  |
//                                                 |
//-------------------------------------------------|

@lazyGlobal off.

run once "/KOSmodore/terminal.ks".
run once "/KOSmodore/GPS.ks".
run once "/KOSmodore/rover.ks".
run once "/KOSmodore/splashes.ks".
run once "/KOSmodore/interface.ks".
run once "/KOSmodore/graphics.ks".
run once "/KOSmodore/math.ks".
run once "/KOSmodore/sampling.ks".
run once "/KOSmodore/pages.ks".
run once "/KOSmodore/datalog.ks".
run once "/KOSmodore/settings.ks".
run once "/KOSmodore/runKS.ks".
run once "/KOSmodore/texted.ks".
//run once "/KOSmodore/basicflow.ks".

global fine to false. // per uscire dal programma e usare il terminale

//Addon KOSProp Monitor
global monitors to addons:kpm:getmonitorcount().
global id to addons:kpm:getguidshort(0).      // OR set id to addons:kpm:getguid(0).
global monindex to addons:kpm:getindexof(id). // If GETINDEXOF Returns -1, GUID Not Found. Works for Whole GUID and Short GUID

local LogStart is 0.
local GPSLogStart is 0.
local PosStart is 0.                      // just for to blink once POS flag
local CuBlinkStart is 0.                   // for to blink the cursor

global SensLog TO LIST().
global LoadedTrack TO LIST().  
global endpos to LatLng(-90,8). 			  //destinazione provvissoria per il rover                   
global SENSELIST TO LIST().


LIST SENSORS IN SENSELIST.

InitTerminalMAIN(monitors).

function ExitOS {
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		ClearTerminal(x).
	}
	clearscreen.
	print "Exit main loop.".
	set fine to true.
}
	
function toggleFlag {
	parameter flagNum is 0.
    if flagNum = 0 {
		
		set PosStart to TIME:SECONDS.
		RecGPSPOS().
	}
	if flagNum = 1 {
		set GPSLogStart to TIME:SECONDS.
	}
	if flagNum = 2 {
		set LogStart to TIME:SECONDS.
	}
	
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO { 
		set myflags:currentmonitor to x. 
		
		if flagNum = 0 {
			myflags:setstate(0,TRUE).	
		}
		if flagNum = 1 {
			myflags:setstate(1,not (myflags:getstate(1))).
		}
		if flagNum = 2 {
			myflags:setstate(2,not (myflags:getstate(2))).	
		}
	}
}

function MyREBOOT {
	REBOOT.	
}

function goroverpos {

	if GPSPOS:Length = 0 {
		print "No destination set." at(0 + debugoffset,1).
	} else {	
	
	//clearScreen.
	brakes off.	
	set endPos to LatLng(GPSPOS[0], GPSPOS[1]).
	
	//solo per inizializzare qualcosa:
	set speedPID:setPoint to 1.   //speed
	set turnPID:setPoint to 0.     	//Direction
		
	global wanted_throttle to 0. // for now.
	lock wheelThrottle to wanted_throttle.
	global wanted_angle to 0.
	lock WHEELSTEERING to wanted_angle.
	
	set controlrover to 1.
	set roverstate to 1.
	} 	
}

function gorovertrack {
	
	if LoadedTrack:Length = 1 { //perché c'è lo header
		print "No track set." at(1,1).
	} else {	
		clearScreen.
		brakes off.	
		
		set speedPID:setPoint to 1.      //Speed
		set turnPID:setPoint to 0.      //Direction
			
		global wanted_throttle to 0. // for now.
		lock wheelThrottle to wanted_throttle.
		global wanted_angle to 0.
		lock WHEELSTEERING to wanted_angle.
	
		set roverstate to 1.
		set controlrover to 2.
	}
}
function DataLogAdd {
			SensLog:add(LIST()).
			SensLog[SensLog:LENGTH-1]:add(TIME:SECONDS).
			for ds in DataSourcesAdded {
				if ds = 1 {SensLog[SensLog:LENGTH-1]:add(ship:sensors:LIGHT).}
				if ds = 2 {SensLog[SensLog:LENGTH-1]:add(ship:sensors:TEMP).}
				if ds = 3 {SensLog[SensLog:LENGTH-1]:add(ship:sensors:PRES).}
				if ds = 4 {SensLog[SensLog:LENGTH-1]:add(ship:mass).}
				if ds = 10 {SensLog[SensLog:LENGTH-1]:add(ship:mass).}
				if ds = 20 {SensLog[SensLog:LENGTH-1]:add(ship:airspeed).}
				if ds = 21 {SensLog[SensLog:LENGTH-1]:add(ship:verticalspeed).}
				if ds = 22 {SensLog[SensLog:LENGTH-1]:add(ship:groundspeed).}
				if ds = 101 {SensLog[SensLog:LENGTH-1]:add(SHIP:GEOPOSITION:LAT).}
				if ds = 102 {SensLog[SensLog:LENGTH-1]:add(SHIP:GEOPOSITION:LNG).}
			}
			
			If Npage = 70 {
				GoPage(70).
			}
			If Npage = 72 {
				GoPage(72).
			}
}

//---------------------Initialize-----------------------

riempiSettings().
DataSourcesAdded:add(4). // default source: Vessel mass
InitSensLog().
InitGPSLog().
switch to 0. //con questa riga i file vengono salvati fisicamente (in archivio)
brakes on.   //for rovers..
SPage(0).

riempisourcelist().
initstextvar().
mainsplash().

SET LBook TO READJSON("/KOSmodore/logbook/logbook.json").
print "Logbook loaded from logbook.json".
set CurrentNote to LBook:LENGTH-1.

//flags
 myflags:setstate(0,FALSE).
  myflags:setstate(1,FALSE).
  myflags:setstate(2,FALSE).
  
//----------------------Starting page-------------------


//GoPage(8).
//GoPage(237).   //new basic
//GoPage(221).   //open basic
//set debugbas to true.

//----------------------Main Loop-----------------------

UNTIL fine {
// Pos flag label blink
if myflags:getstate(0) {
		if TIME:SECONDS > PosStart + 1{
			 MSetFlag(0,FALSE,monitors).
		}
	}

//LOGging
	if myflags:getstate(1) {
		if TIME:SECONDS > GPSLogStart {
			LoadedTrack:add(LIST()).
			LoadedTrack[LoadedTrack:LENGTH-1]:add(TIME:SECONDS).			
			LoadedTrack[LoadedTrack:LENGTH-1]:add(SHIP:GEOPOSITION:LAT).
			LoadedTrack[LoadedTrack:LENGTH-1]:add(SHIP:GEOPOSITION:LNG).
			
			If Npage = 30 {
				Gopage(30).
			}
			If Npage = 32 {
				GoPage(32).
			}
			
			set GPSLogStart to GPSLogStart + SettingsL[2]. //  track sampling time interval
		}
	}
	
	if myflags:getstate(2) {
		if TIME:SECONDS > LogStart {		
			DataLogAdd().
			
			set LogStart to LogStart + SettingsL[1]. // data sampling time interval
		}
	}

//rover
	if controlrover = 1 {
		conroverPos(endPos).
	}
	if controlrover = 2 {
		ConRovertrack(LoadedTrack).
	}
	
//blinking cursor
    if NPage = 237 or NPage = 238 or NPage = 239 or NPage = 240 or
	   NPage = 277 or NPage = 278 or NPage = 279 or NPage = 280 or
	   NPage = 243 or NPage = 244 or NPage = 245 or
	   NPage = 253 or NPage = 254 or NPage = 255 or	   
	   NPage = 52 or NPage = 53 or NPage = 54 or
	   NPage = 55 or NPage = 56 or
	   // file rename
	   NPage = 37 or NPage = 38 or NPage = 39 or
	   NPage = 57 or NPage = 58 or NPage = 59 or
	   NPage = 87 or NPage = 88 or NPage = 89 or
	   NPage = 187 or NPage = 188 or NPage = 189 or
	   NPage = 197 or NPage = 198 or NPage = 199 or
	   NPage = 227 or NPage = 228 or NPage = 229 or
	   
	   NPage = 33 or NPage = 34 or NPage = 35 or
	   NPage = 83 or NPage = 84 or NPage = 85 or
	   NPage = 101 or NPage = 102 or NPage = 103 or
	   NPage = 207 or NPage = 208 or NPage = 209{
		if TIME:SECONDS > CuBlinkStart + .45{
			set CuBlinkStart to TIME:SECONDS.
			set cuVisible to not cuvisible.
			if cuVisible {
				Cur().
			} else {
				Cudel().
			}
		}
	}


// BASIC

if (BASICrun) {RunBASICline().} 

// return to basic program (v1.1.1)

if backtobascase = 2 {
	if TIME:SECONDS > basWStart + basWtime {
		//set exodos to false.
		set backtobascase to 0.
		set BASICrun to true.
	}
}
		
	

	if backtobascase = 1 {		
		if controlrover = 0 {
			set backtobascase to 0.
			set BASICrun to true.
		}
	}

//debug window
	if debugbas and (Npage = 220 or Npage = 230 or NPage = 231) {
		if TIME:SECONDS > debugbasstart + .5{
			set debugbasstart to TIME:SECONDS.
			debugwin().
		}
	}
}
