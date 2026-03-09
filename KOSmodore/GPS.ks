global GPSPOS TO LIST().//////////////////
global Destin to LATLNG(0, 0).////////////

global GEOPOS to LATLNG(10, 20). 

function DRAWTRK {
	GTrack(LoadedTrack).
}

function AddHereToTrack{
	LoadedTrack:add(LIST()).
	LoadedTrack[LoadedTrack:LENGTH-1]:add(TIME:SECONDS).			
	LoadedTrack[LoadedTrack:LENGTH-1]:add(SHIP:GEOPOSITION:LAT).
	LoadedTrack[LoadedTrack:LENGTH-1]:add(SHIP:GEOPOSITION:LNG).
	Gopage(30).
}

function RemoveLastFromTrack{
	if LoadedTrack:LENGTH > 1{
		LoadedTrack:Remove(LoadedTrack:LENGTH-1).	
	}
	Gopage(30).
}

function RemoveFirstFromTrack{
	if LoadedTrack:LENGTH > 1{
		LoadedTrack:Remove(1).
	}
	Gopage(30).
}

function InitGPSLog {	
		LoadedTrack:add(LIST()).
	    LoadedTrack[LoadedTrack:LENGTH-1]:add(0).   //Time
		LoadedTrack[LoadedTrack:LENGTH-1]:add(101). //latitude
		LoadedTrack[LoadedTrack:LENGTH-1]:add(102). //longitude 
}


// SAVE GPS POS
function SaveGPSPOS {
	clearscreen.
	WRITEJSON(GPSPOS, "/KOSmodore/GPSpos.json").
	print "Saved to " + PATH() + "/KOSmodore/GPSpos.json".
}

// save POS to random file
function SaveDestRND {
GoPage(50).
	
	//if EXISTS("/KOSmodore/positions/+fname")
		
	if GPSPOS:length = 0 {
		 print "No destination set. Can't save." at(0,0).
	} else {
		local fname to "".
		FROM {local x is 8.} UNTIL x = 0 STEP {set x to x-1.} DO {
			set fname to fname + (floor(10*RANDOM())).
		}
		set fname to fname + ".pos".

		UNTIL not EXISTS("/KOSmodore/positions/" + fname) {
			set fname to "".
			FROM {local x is 8.} UNTIL x = 0 STEP {set x to x-1.} DO {
				set fname to fname + (floor(10*RANDOM())).
			}
			set fname to fname + ".pos".
		}
	

		WRITEJSON(GPSPOS, "/KOSmodore/positions/" + fname).

		print "----------------------------------------" at(0,14). //LUNGO GIUSTO
		print "Saved to:" at(1,15).
		//print PATH() + "/KOSmodore/positions/" + 
		print fname at (1,16).
	}
}


function ClrGPSPOS {
	GPSPOS:clear.
	if NPage = 50 {GoPage(50).}
}

function RecGPSPOS {
	GPSPOS:clear.
	GPSPOS:add(SHIP:GEOPOSITION:LAT).
	GPSPOS:add(SHIP:GEOPOSITION:LNG).
	set GeoPos to SHIP:GEOPOSITION.
	if NPage = 50 {GoPage(50).}
}

// Clear GPS LOG
function ClearGPSLOG {
	LoadedTrack:clear.
	InitGPSLog().
	Gopage(30).
}

