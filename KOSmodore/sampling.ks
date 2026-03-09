@lazyGlobal off.

// Clear sample LOG
function ClearSensLOG {
	SensLOG:clear.
	InitSensLog().
	if	npage = 70 {
		GoPage(70).
	}
}




// Sensor reading for print (era in main, mai testato fuori)
function button02Press {
	LOCAL X IS 0.
	clearscreen.
	print "Reading time: " + round(TIME:SECONDS,3) at (0,1).
	print SHIP:body at (0,2).
	
	//local bodynome to SHIP:body.
	//print "|" + bodynome + "|" + SHIP:body + "|".
	//print "Temp. model [ K ]: " + BODYATMOSPHERE("Kerbin"):ALTTEMP(10) at (0,2).
	
	//scanning sensor readings by senselist
	
	// ti da la somma delle esposizioni di tutti i pannelli solari.
	//Se nn ce ne sono ti da 0. Quindi l'unità di misura sono tipo i Watt prodotti
	
	
	    // solar panels
		print "Light exp: " + round(ship:sensors:LIGHT,4) at (0,5).
	
		
	
	FOR S IN SENSELIST {
		IF S:ACTIVE {
		} ELSE {
			S:TOGGLE().
		}
		if S:TYPE = "TEMP" {
		print "Temp [ K ]: " + round(ship:sensors:TEMP,4) at (0,6+X). // S:DISPLAY 
		}
		
		if S:TYPE = "PRES" {
		print "Pres [ kPa ]: " + round(ship:sensors:PRES,4) at (0,6+X). // S:DISPLAY
		}
		
		//GRAV and ACC are vectors		
		//if S:TYPE = "GRAV" {
		//print "Grav = " + ship:sensors:GRAV + S:DISPLAY at (0,4+X).
		//}
		
		//if S:TYPE = "ACC" {
		//print "Acc = " + ship:sensors:ACC + S:DISPLAY at (0,4+X).
		//}
		
		//PRINT S:TYPE + ": " + S:DISPLAY at (0,3+X).
	set X to X+1.
	}
	
    
	//print ship:sensors:temp at (0,8).
}

//old view sampling
function button03Press {
	clearscreen.
	local x to 0.
	local y to 0.
	FOR S IN SensLog {
		FOR SS IN S {
			print round(SS,2) at (X,Y).
			set x to x+9.
		}		
		set y to y+1.
		set x to 0.
	}
}