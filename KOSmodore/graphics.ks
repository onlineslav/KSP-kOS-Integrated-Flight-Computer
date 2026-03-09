run once "/KOSmodore/settings.ks".

local visualtrack to list(). 


function gvec {
	//PARAMETER targetOrbitable,vColor, vLabel IS "default".
	parameter a,b,c,co.
	return VECDRAW(
		v(0,0,0),
		v(a,b,c),
		co,
		"w",
		1,
		true,
		SettingsL[0], //marker thikness
		true,
		false
	).
}

// piccolo esempio per illustrare come gestire singoli vettori disegnati in una lista
function draw {
   local vecs to list().
   vecs:add(gvec(-2,4,6,yellow)).
   wait 3.
   set vecs[0]:show to false.
}

function HideTRK {
	clearvecdraws().
}

// visualizza un segmento verticale sulla superficie
// in corrispondenza del puntop geografico
function GSpot {
	parameter gp, lbl.
return VECDRAW(
			{return gp:ALTITUDEPOSITION(gp:TERRAINHEIGHT+3).},
			{return gp:POSITION - gp:ALTITUDEPOSITION(gp:TERRAINHEIGHT+3).},
			{return red.},
			lbl, 1, true, SettingsL[0], false).
}

function GDestination {
	clearvecdraws().
	GSpot(GeoPos,"Destination").
}

function GTrack {
	parameter LT.  
	local gp to LATLNG(10, 20). //geo pos
	local cou to 0.	
	clearvecdraws().
	
	//print  "l: " + LT:LENGTH + ", TCou = " + Tcou at (0,5).
	
	for po in LT {
	
		//print Tcou + "° pos: " + po[1] +", " + po[2] at (0,6).
		set gp to LatLng(po[1], po[2]).
	
		if cou > 0 {
			//wait 1.
			//print cou + ", " + gp:lat  + ", " + gp:lng .
			visualtrack:add(V(0,0,0)).
			set visualtrack[cou-1] TO GSpot(gp, cou + "").
			
		}
		set cou to cou + 1.
	}
}	