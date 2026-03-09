function SRCOOold { //ship-raw to body coordinates
	parameter vec.
	local SOIvec to vec + SHIP:BODY:POSITION.
	local spot TO LATLNG(10, 20). 
	return spot.
}

function SRCOO {    // geo coordinates to ship-raw
	parameter coo,  // coordinate
	          h.    // altezza dal suolo
	
	return geopos:ALTITUDEPOSITION(geopos:TERRAINHEIGHT+h).
    //return geopos:POSITION - geopos:ALTITUDEPOSITION(geopos:TERRAINHEIGHT+3),
            

PRINT ALT:RADAR.
}