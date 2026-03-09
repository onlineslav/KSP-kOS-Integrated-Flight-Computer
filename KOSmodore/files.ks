run once "/KOSmodore/GPS.ks".
run once "/KOSmodore/keybT9.ks".
run once "/KOSmodore/main.ks".
run once "/KOSmodore/runKS.ks".
run once "/KOSmodore/texted.ks".

function saveas {
	if (Npage = 52) or (Npage = 53) or (Npage = 54) {
		WRITEJSON(GPSPOS, "/KOSmodore/positions/" + EmptyProg[0] + ".pos").
		GoPage(50).
	}
	if (Npage = 33) or (Npage = 34) or (Npage = 35) {
		WRITEJSON(LoadedTrack, "/KOSmodore/tracks/" + EmptyProg[0] + ".trk").
		GoPage(30).
	}
	
	// file rename
	if (Npage = 37) or (Npage = 38) or (Npage = 39) {
		set INPstr1 to emptyprog[0].
		MovePATH("/KOSmodore/tracks/" + li[lind],
		"/KOSmodore/tracks/" + emptyprog[0] + ".trk").
		gopage(31).
	}
	if (Npage = 57) or (Npage = 58) or (Npage = 59) {
		set INPstr1 to emptyprog[0].
		MovePATH("/KOSmodore/positions/" + li[lind],
		"/KOSmodore/positions/" + emptyprog[0] + ".pos").
		gopage(51).
	}
	if (Npage = 87) or (Npage = 88) or (Npage = 89) {
		set INPstr1 to emptyprog[0].
		MovePATH("/KOSmodore/sampling/" + li[lind],
		"/KOSmodore/sampling/" + emptyprog[0] + ".dat").
		gopage(71).
	}
	if (Npage = 187) or (Npage = 188) or (Npage = 189) {
		set INPstr1 to emptyprog[0].
		MovePATH("/KOSmodore/kerboscript/" + li[lind],
		"/KOSmodore/kerboscript/" + emptyprog[0] + ".ks").
		gopage(201).
	}
	if (Npage = 197) or (Npage = 198) or (Npage = 199) {
		set INPstr1 to emptyprog[0].
		MovePATH("/KOSmodore/sks/" + li[lind],
		"/KOSmodore/sks/" + emptyprog[0] + ".sks").
		gopage(205).
	}
	if (Npage = 227) or (Npage = 228) or (Npage = 229) {
		set INPstr1 to emptyprog[0].
		MovePATH("/KOSmodore/ksp-basic/" + li[lind],
		"/KOSmodore/ksp-basic/" + emptyprog[0] + ".sbas").
		gopage(221).
	}
	
	
	if (Npage = 83) or (Npage = 84) or (Npage = 85) {
		WRITEJSON(SensLog, "/KOSmodore/sampling/" + EmptyProg[0] + ".dat").
		GoPage(70).
	}
	if (Npage = 207) or (Npage = 208) or (Npage = 209) {
		local fname to EmptyProg[0].
		set EmptyProg to STextAux:copy.
		STextAux:clear.
		if exists("/KOSmodore/kerboscript/" + fname + ".ks") {
		deletepath("/KOSmodore/kerboscript/" + fname + ".ks").
		}
		for lin in emptyprog {
			LOG lin to "/KOSmodore/kerboscript/" + fname + ".ks".
		}
		GoPage(218).
	}
	if (Npage = 243) or (Npage = 244) or (Npage = 245) {
		local fname to EmptyProg[0].
		set EmptyProg to STextAux:copy.
		STextAux:clear.
		WRITEJSON(EmptyProg, "/KOSmodore/ksp-basic/" + fname + ".sbas").
		GoPage(220).
	}
	if (Npage = 253) or (Npage = 254) or (Npage = 255) {
		local fname to EmptyProg[0].
		set EmptyProg to STextAux:copy.
		STextAux:clear.
		WRITEJSON(EmptyProg, "/KOSmodore/sks/" + fname + ".sks").
		GoPage(218).
	}
}

function delfilelist {
	if Npage = 51 {
		deletepath ("/KOSmodore/positions/" + li[lind]).
		gopage(51).
	}
	if Npage = 31 {
		deletepath ("/KOSmodore/tracks/"+ li[lind]).
		gopage(31).
	}
	if Npage = 71 {
		deletepath ("/KOSmodore/sampling/" + li[lind]).
		gopage(71).
	}
	if Npage = 201 {
		deletepath ("/KOSmodore/kerboscript/" + li[lind]).
		gopage(201).
	}
	if Npage = 205 {
		deletepath ("/KOSmodore/sks/" + li[lind]).
		gopage(205).
	}
	if Npage = 221 {
		deletepath ("/KOSmodore/ksp-basic/" + li[lind]).
		gopage(221).
	}
}

function Mkcopyfilelist {
	if Npage = 51 {
		COPYPATH("/KOSmodore/positions/" + li[lind],
		"/KOSmodore/positions/c-" + li[lind]).
		gopage(51).
	}
	if Npage = 31 {
		COPYPATH("/KOSmodore/tracks/" + li[lind],
		"/KOSmodore/tracks/c-" + li[lind]).
		gopage(31).
	}
	if Npage = 71 {
		COPYPATH("/KOSmodore/sampling/" + li[lind],
		"/KOSmodore/sampling/c-" + li[lind]).
		gopage(71).
	}
	if Npage = 201 {
		COPYPATH("/KOSmodore/kerboscript/" + li[lind],
		"/KOSmodore/kerboscript/c-" + li[lind]).
		gopage(201).
	}
	if Npage = 205 {
		COPYPATH("/KOSmodore/sks/" + li[lind],
		"/KOSmodore/sks/c-" + li[lind]).
		gopage(205).
	}
	if Npage = 221 {
		COPYPATH("/KOSmodore/ksp-basic/" + li[lind],
		"/KOSmodore/ksp-basic/c-" + li[lind]).
		gopage(221).
	}
}

function Renfilelist {
	if Npage = 31 {
		//set emptyprog[0] to "".
		TypeString(37,"New file name: ",0, true).
	}
	if Npage = 51 {
		//set emptyprog[0] to "".
		TypeString(57,"New file name: ",0, true).
	}
	if Npage = 71 {
		//set emptyprog[0] to "".
		TypeString(87,"New file name: ",0, true).
	}
	if Npage = 201 {
		//set emptyprog[0] to "".
		TypeString(187,"New file name: ",0, true).
	}
	if Npage = 205 {
		//set emptyprog[0] to "".
		TypeString(197,"New file name: ",0, true).
	}
	if Npage = 221 {
		//set emptyprog[0] to "".
		TypeString(227,"New file name: ",0, true).
	}
	
}

function Savefilelist {
	parameter mult.
	if Npage = 50 {
		if GPSPOS:length = 0 {
			print "No destination set. Can't save." at(0,0).
		}
		else {
			TypeString(52,"File name: ",0, true).
		}
	}
	if Npage = 30 {
		if LoadedTrack:LENGTH < 2 {
			print "No track set. Can't save." at(0,0).
		}
		else {
			TypeString(33,"File name: ",0, true).
		}
	}
	if Npage = 70 {
		if SensLog:LENGTH < 2 {
			 print "No data log set. Can't save." at(0,0).
		}
		else {
			TypeString(83,"File name: ",0, true).
		}
	}
	if Npage = 218 and mult = 0 {TypeString(253,"File name: ",0, true).}
	if Npage = 218 and mult = 1 {TypeString(207,"File name: ",0, true).}
	if Npage = 220 {TypeString(243,"File name: ",0, true).}
}
// SAVE SENSORS LOG
//function button08Press {
//clearscreen.
//WRITEJSON(SensLog, "/KOSmodore/senslog.json").
//
//DELETEPATH("/KOSmodore/senslog.txt").
//FOR S IN SensLog {
//		FOR SS IN S {
//			LOG SS to "/KOSmodore/senslog.txt".
//		}		
//		LOG "" to "/KOSmodore/senslog.txt".
//	}
//
//print "Saved to " + PATH() + "/KOSmodore/senslog.json".
//print "Saved to " + PATH() + "/KOSmodore/senslog.txt".
//
//}