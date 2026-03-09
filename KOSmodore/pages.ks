run once "/KOSmodore/logbook.ks".
run once "/KOSmodore/keybT9.ks".
run once "/KOSmodore/interface.ks".
run once "/KOSmodore/datalog.ks".
run once "/KOSmodore/texted.ks".
run once "/KOSmodore/basicflow.ks".

function GoPage {
	parameter p.
	
	set NPage to p.
	
	// Main
	if p = 1 {                     
		SPage(1).
		InitTerminalMAIN(monitors).
	}
	
	// Test
	if p = 8 {                     
		SPage(8).
		InitTermTEST(monitors).
		clearscreen.
	}
	
	// Rover
	if p = 20 {
		SPage(20).
		InitTerminalROVER(monitors).
	}
	
	// Tracks
	if p = 30 {
		SPage(30).
		InitTerminalGPS(monitors).
		if LoadedTrack:length = 1{
			print "No track set." at(0,0).
		} else {
			print "Track of " + (LoadedTrack:length-1) + " spots set." at(0,0).
		}
	}
	
	// Select track file
	if p = 31 {
		SPage(31).	
		set lind to 0.
		InitTermFileLoad(monitors).
		cd("KOSmodore").
		cd("tracks").
		LIST FILES IN li.
		cd("..").
		cd("..").
		listafile(li).
	}
	
	// View track
	if p = 32 {
		SPageTile(32,"Time          Latitude      Longitude").
		local x to 0.
		local y to 0.
		InitTerminalViewTrack(monitors).
		FOR S IN LoadedTrack {
			if y>0 {
				FOR SS IN S {
					if x = 0 {
						print round(SS,1) at (0,Y).
					} else {
						print round(SS,7) at (X*14,Y).
					}
					set x to x+1.
				}		
			}
			set y to y+1.
			set x to 0.
		}	
	}
		
	// T9 save track pages
	if p = 33 {InitTermFileSave(monitors).}
	if p = 34 {InitTermFileSave1(monitors).}
	if p = 35 {InitTermFileSave2(monitors).}
	
// rename track
	if p = 37 {
		SPageNoClear(37).	
		InitTermFileSave(monitors).
	}
	if p = 38 {
		SPageNoClear(38).	
		InitTermFileSave1(monitors).
	}
	if p = 39 {
		SPageNoClear(39).	
		InitTermFileSave2(monitors).
	}
	
	// Logbook
	//if p = 40 {
//		SPage(40).
		//InitTermLogBook(monitors).	
		//local cou to 0.
		//for lin in LBook[CurrentNote] {
//			print lin at(0,cou).
			//set cou to cou + 1.
		//}
	//}
	
	// Logbook write pages
	//if p = 41 {
		//local cou to 0.
		//resetcicleset().
		//SPage(41).
		//InitTermLogBook1(monitors).	
		//for lin in LBook[CurrentNote] {
//			print lin at(0,cou).
			//set cou to cou + 1.
		//}
		//setxy(0,LBook[CurrentNote]:LENGTH).
		//print "_" at(0,LBook[CurrentNote]:LENGTH).
		////2 righe magiche che editano l'ultima riga del diario:
		////print "_"at(LBook[LBook:LENGTH-1]:LENGTH+1,LBook:LENGTH).
		////set linea to LBook[LBook:LENGTH-1][LBook[LBook:LENGTH-1]:LENGTH-1].
		//print linea + "_" at(0,cou).
	//}
	
	//if p = 42 {
//		local cou to 0.
		//SPage(42).
		//InitTermLogBook2(monitors).
		//for lin in LBook[CurrentNote] {
//			print lin at(0,cou).
			//set cou to cou + 1.
		//}
		//setxy(0,LBook[CurrentNote]:LENGTH).
		//print "_"at(0,LBook[CurrentNote]:LENGTH).
		//print linea + "_" at(0,cou).
	//}
	
//	if p = 43 {
		//local cou to 0.
		//SPage(43).
		//InitTermLogBook3(monitors).
		//for lin in LBook[CurrentNote] {
//			print lin at(0,cou).
			//set cou to cou + 1.
		//}
		//setxy(0,LBook[CurrentNote]:LENGTH).
		//print "_"at(0,LBook[CurrentNote]:LENGTH).
		//print linea + "_" at(0,cou).
	//}
	
	//Destinations
	if p = 50 {
		SPage(50).
		InitTerminalDestination(monitors).
		if GPSPOS:length = 0{
			print "No destination set." at(0,0).
		} else {
			print "Destination set:" at(0,0).
			print round(GPSPOS[0],4) + ", " + round(GPSPOS[1],4) at(0,1).
		}
	}

	if p = 51 {
		SPage(51).	
		set lind to 0.
		InitTermFileLoad(monitors).
		cd("KOSmodore").
		cd("positions").
		LIST FILES IN li.
		cd("..").
		cd("..").
		listafile(li).
	}		

	// save destination
	if p = 52 {InitTermFileSave(monitors).}
	if p = 53 {InitTermFileSave1(monitors).}
	if p = 54 {InitTermFileSave2(monitors).}
	
	// rename destination
	if p = 57 {InitTermFileSave(monitors).}
	if p = 58 {InitTermFileSave1(monitors).}
	if p = 59 {InitTermFileSave2(monitors).}
	
	// Datalog
	if p = 70 {
		SPage(70).
		InitTerminalSampler(monitors).	
		if SensLog:Length = 1 { //perché c'è lo header
			print "No data log set." at(0,0).
		} else {	
			print "LOG of " + (SensLog:length-1) + " samples set." at(0,0).
		}
		print "SOURCES:" at(0,1).
		ShowAddedDSources(2).
	}
	
    // Select datalog file
	if p = 71 {
		SPage(71).	
		set lind to 0.
		InitTermFileLoad(monitors).
		cd("KOSmodore").
		cd("sampling").
		LIST FILES IN li.
		cd("..").
		cd("..").
		listafile(li).
	}
	
	// Datalog view
	if p = 72 {
		// per capire quali sorgenti ci sono
		if SensLog[0]:LENGTH > 1 {
			set selcolumn to 1.
			set maxcolumn to SensLog[0]:LENGTH - 1.
		}
		ViewDataLogSource(selcolumn).
		InitTerminalViewDataLog(monitors).
	}
	
	// ask before changing source list and clean datalog
	if p = 74 {
		SPage(74).
		print "Changing sources will result in the" at(0,0).
		print "loss of the current data log." at(0,1).
		print "If you need it, save it first." at(0,2).
		print "Do you want to proceed clearing" at(0,3).
		print "the current log?" at(0,4).
		InitTermProceedToSource(monitors).
	}
	
	// Select data source
	if p = 75 {
		SPage(75).
		ClearSensLOG(). //clean the data log.
		InitTermShowDataSources(monitors).
		ShowAddedDSources(0).
	}
	
	// Add data source from list
	if p = 76	{
		SPage(76).	
		set lind to 0.
		InitTermSelectDataSourcesAdd(monitors).
		ListaStr(DSnames(DataSources)).
	}
	
	// Delete data source from list
	if p = 77 {
		SPage(77).	
		set lind to 0.
		InitTermSelectDataSourcesSub(monitors).
		ListaStr(DSnames(DataSourcesAdded)).
	}
	
	// save data log
	if p = 83 {InitTermFileSave(monitors).}
	if p = 84 {InitTermFileSave1(monitors).}
	if p = 85 {InitTermFileSave2(monitors).}
	
	// rename datalog
	if p = 87 {InitTermFileSave(monitors).}
	if p = 88 {InitTermFileSave1(monitors).}
	if p = 89 {InitTermFileSave2(monitors).}
	
	// Settings
	if p = 100 {
		SPage(100).	
		InitTermSettings(monitors).
		print "Marker thickness: " + SettingsL[0] at(0,0).
		print "Data log sampling interval (s): " + SettingsL[1] at(0,1).
		print "Track sampling interval (s): " + SettingsL[2] at(0,2).
	}
	
	// rename ks
	if p = 187 {
		SPageNoClear(187).	
		InitTermFileSave(monitors).
	}
	if p = 188 {
		SPageNoClear(188).	
		InitTermFileSave1(monitors).
	}
	if p = 189 {
		SPageNoClear(189).	
		InitTermFileSave2(monitors).
	}
	// rename sks
	if p = 197 {
		SPageNoClear(197).	
		InitTermFileSave(monitors).
	}
	if p = 198 {
		SPageNoClear(198).	
		InitTermFileSave1(monitors).
	}
	if p = 199 {
		SPageNoClear(199).	
		InitTermFileSave2(monitors).
	}
	
	// Programs
	if p = 200 {
		SPage(200).	
		InitTermPrograms(monitors).
	}
	
	// select and load kerboscript from file
	if p = 201 {
		SPage(201).	
		set lind to 0.
		InitTermFileLoad(monitors).
		cd("KOSmodore").
		cd("kerboscript").
		LIST FILES IN li.
		cd("..").
		cd("..").
		listafile(li).
	}
	
	// select and load serializedkerboscript from file
	if p = 205 {
		SPage(205).	
		set lind to 0.
		InitTermFileLoad(monitors).
		cd("KOSmodore").
		cd("sks").
		LIST FILES IN li.
		cd("..").
		cd("..").
		listafile(li).
	}
	
	// export sks to ks file
	if p = 207 {InitTermFileSave(monitors).}
	if p = 208 {InitTermFileSave1(monitors).}
	if p = 209 {InitTermFileSave2(monitors).}
	
	// Running kerboscript program page
	if p = 210 {
		SPageNoCLear(210).	
		ClearTerminal(monitors).
	}
	
	// Ended program page
	if p = 211 {
		SPageNoCLear(211).	
		InitTermEndedProgram(monitors).
	}
	
	// View sks prog
	if p = 218 {
		SPage(218).
		InitTermViewSKS(monitors).	
		CuReset().
		local cou to 0.
		for lin in emptyprog {
			print lin at(0,cou).
			set cou to cou + 1.
		}
	}
	
	// View basic prog
	if p = 220 {
		SPage(220).
		InitTermViewBas(monitors).	
		CuReset().
		local cou to 0.
		
		if debugbas {debugwinframe().}
		
		for lin in emptyprog {
			print lin at(0 + debugoffset, cou).
			set cou to cou + 1.
		}
	}
	
	// Select basic program file
	if p = 221 {
		SPage(221).
		set lind to 0.
		InitTermFileLoad(monitors).
		cd("KOSmodore").
		cd("ksp-basic").
		LIST FILES IN li.
		cd("..").
		cd("..").
		listafile(li).
		set lind to 0.
	}
	
	// rename sbas
	if p = 227 {InitTermFileSave(monitors).}
	if p = 228 {InitTermFileSave1(monitors).}
	if p = 229 {InitTermFileSave2(monitors).}
	
// Running basic program page
//	if p = 230 {
//		SPage(230).	
//		InitTermExeBas(monitors).
//		//ClsNoCur().
//		//reclinumbers(emptyprog).
//		runbasic(emptyprog).
//		//reclinumbers(emptyprog).
//		//removelinumber("10 cane").		
//	}
	
// Running basic program 
	if p = 230 {
		set BASICrun to true.
		set stepmode to false.
		if co = -1 {SPage(230).}
		if debugbas {debugwinframe().}
		//if not stepmode {SPage(230).}
		
		InitTermExeBas(monitors).		
		//print time:seconds at(40,15).
	}
	
// Step running basic program
	if p = 231 {
		set BASICrun to true.
		//print "time:second" at(10,8).
		//SPagenoclear(231).
		//if not stepmode {SPage(230).}
		
		InitTermstepExeBas(monitors).		
		//print time:seconds at(40,15).
	}

// Edit basic prog 0 con cursore
	if p = 237 {
		SPage(237).
		InitTermEditBas0cu(monitors).	
		
		local cou to 0.
		for lin in emptyprog {
			print lin at(0,cou).
			set cou to cou + 1.
		}
		set li to emptyprog.
	}
	
	// Edit basic prog 1 con cursore
	if p = 238 {
		SPage(238).
		InitTermEditBas1cu(monitors).	
		local cou to 0.
		for lin in emptyprog {
			print lin at(0,cou).
			set cou to cou + 1.
		}
		set li to emptyprog.
	}
	
	// Edit basic prog 2 con cursore
	if p = 239 {
		SPage(239).
		InitTermEditBas2cu(monitors).	
		local cou to 0.
		for lin in emptyprog {
			print lin at(0,cou).
			set cou to cou + 1.
		}
		set li to emptyprog.		
	}
	
	// Edit basic prog 3 con cursore
	if p = 240 {
		SPage(240).
		(monitors).
		InitTermEditBas3cu(monitors).	
		local cou to 0.
		for lin in emptyprog {
			print lin at(0,cou).
			set cou to cou + 1.
		}
		set li to emptyprog.
	}
	
	// T9 save BAS pages
	if p = 243 {InitTermFileSave(monitors).}
	if p = 244 {InitTermFileSave1(monitors).}
	if p = 245 {InitTermFileSave2(monitors).}
	
	// T9 save ks pages
	if p = 253 {InitTermFileSave(monitors).}
	if p = 254 {InitTermFileSave1(monitors).}
	if p = 255 {InitTermFileSave2(monitors).}
	
	// Edit sks prog 0 con cursore
	if p = 277 {
		SPage(277).
		InitTermEditBas0cu(monitors).	
		local cou to 0.
		for lin in emptyprog {
			print lin at(0,cou).
			set cou to cou + 1.
		}
		set li to emptyprog.
		//Cur().
	}
	
	// Edit sks prog 1 con cursore
	if p = 278 {
		SPage(278).
		InitTermEditBas1cu(monitors).	
		local cou to 0.
		for lin in emptyprog {
			print lin at(0,cou).
			set cou to cou + 1.
		}
		set li to emptyprog.
		//Cur().
	}
	
	// Edit sks prog 2 con cursore
	if p = 279 {
		SPage(279).
		InitTermEditBas2cu(monitors).	
		local cou to 0.
		for lin in emptyprog {
			print lin at(0,cou).
			set cou to cou + 1.
		}
		set li to emptyprog.
		//Cur().
	}
	
	// Edit sks prog 3 con cursore
	if p = 280 {
		SPage(280).
		InitTermEditKS3cu(monitors).	
		local cou to 0.
		for lin in emptyprog {
			print lin at(0,cou).
			set cou to cou + 1.
		}
		set li to emptyprog.
		//Cur().
	}
	
}

// INF. HERE
function InfHerePAGE {
	SPage(999).
	Print "Latitude: " + round(SHIP:GEOPOSITION:LAT,3) + ", Longitude: "
		+ round(SHIP:GEOPOSITION:LNG,3) at (0,3). 
	print "Ground distance: " + round(SHIP:GEOPOSITION:distance,3) at (0,4).
}

function TypeRealNum {                       //type latitude
		//Types: 1=real, 2=string
		parameter page, label, y, cls. 
		if cls {
			SPage(page).
		} else {
			SPageNoClear(page).
		}
		resetcicleset().
		InitTermTypeRealNum(monitors).
		set emptyprog[0] to "". 
		set cuy to 0.
		set cux to 0.
		Print label at(0,y).	
		set offsy to y+1.
		setxy(0,y+1).
}

 //you can incorporate thi funtion to the first page like save
function TypeString {
	
		parameter page, label, y, cls. 
		if cls {
			SPage(page).
		} else {
			SPageNoClear(page).
		}
		resetcicleset().
		InitTermFileSave(monitors).
		set emptyprog[0] to "". 
		set cuy to 0.
		set cux to 0.
		Print label at(0,y).	
		set offsy to y+1.
		setxy(0,y+1).
}




