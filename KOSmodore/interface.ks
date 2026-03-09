@lazyglobal off.

run once "/KOSmodore/globals.ks".
run once "/KOSmodore/texted.ks".

 
global lind to 0.           // selezione della lista di cose
global Npage to 0.
global li to list().        // lista per il menu scorrevole
global selcolumn to -1.     // datasource selezionata
                            // -1: non ci sono sorgenti
							// 1: prima sorgente impostata
							// 2: seconda sorgente...
global maxcolumn to 0.      // numero di colonne disponibili

//associates a back page to any page 
global PageBacks to list(
	list(  0,   -1),
	list(  1,   -1), 
	list( 20,    1),
	list( 30,    1),
	list( 31,   30),
	list( 32,   30),   
	list( 33,   30),
	list( 34,   30),
	list( 35,   30),
	list( 40,    1),
	list( 41,   40),
	list( 42,   40),
	list( 43,   40),
	list( 50,    1),
	list( 51,   50),
	list( 52,   50),
	list( 53,   50),
	list( 54,   50),
	list( 55,   50),
	list( 56,   50),
	list( 70,    1),
	list( 71,   70),
	list( 72,   70),
	list( 75,   70),
	list( 76,   75),
	list( 77,   75),
	list( 83,   70),
	list( 84,   70),
	list( 85,   70),
	list(100,    1),
	list(101,  100),
	list(102,  100),
	list(103,  100),
	list(200,    1),
	list(201,  200), 
	list(205,  200), 
	list(210,  200),
	list(220,  200),
	list(221,  200),
	list(237,  220),
	list(238,  220),
	list(239,  220),
	list(240,  220),
	list(277,  218),
	list(278,  218),
	list(279,  218),
	list(280,  218)
).


// returns the back page
function PageBack {
	
	for s in PageBacks {
		if s[0] = NPage {
			return s[1].
		}
	}
}

//global li to list().                           // lista per il menu scorrevole

//dice al sistema qual è la pagina corrente e prepara lo schermo spingendo il cursore lontano

// PAGES 
//  0         splash (dalla release 1 durerà un secondo e i tasti permetteranno easter egg e funzioni nascoste - per esempio visualizzare in numero di pagina. dopo quel secondo si passerà alla pagina 1)

//  1         main page

//  8         Test page

//  20        Rover page

//  30        Track page
//  31        selezione e carica track da file
//  32        track view
//  33        save track 0
//  34        save track 1
//  35        save track 2
  //  37        reaname track 0
  //  38        reaname track 1
  //  39        reaname track 2

//  -40        Logbook 
//  -41        Logbook write 1
//  -42        Logbook write 2
//  -43        Logbook write 3

//  50        Destination
//  51        selezione e carica destinazione da file
//  52        save destination 0
//  53        save destination 1
//  54        save destination 2
//  55        type d. latitude
//  56        type d. longitude
  //  57        reaname dest 0
  //  58        reaname dest 1
  //  59        reaname dest 2

//  70        datalog page
//  71        select e carica datalog da file
//  72        data log view
//  74        ask confirmation before changing the source list
//  75        seleziona data sources
//  76        add data source da lista
//  77        del data source da lista
//  83        save datalog 0
//  84        save datalog 1
//  85        save datalog 2
  //  87        reaname datalog 0
  //  88        reaname datalog 1
  //  89        reaname datalog 2

//  100       Settings
//  101       type marker thickness
//  102       type Data sampling time interval
//  103       type Track sampling time interval
  //  187        reaname ks prog 0
  //  188        reaname ks prog 1
  //  189        reaname ks prog 2
  //  197        reaname sks prog 0
  //  198        reaname sks prog 1
  //  199        reaname sks prog 2
//  200       programs
//  201       select and run kerboscript from file
  //  205       select and load serializedkerboscript from file
  //  207       export sks to ks file 0 (ask name)
  //  208       export sks to ks file 1 (ask name)
  //  209       export sks to ks file 2 (ask name)
//  210       running program
//  211       ended program
//  218       View sks prog
//  220       view basic prog
//  221       select and load basic prog from file
  //  227        reaname basic prog 0
  //  228        reaname basic prog 1
  //  229        reaname basic prog 2
//  230       running basic program
//  237       edit basic program 0
//  238       edit basic program 1
//  239       edit basic program 2
//  240       edit basic program 3
//  243       save basic program 0
//  244       save basic program 1
//  245       save basic program 2
  //  253       save sks program 0
  //  254       save sks program 1
  //  255       save sks program 2

//  277       edit sks program 0
//  278       edit sks program 1
//  279       edit sks program 2
//  280       edit sks program 3

function EscSaveFile {
	if NPage = 52 or NPage = 53 or NPage = 54 {
		set offsy to 0.
		set cuY to 0.
		GoPage(50).
	}
	
	 // file rename
	if NPage = 37 or NPage = 38 or NPage = 39 {
		set offsy to 0.
		set cuY to 0.
		GoPage(31).
	}
	if NPage = 57 or NPage = 58 or NPage = 59 {
		set offsy to 0.
		set cuY to 0.
		GoPage(51).
	}
	if NPage = 87 or NPage = 88 or NPage = 89 {
		set offsy to 0.
		set cuY to 0.
		GoPage(71).
	}
	if NPage = 187 or NPage = 188 or NPage = 189 {
		set offsy to 0.
		set cuY to 0.
		GoPage(201).
	}
	if NPage = 197 or NPage = 198 or NPage = 199 {
		set offsy to 0.
		set cuY to 0.
		GoPage(205).
	}
	if NPage = 227 or NPage = 228 or NPage = 229 {
		set offsy to 0.
		set cuY to 0.
		GoPage(221).
	}
	
	if NPage = 33 or NPage = 34 or NPage = 35 {
		set offsy to 0.
		set cuY to 0.
		Gopage(30).
	}
	if NPage = 83 or NPage = 84 or NPage = 85 {
		set offsy to 0.
		set cuY to 0.
		Gopage(70).
	}
	if NPage = 207 or NPage = 208 or NPage = 209 {
		set emptyprog to STextAux:copy.
		STextAux:clear.
		set offsy to 0.
		set cuY to 0.
		Gopage(218).
	}
	if NPage = 243 or NPage = 244 or NPage = 245 {
		set emptyprog to STextAux:copy.
		STextAux:clear.
		set offsy to 0.
		set cuY to 0.
		Gopage(220).
	}
	if NPage = 253 or NPage = 254 or NPage = 255 {
		set emptyprog to STextAux:copy.
		STextAux:clear.
		set offsy to 0.
		set cuY to 0.
		Gopage(218).
	}
}

function TypeRealNumAccept {    
	if Npage = 56 { //tipico caso in cui l'ordine degli if conta
		set INPstr2 to emptyprog[0].   
		GPSPOS:clear.
		GPSPOS:add(INPstr1:tonumber(-9999)). //se non è un numero valido 	restituisce -9999
		GPSPOS:add(INPstr2:tonumber(-9999)).		
		GoPage(50).
    }
	if Npage = 55 {
		set INPstr1 to emptyprog[0].
		cuBlank().  //Blanken the cursor
		TypeRealNum(56,"Type longitude: ",2, false).
		
	}

	if Npage = 101 {
		set INPstr1 to emptyprog[0].
		set SettingsL[0] to INPstr1:tonumber(-9999).
		GoPage(100).
	}
	if Npage = 102 {
		set INPstr1 to emptyprog[0].
		set SettingsL[1] to INPstr1:tonumber(-9999).
		GoPage(100).
	}
	if Npage = 103 {
		set INPstr1 to emptyprog[0].
		set SettingsL[2] to INPstr1:tonumber(-9999).
		GoPage(100).
	}
}



// cancella la pagina facendo sparire il cursore
function ClsNoCur {
	clearscreen.
	FROM {               //porta il cursore dove non lo vede nessuno
		local xx is 1.
	}
	UNTIL xx = 18 STEP {
		set xx to xx+1.
	} 
	DO {
		print " ".
	}
}

function SPage{
	parameter p.
	ClsNoCur().
	print "P" + p at(SettingsL[3]-4,16).
		set Npage to p.
}


function SPageTile{
	parameter p,title.
	clearscreen.
	  print "[#DFDFDF]" + title.
	FROM {               //porta il cursore dove non lo vede nessuno
		local xx is 1.
		}
	UNTIL xx = 17 STEP {
		set xx to xx+1.
	} 
	DO {
		print " ".
	}
	print "P" + p at(SettingsL[3]-4,16).
	set Npage to p.
}

function SPageNoClear{
	parameter p.
	print "P" + p at(SettingsL[3]-4,16).
	set Npage to p.
}


function ruler {
	FROM {
		local xx is 1.
		}
	UNTIL xx = 17 STEP {
		set xx to xx+1.
	} 
	DO {
		print "- " + xx at (0,xx).
	}
}

function listafile {   //stampa una lista selezionalible di file
	parameter li.
	local cou to 0.
    local fileline to "".
	if lind > li:length-1 {
		set lind to 0.
	}
	if lind < 0 {
		set lind to li:length-1.
	}
	
	FOR bod in li {
		set fileline to bod:name + " (" + bod:size+" B)          ".
		if cou = lind {
			print " > [#FFFFFF]" + fileline at(0,lind).
		} else {
			PRINT "  " + fileline at(0,cou).
		}
		set cou to cou + 1.
	}
	
	 
}

// prints the cursor and changes its position in memory. it is the cursor of the 
// text editors
function SetEditCursor {
	parameter li.
	local cou to 0.
	FOR bod in li {
		//PRINT "#" at(bod:length,cou). // when it will scroll, cut it
		set cou to cou + 1.
	}
	if lind > cou-1 {set lind to cou-1.}
	if lind < 0 {set lind to 0.}
	print "_" at(li[lind]:length,lind). 
	setxy(emptyprog[lind]:length,lind).	
}

//setta il cursore per scrivere riscrivere tutta la riga (x=0)
function SetEditCursor0 {
	parameter li.
	local cou to 0.
	
	FOR bod in li {
		//PRINT "#" at(bod:length,cou). // when it will scroll, cut it
		set cou to cou + 1.
	}
	
	if lind > cou-1 {
		set lind to cou-1.
	}
	if lind < 0 {
		set lind to 0.
	}
	//PRINT "#" at(li[lind]:length + linea:length, lind). // when it will scroll, cut it
	print "_" at(li[lind]:length,lind). 
	
	setxy(0,lind).
	//print lind at(0,8). 
	
}



function listastr {   //stampa una lista selezionalible di stringhe
	parameter lis.
	//global li to LIST().
	local cou to 0.
 
	FOR bod in lis {
		PRINT "  " + bod at(0,cou).
		set cou to cou + 1.
	}
	if lind > cou-1 {
		set lind to 0.
	}
	if lind < 0 {
		set lind to cou-1.
	}
	print ">" at(1,lind). 
}

function ListaFileDown{
	//if (Npage = 10) or (Npage = 11) or (Npage = 51) {
		set lind to lind +1.
		listafile(li).
}

function ListaFileUp{
	//if (Npage = 10 ) or (Npage = 11) or (Npage = 51) {
		set lind to lind -1.
		listafile(li).
}

//function EditLineDown{
//	//if (Npage = 10) or (Npage = 11) or (Npage = 51) {
//		//set emptyprog[lind] to linea.
//		PRINT " " at(emptyprog[lind]:length, lind).
//		set lind to lind +1.
//		SetEditCursor(li).
//		
//		set linea to "".
//}

//function EditLineUp{
//	//if (Npage = 10 ) or (Npage = 11) or (Npage = 51) {
//		//set emptyprog[lind] to linea.
//		PRINT " " at(emptyprog[lind]:length, lind).
//		set lind to lind -1.
//		SetEditCursor(li).
//		
//		set linea to "".	
//}

//save the current line. you must do it, when you change line before saving the file, before running it..
//function EditRecordLine{
//	set emptyprog[lind] to linea.
//}

function ListaStrDown{
	//if (Npage = 10) or (Npage = 11) or (Npage = 51) {
		parameter li.
		set lind to lind +1.
		listaStr(li).
}

function ListaStrUp{
	//if (Npage = 10 ) or (Npage = 11) or (Npage = 51) {
		parameter li.
		set lind to lind -1.
		listaStr(li).
}


// sistema deprecato: fare una funzionae per ogni if
function EnterListselect{
	
	
	if Npage = 51 {
		set GPSPOS TO READJSON("/KOSmodore/positions/" + li[lind]).
		GoPage(50).
	}
	if Npage = 31 {
		set LoadedTrack TO READJSON("/KOSmodore/tracks/" + li[lind]).
		Gopage(30).
	}
	if Npage = 71 {
		set SensLog TO READJSON("/KOSmodore/sampling/" + li[lind]).
		
		
		//reimposta la lista risorse in accordo col file caricato
		local co to 0.
		DataSourcesAdded:clear.
		FOR ds in SensLog[0] {
			//print DSname(ds) + " (" + ds + ")" at(0,co).
			if co > 0 {
				DataSourcesAdded:add(ds).
				print DSname(ds) at(0,co+5).
			}
			set co to co + 1.
		}
		
		GoPage(70).
	}
	if Npage = 76 {
		set DataSourcesAdded to AddIfNoItem(DataSourcesAdded,DataSources[lind]).
		GoPage(75).
	}
	if Npage = 77 {
		set DataSourcesAdded to SubItem(DataSourcesAdded,DataSourcesAdded[lind]).
		GoPage(75).
	}
	if Npage = 201 {
		GoPage(210).
		RunKS(li[lind]).
		GoPage(211).
		
	}
	if Npage = 205 {
		set emptyprog to readjson("/KOSmodore/sks/" + li[lind]).
		GoPage(218).
		
	}
	if Npage = 221 {
		set emptyprog to readjson("/KOSmodore/ksp-basic/" + li[lind]).
		GoPage(220).
		
	}
}

function PrevColumn {
	if selcolumn > 1 {
		set selcolumn to selcolumn - 1.
		ViewDataLogSource(selcolumn).
	}
}
	
function NextColumn {
	if (selcolumn < maxcolumn) and not(selcolumn = -1) {
		set selcolumn to selcolumn + 1.
		ViewDataLogSource(selcolumn).
	}
}

function GoPageResettingCur {
	parameter p.
	set cuy to 0.
	set cux to 0.
	set offsy to 0.
	GoPage(p).
}

