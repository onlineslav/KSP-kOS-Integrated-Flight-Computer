@lazyGlobal off.

run once "/KOSmodore/interface.ks".
run once "/KOSmodore/main.ks".


// 0   time
// 1   Light exposition of the solar panels 
// 2   temperature outside the command module
// 3   pressure outside the command module
// 4   mass
// 10  dynamic pressure (Q)
// 20  Air speed (How fast the ship is moving relative to the air)
// 21  Vertical Speed
// 22  Ground Speed (horizontal speed)

// 101 latitude
// 102 longitude
global DataSources TO LIST().
global DataSourcesAdded TO LIST().
//set mylexicon to lexicon().
//set mylexicon["key1"] to "value1".
//set mylexicon["key2"] to "value2".

function InitSensLog {	
		SensLog:add(LIST()).
	    SensLog[SensLog:LENGTH-1]:add(0). //Time
		
		for s in DataSourcesAdded {
			SensLog[SensLog:LENGTH-1]:add(s).
		}
		//SensLog[SensLog:LENGTH-1]:add(1). //Light exposition of the solar panels 
		//FOR S IN SENSELIST {
			//if S:TYPE = "TEMP" {
			//	SensLog[SensLog:LENGTH-1]:add(2).
//			}	
//			if S:TYPE = "PRES" {
//				SensLog[SensLog:LENGTH-1]:add(3).
//			}		
//		}
}


function riempisourcelist {
	//DataSources:add(0).
	DataSources:add(1). // Light exposition of the solar panels 
	DataSources:add(4). // Vessel mass
	DataSources:add(10). // dynamic pressure (Q)
	DataSources:add(20). // Air speed
	DataSources:add(21). // vertical speed
	DataSources:add(22). // ground speed
	DataSources:add(101).
	DataSources:add(102).
	
	FOR S IN SENSELIST {
		
			if S:TYPE = "TEMP" {
				DataSources:add(2).
			}
	
			if S:TYPE = "PRES" {
				DataSources:add(3).
			}		
		}
}

// converte un codice nel nome del data source
Function DSname {
	parameter nu.
	local str to "".
	if nu = -1 { set str to "(no datasources)".}
	if nu = 0 { set str to "Time".}
	if nu = 1 { set str to "Light exposition".}	
	if nu = 2 { set str to "Temperature".}	
	if nu = 3 { set str to "Pressure".}
	if nu = 4 { set str to "Vessel mass".}	
	if nu = 10 { set str to "Dynamic pressure (Q)".}
	if nu = 20 { set str to "Air speed".}		
	if nu = 21 { set str to "Vertical speed".}	
	if nu = 22 { set str to "Ground Speed".}	
	if nu = 101 { set str to "Latitude".}	
	if nu = 102 { set str to "Longitude".}	
	return str.
}

// converte una lista di codici in una lista di nomi dei data source
Function DSnames {	     
	parameter li.        // list input
	local lo to list().  // list output
	FOR ds in li {
		lo:add(DSname(ds)).
	}
	return lo.
}

function ShowAddedDSources {   
	parameter co.
	//local co to 0.
	if DataSourcesAdded:length = 0 {
		print "No sources selected." at(0,co).
	} else {
		FOR ds in DataSourcesAdded {
			//print DSname(ds) + " (" + ds + ")" at(0,co).
			print "-" + DSname(ds) at(0,co).
			set co to co + 1.
		}
	}
}

//Aggiunge ad una lista di interi un elemento solo se non presente
function AddIfNoItem {
	parameter li, item.
	//local lo.
	if li:find(item) = -1 {
		li:add(item).
	}
	return li.
}

//toglie da una lista di interi un elemento
function SubItem {
	parameter li, item.
	//local lo.
	//if not (li:find(item) = -1) {
		li:remove(li:find(item)).
	//}
	return li.
}

function ViewDataLogSource {
	parameter Source.
	local x to 0.
	local y to 0.
	
	if Source = -1 {
		SPageTile(72,"Time         " + DSname(-1)).
	} else {
		SPageTile(72,"Time         " + DSname(SensLog[0][Source])).
	}
	
	FOR S IN SensLog {
		if y>0 {
			FOR SS IN S {
				if x = 0 {
					print round(SS,1) at (0,Y).
				} 
				if x = Source {
					print round(SS,6) at (13,Y).
				}
				set x to x+1.
			}		
		}
		set y to y+1.
		set x to 0.
	}
}

function PrevColumnSource {
	PrevColumn().
	ViewDataLogSource(selcolumn).
}
	
function NextColumnSource {
	NextColumn().
	ViewDataLogSource(selcolumn).
}
	

