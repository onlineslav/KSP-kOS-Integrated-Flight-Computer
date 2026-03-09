@lazyglobal off.

run once "/KOSmodore/rover.ks".

// per il LOCATE x,y e il PRINT
local lx to 0.
local ly to 0.

//line execution counter (address)
global co to -1. //so it starts from 0. tsk!

//cumulative counter (to create the .ks file names)
local cco to -1. //so it starts from 0. tsk!

//line numbers
local linumbers to list(). 

local txtcolor to "".

global stepmode to false.

//"back to basic" case
global backtobascase to 0.


//debug mode. window on the right if true
global debugbas to false.
//all the outputs will be moved right in debug mode for the window
global debugoffset to 0. 

//for the debug windows refreshing time
global debugbasstart to 0.

global exodos to false.

//for the W command (backtobascase=2)
global basWStart to 0.
global basWTime to 0.

global BASICrun to false.


function basAbort {
	//set exodos to true.
	set BASICrun to false.
	resetbasicstate().
	
}

function steprun {
	set stepmode to true.
    if (npage <> 231) {
		spage(231).
		if debugbas {debugwinframe().}
	}
	gopage(231).
}

//function refreshdebug{
//	parameter x.

//}


function debugwinframe{
FROM {local ux is 0.}
		UNTIL ux = 17 STEP {set ux to ux+1.}
		DO { print "             |" at(0,ux).}
		
		print "T: ..." at(0,0).                 
		print "Line: " at(0,1).           
		print "Run:  " at(0,2).     
		print "Case: " at(0,3).
		print "Exit: " at(0,4).     

}

function toggledebug{
	parameter ix.
	
	set debugbas to not debugbas.
	
	set mylabels:currentmonitor to ix.
	
	if debugbas {
		mylabels:setlabel(0, "  Debug([#00FF00]on[#FFFFFF]) ").
		set debugoffset to 15.
		clsnoCur().
		debugwinframe().
		
	}
	
	if not debugbas {
		mylabels:setlabel(0, " Debug([#FFA050]off[#FFFFFF]) ").
		set debugoffset to 0.
		clsnoCur().
	}
	
	if Npage = 220{
		//ClsNoCur().
		local cou to 0.
		
		for lin in emptyprog {
			print lin at(0 + debugoffset, cou).
			set cou to cou + 1.
		}
	}
	
	
	
		
	//refreshdebug(ix).
	
	
}

function debugwin{
	local ti to round(debugbasstart - (floor(debugbasstart/1000)*1000), 1).
	
	print ti + "  " at(6,0).                 
	print co + "  " at(6,1).           
	print BASICrun + "  " at(6,2).     
	print backtobascase + "  " at(6,3).
	print exodos + "  " at(6,4).       
	
	//FROM {local x is 0.}
	//	UNTIL x = 17 STEP {set x to x+1.}
	//	DO { print "|" at(debugoffset-2,x).}
	
}

function resetbasicstate {
	
	if Npage = 231 {
		Spage(231).
		if debugbas {debugwinframe().}
	}
	
	set lx to 0.
	set ly to 0.
	set txtcolor to "".
	set	co to -1.
	set	cco to -1.
    linumbers:clear.
	set exodos to false.
	set basicrun to false.
}

function reclinumbers {
	parameter progr.
	local cou to 0.
	local sepi to 0.
	local linu to 0.	
	
		//clearscreen.
		for l in progr {
			set sepi to l:FIND(" ").
	        if sepi > -1 {
				set linu to l:REMOVE(sepi,(l:length-sepi)):tonumber(-1).
				if linu > -1 {
					linumbers:add(list()).
					linumbers[linumbers:length-1]:add(linu).
					linumbers[linumbers:length-1]:add(cou).
				}
				//print "-" + linu + "-" at(0,cou).
			}
			set cou to cou + 1.
		}
	//print linumbers.
	
}

function removelinumber {
	parameter li.
	local sepi to li:FIND(" ").
	local linu to 0.
	if sepi > -1 {
		set linu to li:REMOVE(sepi,(li:length-sepi)):tonumber(-1).
		if linu > -1 {
			set li to li:REMOVE(0,sepi+1).
		}
	}
	return li.
}

function printgotoid {
	clearscreen.
	print linumbers.
	print posline(14).
}

//return the position of the numbered line
function posline {
	
	parameter nu.
	//local co to 0.
	local re to -1.
	for s in linumbers {
	 //print "l:" + linumbers:length at (14,co+6).
	 //print "co:" + co at (14,co+7).
	 //print s[0] at (14,co+8).
	 //set co to co + 1.
	//print "s di 0:" + s[0] + ".".
	//print "s di 1:" + s[1] + ".".
		if s[0] = nu {
			
			set re to s[1].
			break.
		}// else {
			//set re to -1.
	//	}
	}
	
	return re.
}



function coCLS {
	parameter li.
	if li:trim:toupper = "CLS" {
		if Npage = 230{ClsNoCur().}
		set lx to 0.
		set ly to 0.
	}
}

function coPRINT {
	parameter li.
	local l to li:trim:toupper.
	local lxx to 0.
	local uniq to 0.
	local sepi to l:FIND(" ").
	if sepi > -1 {
		
		if (l:REMOVE(sepi,(l:length-sepi)) = "PRINT") or 
		   (l:REMOVE(sepi,(l:length-sepi)) = "?") {
			//misteriosamente se uso sempre lo stesso nome per il file runpath
			//esegue sempre il primo che ha eseguito. per cui metto nomi diversi 	
			if Npage = 230 or Npage = 231{
				set lxx to (lx + debugoffset).
				set uniq to random().
				//print "lxx:"+lxx+" do:"+debugoffset at (44,ly).
				log "PRINT " + txtcolor + l:REMOVE(0,sepi+1) +
				" at(" + (lxx*1) + "," + ly +")." to // string to number.. tsk!
				"/KOSmodore/temp/print" + uniq + ".ks".
				RUNPATH("/KOSmodore/temp/print" + uniq + ".ks").
				DELETEPATH("/KOSmodore/temp/print" + uniq + ".ks").
			}
			set lx to 0.
			set ly to ly + 1.
		}
	}
}

function coWAIT {
	parameter li.
	local l to li:trim:toupper.
	local uniq to 0.
	local sepi to l:FIND(" ").
	if sepi > -1 {
		
		if (l:REMOVE(sepi,(l:length-sepi)) = "WAIT"){
			//misteriosamente se uso sempre lo stesso nome per il file runpath
			//esegue sempre il primo che ha eseguito. per cui metto nomi diversi 	
			set uniq to random().
			log "WAIT " + l:REMOVE(0,sepi+1) + "."
			to "/KOSmodore/temp/wait" + uniq + ".ks".
			RUNPATH("/KOSmodore/temp/wait" + uniq + ".ks").
			DELETEPATH("/KOSmodore/temp/wait" + uniq + ".ks").
		}
	}
}

function coCOLOR {
	parameter li.
	local l to li:trim:toupper.
	local color to "".
	local sepi to l:FIND(" ").
	
	if l = "CO" {set Txtcolor to "".}
	else
		if sepi > -1 {
			if (l:REMOVE(sepi,(l:length-sepi)) = "CO"){
				set color to l:REMOVE(0,sepi+1).
			
				if color = "R" {set Txtcolor to """[#FF0000]""+".}
				if color = "G" {set Txtcolor to """[#00FF00]""+".}
				if color = "B" {set Txtcolor to """[#0000FF]""+".}
				if color = "Y" {set Txtcolor to """[#FFFF00]""+".}
				if color = "C" {set Txtcolor to """[#FFFF00]""+".}
				if color = "W" {set Txtcolor to """[#FFFFFF]""+".}
				if color = "O" {set Txtcolor to """[#FF8000]""+".}
				if color = "P" {set Txtcolor to """[#FF8080]""+".}
				if color = "M" {set Txtcolor to """[#FF00FF]""+".}
				if color = "V" {set Txtcolor to """[#9933FF]""+".}
				if color = "BLACK" {set Txtcolor to """[#9933FF]""+".}
				if color = "GRAY" {set Txtcolor to """[#909090]""+".}
				if color = "BROWN" {set Txtcolor to """[#663300]""+".}		
			}
		}
}

function coksline {
	parameter li.
	local l to li:trim.
	
	if l:length >0 {
		if l:SUBSTRING(0,1) = "#"{

			log l:REMOVE(0,1):trim to "/KOSmodore/temp/coKSLine" + cco + ".ks".
			RUNPATH("/KOSmodore/temp/coKSLine" + cco + ".ks").
			DELETEPATH("/KOSmodore/temp/coKSLine" + cco + ".ks").
		}
	}
}

function coSD {
	parameter li.
	local l to li:trim:toupper.
	
	local sepi to l:FIND(" ").
	//local lat to 0.
	//local lon to 0.
	local wholeline to li.
	local command to "".
	local parameters to "".
	local par1 to "".
	local par2 to "".
	
	if sepi > -1 {
		set command to l:REMOVE(sepi,(l:length-sepi)).
		if command = "SD" {
			set l to wholeline.
			set parameters to l:REMOVE(0,sepi+1).
			set l to parameters.
			set sepi to l:FIND(" ").
			set par1 to l:substring(0,sepi).
			set l to parameters.
			set par2 to l:REMOVE(0,sepi+1).
		   
		    GPSPOS:clear.
			GPSPOS:add(par1:tonumber(-9999)).
			GPSPOS:add(par2:tonumber(-9999)).
		}
	}
}

function coLOCATE {
	parameter li.
	local l to li:trim:toupper.
	
	local sepi to l:FIND(" ").
	local wholeline to li.
	local command to "".
	local parameters to "".
	local par1 to "".
	local par2 to "".
	
	if sepi > -1 {
		set command to l:REMOVE(sepi,(l:length-sepi)).
		if command = "LOCATE" {
			set l to wholeline.
			set parameters to l:REMOVE(0,sepi+1).
			set l to parameters.
			set sepi to l:FIND(" ").
			set par1 to l:substring(0,sepi).
			set l to parameters.
			set par2 to l:REMOVE(0,sepi+1).
		   	    
			set lx to par1:tonumber(-1).
			set ly to par2:tonumber(-1).
			
			//print "p1:" + par1.
			//print "p2:" + par2.
		}
	}
}

function coSDF {
	parameter li.
	local l to li:trim:toupper.
	
	local sepi to l:FIND(" ").
	local wholeline to li.
	local command to "".
	local param to "".
	
	if sepi > -1 {
		set command to l:REMOVE(sepi,(l:length-sepi)).
		if command = "SDF" {
			set l to wholeline.
			set param to l:REMOVE(0,sepi+1).   
		    set GPSPOS TO READJSON("/KOSmodore/positions/" + param + ".pos").
		}
	}
}

function isGOTO {  //is a goto?
	parameter li.
	local l to li:trim:toupper.
	
	local sepi to l:FIND(" ").
	local itis to false.
	if sepi > -1 {
		if (l:REMOVE(sepi,(l:length-sepi)) = "GOTO") {
			set itis to true.
		}
	}
	return itis.
}

function coGOTO {
	parameter li.
	local l to li:trim:toupper.
	
	local sepi to l:FIND(" ").
	if sepi > -1 {
		
		if (l:REMOVE(sepi,(l:length-sepi)) = "GOTO") {
			local numto to l:REMOVE(0,sepi+1):tonumber(-1).
			//print "l:" + co  at(8,11).
			//print "g:" + numto at(8,12).
			//print "p:" + (posline(numto))-1 at(8,13).
			set co to (posline(numto)-1).  //goto seeks the previous line
		}
	}
}

function coShipSys {
	parameter li.
	if li:trim:toupper = "AG1" {toggle AG1.}
	if li:trim:toupper = "AG2" {toggle AG2.}
	if li:trim:toupper = "AG3" {toggle AG3.}
	if li:trim:toupper = "AG4" {toggle AG4.}
	if li:trim:toupper = "AG5" {toggle AG5.}
	if li:trim:toupper = "AG6" {toggle AG6.}
	if li:trim:toupper = "AG7" {toggle AG7.}
	if li:trim:toupper = "AG8" {toggle AG8.}
	if li:trim:toupper = "AG9" {toggle AG9.}
	if li:trim:toupper = "AG10" {toggle AG10.}
	
	if li:trim:toupper = "GEAR" {toggle GEAR.}
	
	
}

function coRD {
	parameter li.
	if li:trim:toupper = "RD" {
		goroverpos().
	}
}

function coWR {
	parameter li.
	if li:trim:toupper = "WR" {
		set BASICrun to false.
		set backtobascase to 1.
	}
}

function coW {
	parameter li.
	if li:trim:toupper = "W" {
		set basWStart to TIME:SECONDS.
		set basWtime to 1.
		set BASICrun to false.
		set backtobascase to 2.
	}
}


function coEND {
	parameter li.
	local exo to false.
	if li:trim:toupper = "END" {
		set exo to true.
	}
	return exo.
}



function RunBASICline {
	local LiNoN to "". //line without number
	
	if co = -1 {reclinumbers(emptyprog).}
	
	if (not (emptyprog:length > co+1)) {set exodos to true.}
	
	if (exodos=false) {
		set co to co + 1.
		set cco to cco + 1.
		set LiNoN to removelinumber(emptyprog[co]).
		
		//commands
		coCLS(LiNoN).
		coPRINT(LiNoN).  
		coWAIT(LiNoN).
		coLOCATE(LiNoN).
		coSD(LiNoN).
		coSDF(LiNoN).
		coShipSys(LiNoN).
		coRD(LiNoN).
        //coW(LiNoN).
		coWR(LiNoN).
		coKSLine(LiNoN).	
		coCOLOR(LiNoN).			
		coGOTO(LiNoN).
		
		if coEND(LiNoN) {set exodos to true.}
		
	} else {
		set co to -1.
		set cco to -1.
		set exodos to false.
		set BASICrun to false.
		resetbasicstate().
	}
	//print "co:" + co + "(" + emptyprog:length + ")" at(11,11).
	
	//set BASICrun to false.
	if stepmode {set BASICrun to false.}
	
	
	if exodos {
		set BASICrun to false.
		resetbasicstate().
	
	}
}
