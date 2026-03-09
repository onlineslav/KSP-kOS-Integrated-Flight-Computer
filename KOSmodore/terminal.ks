@lazyglobal off.

run once "/KOSmodore/rover.ks".
run once "/KOSmodore/keybT9.ks".
run once "/KOSmodore/keyT9masks.ks".
run once "/KOSmodore/logbook.ks".
run once "/KOSmodore/files.ks".
run once "/KOSmodore/GPS.ks".
run once "/KOSmodore/test.ks".
run once "/KOSmodore/hovering.ks".
//run once "/KOSmodore/basicflow.ks".

global mybuttons to addons:kpm:buttons.
global mylabels to addons:kpm:labels.
global myflags to addons:kpm:flags.

// silly infos
function button05Press {
	clearscreen.	
	print "GUID: " + id.
	print "monindex: " + monindex.
}

function MSetLabel{
	parameter nu.
	parameter stri.
	parameter monitors.
	
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		set mylabels:currentmonitor to x.
		mylabels:setlabel(nu, stri).
	}
}

function MSetFlag{
	parameter nu.
	parameter boo.
	parameter monitors.
	
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		 set myflags:currentmonitor to x. 
	     myflags:setstate(nu,FALSE).
	}
}
	
function MNOP{
}

function InitTerminalMAIN {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
	    
		mylabels:setlabel(0,"    Dest.   ").
		mylabels:setlabel(1,"  Tracks  ").
		mylabels:setlabel(2," Data log ").
		mylabels:setlabel(3,"   Rover  ").
		
		mylabels:setlabel(5," Programs ").
		mylabels:setlabel(6,"            ").   //assegnamento coatto a EXIT
		
		//mylabels:setlabel(7,"     TEST   ").
		mylabels:setlabel(9,"  Reboot  ").
		mylabels:setlabel(10,"  Exit OS ").
		mylabels:setlabel(13," Settings ").
  
		mybuttons:setdelegate(0,GoPage@:bind(50)).
		mybuttons:setdelegate(1,GoPage@:bind(30)).
		mybuttons:setdelegate(2,GoPage@:bind(70)).	
		mybuttons:setdelegate(3,GoPage@:bind(20)). //rover page
		//mybuttons:setdelegate(4,GoPage@:bind(40)).
		mybuttons:setdelegate(5,GoPage@:bind(200)).
		//mybuttons:setdelegate(7,GoPage@:bind(8)).
		
		mybuttons:setdelegate(9,MyReboot@).
		mybuttons:setdelegate(10,exitos@).
		mybuttons:setdelegate(13,GoPage@:bind(100)).
		 
		myflags:setlabel(0,"Dest.").
		myflags:setlabel(1,"Track").	
		myflags:setlabel(2,"D.log").  
	}
}


// TEST - 8
function InitTermTEST {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
	    
		mylabels:setlabel(0,"   Hover    ").
		mylabels:setlabel(5,"  Test ΔV ").   //assegnamento coatto a EXIT
		
		mybuttons:setdelegate(0,hovertest@).
		mybuttons:setdelegate(5,stampa@).
		
		mybuttons:setdelegate(-2,GoPage@:bind(1)).      //CANCEL>main
		
		
	}
}

// Destination
function InitTerminalDestination {

	parameter monitors.
	
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		
		mylabels:setlabel(0,"   Set Here ").
		mylabels:setlabel(1,"  Type D. ").
		
		mylabels:setlabel(7,"   Clear D. ").
		mylabels:setlabel(8," Save D.  ").
		mylabels:setlabel(9," QSave D. ").
		mylabels:setlabel(10,"  Load D. ").
		mylabels:setlabel(12,"  Draw D. ").
		mylabels:setlabel(13,"  Hide D.   ").
		
		
		mybuttons:setdelegate(0,toggleFlag@:BIND(0)).
		mybuttons:setdelegate(1,TypeRealNum@:BIND(55,"Type latitude: ",0, true)).
		mybuttons:setdelegate(2,GDestination@).
		mybuttons:setdelegate(7,ClrGPSPOS@).
		mybuttons:setdelegate(8,Savefilelist@:bind(0)).
		mybuttons:setdelegate(9,SaveDestRND@).
		mybuttons:setdelegate(10,GoPage@:bind(51)).
		mybuttons:setdelegate(12,GDestination@).
		mybuttons:setdelegate(13,HideTRK@).
	
		mybuttons:setdelegate(-2,GoPage@:bind(1)).      //CANCEL>main
		
		myflags:setlabel(0,"Dest.").  
	}
}



// Type RealNum Accept - P55 (latitude) - p56 (lng) - 101 - 102 - 103
function InitTermTypeRealNum {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		ClearTerminal(x).
		
		SetNumbersOnlyE(x).
	
	
		
		mybuttons:setdelegate(-5,CuLeft@).           //LEFT
		mybuttons:setdelegate(-6,CuRight@).          //RIGHT
	
	    mybuttons:setdelegate(-2,GoPage@:bind(PageBack())). //CANCEL
	}
}


// Save FIle 0 - P52 dest - p33 track - p83 Data Log - p243 bas
function InitTermFileSave {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		SetBasicSymbolsE(x).
	
		mylabels:setlabel(7,"     [...]  ").
		mylabels:setlabel(8,"  - + & . ").
		
		mybuttons:setdelegate(7,cicleset@).
		mybuttons:setdelegate(8,punct3e@).
	
		mybuttons:setdelegate(-1,saveas@).        //ENTER
		mybuttons:setdelegate(-2,EscSaveFile@).   //CANCEL
		mybuttons:setdelegate(-5,CuLeft@).           //LEFT
		mybuttons:setdelegate(-6,CuRight@).          //RIGHT
		
	}
}

// Save File 1 - P53 dest - p34 track - p84 Data Log
function InitTermFileSave1 {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		SetBasicSymbolsLowerE(x).

		mylabels:setlabel(7,"     [...]  ").
		mylabels:setlabel(8,"  - + & . ").
		
		mybuttons:setdelegate(7,cicleset@).
		mybuttons:setdelegate(8,punct3e@).
	
		mybuttons:setdelegate(-1,saveas@).        //ENTER
		mybuttons:setdelegate(-2,EscSaveFile@).   //CANCEL
		mybuttons:setdelegate(-5,CuLeft@).           //LEFT
		mybuttons:setdelegate(-6,CuRight@).          //RIGHT
	}
}

// Save File 2 - P54 dest - p35 track - p85 Data Log
function InitTermFileSave2 {
parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		SetBasicSymbolsNumbersE(x).
  
		mylabels:setlabel(7,"     [...]  ").
		mylabels:setlabel(8,"  - + & . ").
		
		mybuttons:setdelegate(7,cicleset@).
		mybuttons:setdelegate(8,punct3e@).
  
		mybuttons:setdelegate(-1,saveas@).        //ENTER
		mybuttons:setdelegate(-2,EscSaveFile@).   //CANCEL
		mybuttons:setdelegate(-5,CuLeft@).           //LEFT
		mybuttons:setdelegate(-6,CuRight@).          //RIGHT
	}
}


// TRACKS - page 30
function InitTerminalGPS {

	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		mylabels:setlabel(0,"    + Here  ").
		mylabels:setlabel(1,"  - Last  ").
		mylabels:setlabel(2,"  - First ").
		mylabels:setlabel(3,"  View T. ").
		mylabels:setlabel(4,"  Log TRK ").
		mylabels:setlabel(7,"  Clear TRK ").
		mylabels:setlabel(8,"  Save T. ").
		mylabels:setlabel(9,"  Load T. ").
		//mylabels:setlabel(10,"QSave T.").
		mylabels:setlabel(12," Draw TRK ").
		mylabels:setlabel(13," Hide TRK   ").
 
	
		mybuttons:setdelegate(0,AddHereToTrack@).
		mybuttons:setdelegate(1,RemoveLastFromTrack@).
		mybuttons:setdelegate(2,RemoveFirstFromTrack@).
		mybuttons:setdelegate(3,GoPage@:bind(32)).
		mybuttons:setdelegate(4,toggleFlag@:BIND(1)). 
		mybuttons:setdelegate(7,ClearGPSLOG@).
		mybuttons:setdelegate(8,Savefilelist@:bind(0)).
		mybuttons:setdelegate(9,GoPage@:bind(31)).
		//mybuttons:setdelegate(10,SaveDestRND@).
		mybuttons:setdelegate(12,DrawTRK@).
		mybuttons:setdelegate(13,HideTRK@).	
		
		mybuttons:setdelegate(-2,GoPage@:bind(1)).         //CANCEL
	}
}



// File select and load - P31 - P51 - P71 - P201
function InitTermFileLoad {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		ClearTerminal(x).
		
		mylabels:setlabel(0," Make a copy").
		mylabels:setlabel(1,"  Rename  ").
		mylabels:setlabel(3,"  [#FF4040]Delete  ").
		
		mybuttons:setdelegate(0,MkcopyFileList@).
		mybuttons:setdelegate(1,RenFileList@).  		
		mybuttons:setdelegate(3,delFileList@).    
		
		mybuttons:setdelegate(-2,GoPage@:bind(PageBack())).  //CANCEL
		mybuttons:setdelegate(-1,EnterListselect@).          //ENTER
		mybuttons:setdelegate(-3,listaFileUp@).              //UP
		mybuttons:setdelegate(-4,listaFileDown@).            //DOWN
	}
}

// TRACKS view - page 32
function InitTerminalViewTrack {

	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		
		mybuttons:setdelegate(-2,GoPage@:bind(30)).         //CANCEL
	}
}

// Data Log page 70
function InitTerminalSampler {

	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		mylabels:setlabel(0,"   + Sample ").
		mylabels:setlabel(1,"    Log   ").
		mylabels:setlabel(2,"  Sources ").
		mylabels:setlabel(4,"  View DL ").
		
		mylabels:setlabel(7,"  Clear DL  ").
		mylabels:setlabel(8,"  Save DL ").
		mylabels:setlabel(9,"  Load DL ").
				
		mybuttons:setdelegate(0,DataLogAdd@).  
		mybuttons:setdelegate(1,toggleFlag@:BIND(2)). 
		mybuttons:setdelegate(2,GoPage@:bind(74)).  
		mybuttons:setdelegate(4,GoPage@:bind(72)).  
		mybuttons:setdelegate(7,ClearSensLOG@).
		mybuttons:setdelegate(8,Savefilelist@:bind(0)).
		mybuttons:setdelegate(9,GoPage@:bind(71)).
		
		mybuttons:setdelegate(-2,GoPage@:bind(1)).         //CANCEL
		
		myflags:setlabel(2,"D.log").	
	}
}



// view data log - P72
function InitTerminalViewDataLog {

	parameter monitors.
	
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		
		mylabels:setlabel(1," < Source ").
		mylabels:setlabel(2," Source > ").
		
		mybuttons:setdelegate(1,PrevColumnSource@). 
		mybuttons:setdelegate(2,NextColumnSource@).  
				
		mybuttons:setdelegate(-2,GoPage@:bind(70)).         //CANCEL
	}
}

// ROVER
function InitTerminalROVER {

	parameter monitors.
	
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		
		mylabels:setlabel(4," Go Track ").   // segui track
		mylabels:setlabel(5,"  Go Dest ").  // vai a dest
		mylabels:setlabel(7,"   Brake on ").
		mylabels:setlabel(8,"  BRK off ").
	
		mylabels:setlabel(10,"  Reboot  ").
 
		mybuttons:setdelegate(4,gorovertrack@).
		mybuttons:setdelegate(5,goroverpos@).
		mybuttons:setdelegate(7,brakes_on@).
		mybuttons:setdelegate(8,brakes_off@).	
		mybuttons:setdelegate(10,MyReboot@).
	
		mybuttons:setdelegate(-2,GoPage@:bind(1)).         //CANCEL
	}
}

// PAGE 40
function InitTermLogBook {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
  
		mylabels:setlabel(0,"    + Page  ").
		mylabels:setlabel(1,"   Edit   ").
		mylabels:setlabel(2,"  Previus ").
		mylabels:setlabel(3,"   Next   ").
	
		mybuttons:setdelegate(0,LbookNewPage@).
		mybuttons:setdelegate(1,GoPage@:bind(41)). //Logbook write
		mybuttons:setdelegate(2,showPrevNote@).
		mybuttons:setdelegate(3,showNextNote@).
	
		mybuttons:setdelegate(-2,LeaveLBook@).         //CANCEL	
	}
}

// page 74 - the data log will be lost. proceed?
function InitTermProceedToSource{
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		
		mylabels:setlabel(0,"      OK    ").
		mylabels:setlabel(1,"  Cancel  ").
				
		mybuttons:setdelegate(0,GoPage@:bind(75)).
		mybuttons:setdelegate(1,GoPage@:bind(70)).
		
		mybuttons:setdelegate(-2,GoPage@:bind(70)).             //CANCEL
	}
}

// page 75
function InitTermShowDataSources{
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		
		mylabels:setlabel(0,"   + Source ").
		mylabels:setlabel(1," - Source ").
				
		mybuttons:setdelegate(0,GoPage@:bind(76)).
		mybuttons:setdelegate(1,GoPage@:bind(77)).
		
		mybuttons:setdelegate(-2,GoPage@:bind(70)).            //CANCEL
	}
}

// pages 76 
function InitTermSelectDataSourcesAdd{
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		
		mybuttons:setdelegate(-2,GoPage@:bind(70)).            //CANCEL
		mybuttons:setdelegate(-1,EnterListselect@).   //ENTER
		mybuttons:setdelegate(-3,listaStrUp@:bind(DSnames(DataSources))). //UP
		mybuttons:setdelegate(-4,listaStrDown@:bind(DSnames(DataSources))). //DOWN
	}
}

// p 77
function InitTermSelectDataSourcesSub{
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		
		mybuttons:setdelegate(-2,GoPage@:bind(70)).             //CANCEL
		mybuttons:setdelegate(-1,EnterListselect@).   //ENTER
		mybuttons:setdelegate(-3,listaStrUp@:bind(DSnames(DataSourcesAdded))). //UP
		mybuttons:setdelegate(-4,listaStrDown@:bind(DSnames(DataSourcesAdded))). //DOWN
	}
}

// page 100 - Settings
function InitTermSettings{
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		
		mylabels:setlabel(0,"   Set m. t.").
		mylabels:setlabel(1," Set d. i.").
		mylabels:setlabel(2," Set t. i.").
						
		mybuttons:setdelegate(0,TypeRealNum@:BIND(101,"Type marker thickness: ",0, true)).
		mybuttons:setdelegate(1,TypeRealNum@:BIND(102,"Type DL interval (s): ",0, true)).
		mybuttons:setdelegate(2,TypeRealNum@:BIND(103,"Type TRK interval (s): ",0, true)).

		mybuttons:setdelegate(-2,GoPage@:bind(1)).             //CANCEL
	}
}

// page 200 - Programs
function InitTermPrograms{
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
		

		mylabels:setlabel(0,"   New KS   ").
		mylabels:setlabel(1,"New BASIC ").
		
		mylabels:setlabel(4,"  Run .ks ").
		mylabels:setlabel(7,"  Load KS ").
		mylabels:setlabel(8,"Load BASIC").
						
	    mybuttons:setdelegate(0,NewKSFile@).
		mybuttons:setdelegate(1,NewBasFile@).
		mybuttons:setdelegate(4,GoPage@:bind(201)).
		mybuttons:setdelegate(7,GoPage@:bind(205)).
		mybuttons:setdelegate(8,GoPage@:bind(221)).
	  
		
		mybuttons:setdelegate(-2,GoPage@:bind(1)).             //CANCEL
	}
}



// page 211 - ended program
function InitTermEndedProgram {

	parameter monitors.
	
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
				
		mybuttons:setdelegate(-2,GoPage@:bind(200)).       //CANCEL
		
	}
}

// PAGE 218 view sks prog
function InitTermViewSKS {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
  
		mylabels:setlabel(0,"  Export .ks").
		mylabels:setlabel(2,"   Edit   ").
		
		mylabels:setlabel(7, "     Clear  ").
		mylabels:setlabel(8, "   Save   ").
		mylabels:setlabel(9, "   Load   ").
				
		mybuttons:setdelegate(0,copyvarsave@:bind(207,1)).
		mybuttons:setdelegate(2,GoPageResettingCur@:bind(277)).
		//mybuttons:setdelegate(3,GoPage@:bind(222)).
		
		mybuttons:setdelegate(7,ClearKSFile@).	
		mybuttons:setdelegate(8,copyvarsave@:bind(253,0)).	
		mybuttons:setdelegate(9,GoPage@:bind(205)).
		
		
		mybuttons:setdelegate(-2,GoPage@:bind(200)).         //CANCEL
		
	}
}

// PAGE 220 view basic prog
function InitTermViewBas {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
  
		if debugbas {mylabels:setlabel(0, "  Debug([#00FF00]on[#FFFFFF]) ").}
		else {mylabels:setlabel(0, " Debug([#FFA050]off[#FFFFFF]) ").}
		
		mylabels:setlabel(1, "    Run   ").
		mylabels:setlabel(2, " Step run ").
		mylabels:setlabel(3, "   Reset  ").
		
		
		mylabels:setlabel(4, "    Edit  ").
		
		//mylabels:setlabel(5, "  Show #  ").
		mylabels:setlabel(7, "   Clear  ").
		mylabels:setlabel(8, "   Save   ").
		mylabels:setlabel(9, "   Load   ").
		//mylabels:setlabel(11, "  GOTO-id ").
				
		//mybuttons:setdelegate(0,DebugToggle@:bind(x)).
		mybuttons:setdelegate(0,toggledebug@:bind(x)).
	
		//mybuttons:setdelegate(1,runlinebas@:bind(emptyprog)).
		mybuttons:setdelegate(1,GoPage@:bind(230)).
		mybuttons:setdelegate(2,steprun@).
		mybuttons:setdelegate(3,resetbasicstate@).
		mybuttons:setdelegate(4,GoPageResettingCur@:bind(237)).
		
		mybuttons:setdelegate(7,initstextvar@).	
		mybuttons:setdelegate(8,copyvarsave@:bind(243,0)).
		mybuttons:setdelegate(9,GoPage@:bind(221)).
		//mybuttons:setdelegate(11,PrintGotoid@).
		
		mybuttons:setdelegate(-2,GoPage@:bind(200)).         //CANCEL
		
	}
}


// PAGE 230 - executing basic
function InitTermExeBas {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
        mylabels:setlabel(4,"   Abort  ").
				 
		mybuttons:setdelegate(4,basAbort@).
		
		mybuttons:setdelegate(-2,GoPage@:bind(220)).         //CANCEL
		       				
	}
}

// PAGE 231 - executing step run basic
function InitTermstepExeBas {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
        mylabels:setlabel(1," Continue ").
		mylabels:setlabel(2," Next step").
		mylabels:setlabel(3, "   Reset  ").
		
		
				 
		mybuttons:setdelegate(1,GoPage@:bind(230)).
		mybuttons:setdelegate(2,steprun@).
		mybuttons:setdelegate(3,resetbasicstate@).
		mybuttons:setdelegate(-2,GoPage@:bind(220)).         //CANCEL
		       				
	}
}

// PAGE 237 - 277 edit  prog 0 cursore
function InitTermEditBas0cu {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
        
		SetBasicSymbolsE(x).
  
		mylabels:setlabel(7,"     [...]  ").
		mylabels:setlabel(8,"   Canc   ").
		mylabels:setlabel(9,"  . , ! ? ").
		 
		mybuttons:setdelegate(7,cicleset@).
		mybuttons:setdelegate(8,T9CancE@).
		mybuttons:setdelegate(9,punct2e@).
       		
		mybuttons:setdelegate(-2,GoPage@:bind(PageBack())). //CANCEL
		mybuttons:setdelegate(-3,CuUp@).             //UP
		mybuttons:setdelegate(-4,CuDown@).           //DOWN
		mybuttons:setdelegate(-5,CuLeft@).           //LEFT
		mybuttons:setdelegate(-6,CuRight@).          //RIGHT
		mybuttons:setdelegate(-1,InsertLineKST@).  //ENTER
		
	}
}

// PAGE 238 - 278 edit prog 1 cursore
function InitTermEditBas1cu {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
        
		SetBasicSymbolsLowerE(x).
  
		mylabels:setlabel(7,"     [...]  ").
		mylabels:setlabel(8,"   Canc   ").
		mylabels:setlabel(9,"  . , ! ? ").
		 
		mybuttons:setdelegate(7,cicleset@).
		mybuttons:setdelegate(8,T9CancE@).
		mybuttons:setdelegate(9,punct2e@).
       		
		mybuttons:setdelegate(-2,GoPage@:bind(PageBack())). //CANCEL
		mybuttons:setdelegate(-3,CuUp@).             //UP
		mybuttons:setdelegate(-4,CuDown@).           //DOWN
		mybuttons:setdelegate(-5,CuLeft@).           //LEFT
		mybuttons:setdelegate(-6,CuRight@).          //RIGHT
		mybuttons:setdelegate(-1,InsertLineKST@).  //ENTER
		
	}
}

// PAGE 239 - 279 edit prog 2 cursore
function InitTermEditBas2cu{
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
        
		SetBasicSymbolsNumbersE(x).
  
		mylabels:setlabel(7,"     [...]  ").
		mylabels:setlabel(8,"  - / : ' ").
		mylabels:setlabel(9,"  . , ! ? ").
		mylabels:setlabel(10,"  < > @ / ").
		mylabels:setlabel(11,"  "" * ^ $ ").
		
		mylabels:setlabel(13,"  ( ) = #   ").
	
		mybuttons:setdelegate(7,cicleset@).
		mybuttons:setdelegate(8,punct1e@).
		mybuttons:setdelegate(9,punct2e@).
		mybuttons:setdelegate(10,punct5e@).
		mybuttons:setdelegate(11,punct4e@).
		
		mybuttons:setdelegate(13,parentesiE@).
  
		mybuttons:setdelegate(-2,GoPage@:bind(PageBack())). //CANCEL
		mybuttons:setdelegate(-3,CuUp@).             //UP
		mybuttons:setdelegate(-4,CuDown@).           //DOWN
		mybuttons:setdelegate(-5,CuLeft@).           //LEFT
		mybuttons:setdelegate(-6,CuRight@).          //RIGHT
		mybuttons:setdelegate(-1,InsertLineKST@).  //ENTER
		
	}
}


// PAGE 240 edit basic prog 3 cursore
function InitTermEditBas3cu {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
       
		
		mylabels:setlabel(0, "     PRINT  ").
		mylabels:setlabel(1, "    CLS   ").
		mylabels:setlabel(2, "    END   ").
		mylabels:setlabel(3, "   WAIT   ").
		mylabels:setlabel(4, "    REM   ").
		mylabels:setlabel(5, " Clr Line ").
		mylabels:setlabel(7, "     [...]  ").
		mylabels:setlabel(8, " PRINT """" ").
		mylabels:setlabel(9, "   LOCATE  ").
		mylabels:setlabel(10,"   GOTO   ").
		 
		mybuttons:setdelegate(0,T9strE@:bind("PRINT ")).
		mybuttons:setdelegate(1,T9strE@:bind("CLS")).
		mybuttons:setdelegate(2,T9strE@:bind("END")).
		mybuttons:setdelegate(3,T9strE@:bind("WAIT ")).
		mybuttons:setdelegate(4,T9strE@:bind("REM ")).
		mybuttons:setdelegate(5,ClearLine@).
		mybuttons:setdelegate(7,cicleset@).
		mybuttons:setdelegate(8,T9strCurE@:bind("PRINT """"",7)).
		mybuttons:setdelegate(9,T9strE@:bind("LOCATE ")).
		mybuttons:setdelegate(10,T9strE@:bind("GOTO ")).
       		
		mybuttons:setdelegate(-2,GoPage@:bind(PageBack())). //CANCEL
		mybuttons:setdelegate(-3,CuUp@).             //UP
		mybuttons:setdelegate(-4,CuDown@).           //DOWN
		mybuttons:setdelegate(-5,CuLeft@).           //LEFT
		mybuttons:setdelegate(-6,CuRight@).          //RIGHT
		mybuttons:setdelegate(-1,InsertLineKST@).  //ENTER
		
	}
}

// PAGE 280 edit ks prog 3 cursore
function InitTermEditKS3cu {
	parameter monitors.
	FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
		
		ClearTerminal(x).
       
		mylabels:setlabel(0, "     PRINT  ").
		mylabels:setlabel(1, " FUNCTION ").
		mylabels:setlabel(2, " CLEARS.. ").
		mylabels:setlabel(3, "  WAIT()  ").
		mylabels:setlabel(4, "  GLOBAL  ").
		mylabels:setlabel(5, " Clr Line ").
		mylabels:setlabel(7, "     [...]  ").
		mylabels:setlabel(8, " PRINT """" ").
		 
		mybuttons:setdelegate(0,T9strE@:bind("PRINT")).
		mybuttons:setdelegate(1,T9strE@:bind("FUNCTION")).
		mybuttons:setdelegate(2,T9strE@:bind("CLEARSCREEN")).
		mybuttons:setdelegate(3,T9strCurE@:bind("WAIT()",5)).
		mybuttons:setdelegate(4,T9strE@:bind("GLOBAL")).
		mybuttons:setdelegate(5,ClearLine@).
		mybuttons:setdelegate(7,cicleset@).
		mybuttons:setdelegate(8,T9strCurE@:bind("PRINT""""",6)).
       		
		mybuttons:setdelegate(-2,GoPage@:bind(PageBack())). //CANCEL
		mybuttons:setdelegate(-3,CuUp@).             //UP
		mybuttons:setdelegate(-4,CuDown@).           //DOWN
		mybuttons:setdelegate(-5,CuLeft@).           //LEFT
		mybuttons:setdelegate(-6,CuRight@).          //RIGHT
		mybuttons:setdelegate(-1,InsertLineKST@).  //ENTER
		
	}
}