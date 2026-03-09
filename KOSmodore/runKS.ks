@lazyGlobal off.

global emptyprog to list(). //it may be called "sourceprog". it contains the program as it is readed from file
global preprocprog to list(). //it contains the program withoyt the comments and the labels. informations about the labels for the GOTO command are stored in the GOTOInf list
global GOTOInf to list(list()). //it associate the position of GOTO line to the position where to jump
//global editedline to 0. // edited line of a text (up and down to scroll)
//noooo correggi! a fare list(list()) crei subliste vuote!

function RunKS {
	//RUNPATH( "/KOSmodore/kerboscript/001.ks", 1, 2 ). 
	parameter fname.
	RUNPATH( "/KOSmodore/kerboscript/" + fname). 
}


function preprocess {

//!!!!!!!!!!!!!!!!!!!!!!!!!
//   invece che caricare nella nuova lista solo le righe che non da eliminare
//   duplica la lista e semplicemente togli quelle da eliminare
//   puoi fare anche più funzioni tipo: togli commenti, togli righe vuote..
//!!!!!!!!!!!!!!!!!!!!!!!!!

	parameter prog.
	local pproced to list().
	local auxi to 0. //auxiliary nummber
	//local firstword to "".
	
	FROM {local x is 4.} UNTIL x = prog:LENGTH-1 STEP {set x to x+1.} DO {
		//set firstword to prog[x].
		
		//Remove REM comments
		if not(prog[x]:TOUPPER:startswith("REM ")) { 
		//	pproced:add(prog[x]).
		
		//Remove ' comments
			set auxi to prog[x]:FIND("'").
			if auxi = -1 {
				pproced:add(prog[x]).
			} else {
				pproced:add(prog[x]:REMOVE(auxi,(prog[x]:length-auxi))).
			}
		}
	}
	return pproced.
}

function ClearLine {
	set emptyprog[CuY] to "".
	print "                                        " at(0,CuY).
	set CuX to 0.
}

function initstextvar {
	emptyprog:clear.
	emptyprog:add("").   //otherwise he doesn't know where to draw the cursor
	ClsNoCur().
	
}

function NewBasFile {
	ClearKSFile().
	set cuy to 0.  //would be better run it once
	//set offsy to 1. //
	set cux to 0.
	GoPage(237).
}

function ClearKSFile {
	emptyprog:clear.
	// add also the header (version ecc.)
	emptyprog:add("").   //otherwise he doesn't know where to draw the cursor
	ClsNoCur().
	//set lind to 0.
	//GoPage(277).
}

function NewKSFile {
	ClearKSFile().
	set cuy to 0.  //would be better run it once
	//set offsy to 1. //
	set cux to 0.
	GoPage(277).
}





