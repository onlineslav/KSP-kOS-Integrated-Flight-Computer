@lazyglobal off.

//svuota tutti i bottoni, etichette, funzioni delegate ecc.
function ClearTerminal {
	    
		parameter x.

		set mybuttons:currentmonitor to x. 
		set mylabels:currentmonitor to x.
		set myflags:currentmonitor to x.   
    
		mylabels:setlabel(0,"            ").
		mylabels:setlabel(1,"          ").
		mylabels:setlabel(2,"          ").
		mylabels:setlabel(3,"          ").
		mylabels:setlabel(4,"          ").
		mylabels:setlabel(5,"          ").
		mylabels:setlabel(6,"    XXXX    ").   //assegnamento coatto a EXIT
		mylabels:setlabel(7,"            ").
		mylabels:setlabel(8,"          ").
		mylabels:setlabel(9,"          ").
		mylabels:setlabel(10,"          ").
		mylabels:setlabel(11,"          ").
		mylabels:setlabel(12,"          ").
		mylabels:setlabel(13,"            ").
  
		mybuttons:setdelegate(0,MNOP@). 
		mybuttons:setdelegate(1,MNOP@). 
		mybuttons:setdelegate(2,MNOP@).
		mybuttons:setdelegate(3,MNOP@).  
		mybuttons:setdelegate(4,MNOP@).
		mybuttons:setdelegate(5,MNOP@).
		mybuttons:setdelegate(7,MNOP@).
		mybuttons:setdelegate(8,MNOP@).
		mybuttons:setdelegate(9,MNOP@).
		mybuttons:setdelegate(10,MNOP@).
		mybuttons:setdelegate(11,MNOP@).
		mybuttons:setdelegate(12,MNOP@).
		mybuttons:setdelegate(13,MNOP@).
  
		mybuttons:setdelegate(-1,MNOP@).          //ENTER
		mybuttons:setdelegate(-2,MNOP@).          //CANCEL
		mybuttons:setdelegate(-3,MNOP@).          //UP
		mybuttons:setdelegate(-4,MNOP@).          //DOWN
		mybuttons:setdelegate(-5,MNOP@).          //LEFT
		mybuttons:setdelegate(-6,MNOP@).          //RIGHT
 
}



function SetBasicSymbolsE {
	parameter x.//monitors.
	//FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
  
		set mybuttons:currentmonitor to x. 
		set mylabels:currentmonitor to x.
		set myflags:currentmonitor to x.   
  
		mylabels:setlabel(0,"     A B C  ").
		mylabels:setlabel(1,"   D E F  ").
		mylabels:setlabel(2,"   G H I  ").
		mylabels:setlabel(3,"   J K L  ").
		mylabels:setlabel(4,"   Space  ").
		mylabels:setlabel(5,"Back Space").
		

		mylabels:setlabel(10,"   M N O  ").
		mylabels:setlabel(11,"  P Q R S ").
		mylabels:setlabel(12,"   T U V  ").
		mylabels:setlabel(13,"  W X Y Z   ").
  
		mybuttons:setdelegate(0,T9ABCe@). 
		mybuttons:setdelegate(1,T9DEFe@). 
		mybuttons:setdelegate(2,T9GHIe@).
		mybuttons:setdelegate(3,T9JKLe@).  
		mybuttons:setdelegate(4,T9spaceE@).
		mybuttons:setdelegate(5,T9backsE@).
	
		mybuttons:setdelegate(10,T9MNOe@).
		mybuttons:setdelegate(11,T9PQRSe@).
		mybuttons:setdelegate(12,T9TUVe@).
		mybuttons:setdelegate(13,T9WXYZe@).
	//}
}





function SetBasicSymbolsLowerE {
	parameter x.//monitors.
	//FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
  
		set mybuttons:currentmonitor to x. 
		set mylabels:currentmonitor to x.
		set myflags:currentmonitor to x.   
  
		mylabels:setlabel(0,"     a b c  ").
		mylabels:setlabel(1,"   d e f  ").
		mylabels:setlabel(2,"   g h i  ").
		mylabels:setlabel(3,"   j k l  ").
		
		mylabels:setlabel(4,"   Space  ").
		mylabels:setlabel(5,"Back Space").
		
		mylabels:setlabel(10,"   m n o  ").
		mylabels:setlabel(11,"  p q r s ").
		mylabels:setlabel(12,"   t u v  ").
		mylabels:setlabel(13,"  w x y z   ").
  
		mybuttons:setdelegate(0,T9ABCle@). 
		mybuttons:setdelegate(1,T9DEFle@). 
		mybuttons:setdelegate(2,T9GHIle@).
		mybuttons:setdelegate(3,T9JKLle@).  
		mybuttons:setdelegate(4,T9spaceE@).
		mybuttons:setdelegate(5,T9backsE@).
		
		mybuttons:setdelegate(10,T9MNOle@).
		mybuttons:setdelegate(11,T9PQRSle@).
		mybuttons:setdelegate(12,T9TUVle@).
		mybuttons:setdelegate(13,T9WXYZle@).
}



function SetBasicSymbolsNumbersE {
	parameter x.//monitors.
	//FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
  
		set mybuttons:currentmonitor to x. 
		set mylabels:currentmonitor to x.
		set myflags:currentmonitor to x.   
		
		mylabels:setlabel(0,"     1 2 3  ").
		mylabels:setlabel(1,"   4 5 6  ").
		mylabels:setlabel(2,"   7 8 9  ").
		mylabels:setlabel(3,"   0 + -  ").
	
		mylabels:setlabel(4,"   Space  ").
		mylabels:setlabel(5,"Back Space").
		
		mybuttons:setdelegate(0,T9123e@). 
		mybuttons:setdelegate(1,T9456e@). 
		mybuttons:setdelegate(2,T9789e@).
		mybuttons:setdelegate(3,T90signsE@).  
		mybuttons:setdelegate(4,T9spaceE@).
		mybuttons:setdelegate(5,T9backsE@).
}



function SetNumbersOnlyE {
	parameter x.//monitors.
	//FROM {local x is 0.} UNTIL x = monitors STEP {set x to x+1.} DO {
  
		set mybuttons:currentmonitor to x. 
		set mylabels:currentmonitor to x.
		set myflags:currentmonitor to x.   
		
		mylabels:setlabel(0,"       1    ").
		mylabels:setlabel(1,"     2    ").
		mylabels:setlabel(2,"     3    ").
		mylabels:setlabel(3,"     4    ").
		mylabels:setlabel(4,"     5    ").
		mylabels:setlabel(5,"Back Space").
		mylabels:setlabel(7,"       6    ").
		mylabels:setlabel(8,"     7    ").
		mylabels:setlabel(9,"     8    ").
		mylabels:setlabel(10,"     9    ").
		mylabels:setlabel(11,"     0    ").
		mylabels:setlabel(12,"     .    ").
		mylabels:setlabel(13,"     -      ").
		
		mybuttons:setdelegate(0,T91e@). 
		mybuttons:setdelegate(1,T92e@). 
		mybuttons:setdelegate(2,T93e@).
		mybuttons:setdelegate(3,T94e@).  
		mybuttons:setdelegate(4,T95e@).
		mybuttons:setdelegate(5,T9backsE@).
		mybuttons:setdelegate(7,T96e@).
		mybuttons:setdelegate(8,T97e@).
		mybuttons:setdelegate(9,T98e@).
		mybuttons:setdelegate(10,T99e@).
		mybuttons:setdelegate(11,T90e@).
		mybuttons:setdelegate(12,T9dotE@).
		mybuttons:setdelegate(13,T9minusE@).
		
		mybuttons:setdelegate(-1, TypeRealNumAccept@).   //ENTER
		//mybuttons:setdelegate(-2, MMAIN@).               //CANCEL
		//mybuttons:setdelegate(-5, T9LEFTE@).              //LEFT
	
	//}
}