/*
NOTE: This code was written using Microsoft's Visual C++ v 6.0.
I don't know how well it will work with other compilers.

You also will need the Direct X 7.0 SDK, and to install the lib
and include files from the SDK to your C++ lib and include
folders. You can get the DirectX 7.0 SDK (large!) here:
http://download.microsoft.com/download/win98SE/SDK/7.0/W9X/EN-US/dx7sdk-700.1.exe


========================
A* Pathfinder - The Maze
========================
By Patrick Lester, pwlester@policyalmanac.org

A fuller implementation than the Basics demo. This is the 
template you will want to work from if you want to write or
customize your own version of A*.

Instructions
------------
- Press enter to start pathfinding mode
- left or right click anywhere on the map to make the smiley
  go there.
- search times are printed in the upper left hand coner
- map may be edited by pressing enter button to toggle to
  map editing mode and left clicking on the map.

*/


#include "aStarLibrary.h"

//-----------------------------------------------------------------------------
// Global variables and constants
//-----------------------------------------------------------------------------

char smileyActivated = 0;

int searchTime, g_showDirections=0; 


//-----------------------------------------------------------------------------
// Function Prototypes: where necessary
//-----------------------------------------------------------------------------
void nextstep(void);


//-----------------------------------------------------------------------------
// Name: nextstep
// Desc: returns 
//-----------------------------------------------------------------------------
void nextstep(){
    
}

